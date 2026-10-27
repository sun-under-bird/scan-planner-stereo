#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;
using Clock = std::chrono::steady_clock;

class StereoDepth : public rclcpp::Node
{
public:
  StereoDepth() : Node("stereo_depth_node")
  {
    method_ = declare_parameter<std::string>("method", "bm");
    disparities_ = declare_parameter<int>("num_disparities", 96);
    block_ = declare_parameter<int>("block_size", 15);
    downsample_ = declare_parameter<int>("downsample", 1);
    threads_ = declare_parameter<int>("opencv_threads", 2);
    min_depth_ = declare_parameter<double>("min_depth", 0.3);
    max_depth_ = declare_parameter<double>("max_depth", 3.0);
    max_age_ = declare_parameter<double>("max_image_age", 0.2);
    const double pair_tolerance = declare_parameter<double>("max_pair_time_diff", 0.01);
    const int uniqueness = declare_parameter<int>("uniqueness_ratio", 10);
    const int texture = declare_parameter<int>("texture_threshold", 10);
    const int speckle = declare_parameter<int>("speckle_window_size", 50);
    if ((method_ != "bm" && method_ != "sgbm") || disparities_ < 16 || disparities_ % 16 ||
        block_ < 5 || block_ > 255 || block_ % 2 == 0 || downsample_ < 1 ||
        threads_ < 1 || min_depth_ <= 0 || max_depth_ <= min_depth_ || max_depth_ > 65.0 ||
        max_age_ <= 0 || pair_tolerance <= 0)
      throw std::runtime_error("Invalid stereo method, disparity, block, scale, threads or range");
    cv::setNumThreads(threads_);
    if (method_ == "bm")
    {
      auto bm = cv::StereoBM::create(disparities_, block_);
      bm->setTextureThreshold(texture);
      bm->setUniquenessRatio(uniqueness);
      matcher_ = bm;
    }
    else
      matcher_ = cv::StereoSGBM::create(0, disparities_, block_, 8 * block_ * block_,
          32 * block_ * block_, 1, 31, uniqueness, speckle, 2, cv::StereoSGBM::MODE_SGBM_3WAY);
    matcher_->setSpeckleWindowSize(speckle);
    matcher_->setSpeckleRange(2);
    matcher_->setDisp12MaxDiff(1);
    depth_pub_ = create_publisher<Image>("depth", rclcpp::SensorDataQoS().keep_last(2));
    info_pub_ = create_publisher<Info>("camera_info", rclcpp::SensorDataQoS());
    stats_pub_ = create_publisher<std_msgs::msg::String>("statistics", 1);
    left_info_sub_ = create_subscription<Info>("left/camera_info", rclcpp::SensorDataQoS(),
        [this](Info::ConstSharedPtr info) { std::lock_guard<std::mutex> lock(mutex_); left_info_ = info; });
    right_info_sub_ = create_subscription<Info>("right/camera_info", rclcpp::SensorDataQoS(),
        [this](Info::ConstSharedPtr info) { std::lock_guard<std::mutex> lock(mutex_); right_info_ = info; });
    auto qos = rclcpp::SensorDataQoS().keep_last(2).get_rmw_qos_profile();
    left_sub_.subscribe(this, "left/image_rect", qos);
    right_sub_.subscribe(this, "right/image_rect", qos);
    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(3), left_sub_, right_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(pair_tolerance));
    sync_->registerCallback(std::bind(&StereoDepth::pairCallback, this,
                                     std::placeholders::_1, std::placeholders::_2));
    stats_started_ = Clock::now();
    stats_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&StereoDepth::statistics, this));
    worker_ = std::thread(&StereoDepth::work, this);
    RCLCPP_INFO(get_logger(), "Stereo %s: %d disparities, block %d, scale 1/%d, OpenCV threads %d",
                method_.c_str(), disparities_, block_, downsample_, threads_);
  }

  ~StereoDepth() override
  {
    { std::lock_guard<std::mutex> lock(mutex_); stopping_ = true; }
    condition_.notify_one();
    if (worker_.joinable()) worker_.join();
  }

private:
  using Policy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  void pairCallback(Image::ConstSharedPtr left, Image::ConstSharedPtr right)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++pairs_;
    if (pending_left_) ++overwritten_;
    pending_left_ = left; pending_right_ = right;
    condition_.notify_one();
  }
  void work()
  {
    while (true)
    {
      Image::ConstSharedPtr left, right;
      Info::ConstSharedPtr li, ri;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return stopping_ || pending_left_; });
        if (stopping_) return;
        left.swap(pending_left_); right.swap(pending_right_);
        li = left_info_; ri = right_info_;
      }
      try { process(left, right, li, ri); }
      catch (const std::exception &error)
      {
        if (rclcpp::ok()) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                              "Rejected stereo pair: %s", error.what());
        std::lock_guard<std::mutex> lock(mutex_); ++rejected_;
      }
    }
  }

  void process(const Image::ConstSharedPtr &left, const Image::ConstSharedPtr &right,
               const Info::ConstSharedPtr &li, const Info::ConstSharedPtr &ri)
  {
    const auto start = Clock::now();
    if (!li || !ri) throw std::runtime_error("waiting for left/right CameraInfo");
    if (left->encoding != "mono8" || right->encoding != "mono8" ||
        left->width != right->width || left->height != right->height ||
        li->width != left->width || li->height != left->height ||
        ri->width != right->width || ri->height != right->height ||
        left->header.frame_id.empty() || li->header.frame_id != left->header.frame_id ||
        (ri->header.frame_id != right->header.frame_id && ri->header.frame_id != li->header.frame_id))
      throw std::runtime_error("need matching rectified mono8 images and CameraInfo");
    if (left->header.stamp.sec == 0 && left->header.stamp.nanosec == 0)
      throw std::runtime_error("zero exposure timestamp");
    const double age = (now() - rclcpp::Time(left->header.stamp)).seconds();
    if (age > max_age_ || age < -0.05) throw std::runtime_error("stale/future image timestamp");
    if (left->width % downsample_ || left->height % downsample_)
      throw std::runtime_error("image dimensions must be divisible by downsample");
    const double scale = 1.0 / downsample_;
    const double fx = li->p[0] * scale, fy = li->p[5] * scale;
    if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0 || fy <= 0 ||
        !std::isfinite(ri->p[0]) || ri->p[0] <= 0)
      throw std::runtime_error("invalid calibrated focal length");
    const double baseline = li->p[3] / li->p[0] - ri->p[3] / ri->p[0];
    const double cx = (li->p[2] + 0.5) * scale - 0.5;
    const double cy = (li->p[6] + 0.5) * scale - 0.5;
    const double principal_offset = (li->p[2] - ri->p[2]) * scale;
    if (!std::isfinite(baseline) || baseline <= 0.0 || !std::isfinite(cx) ||
        !std::isfinite(cy) || !std::isfinite(principal_offset))
      throw std::runtime_error("need a positive calibrated stereo baseline in right P");
    cv::Mat l = cv_bridge::toCvShare(left, "mono8")->image;
    cv::Mat r = cv_bridge::toCvShare(right, "mono8")->image;
    cv::Mat small_l, small_r;
    if (downsample_ != 1)
    {
      cv::resize(l, small_l, cv::Size(), scale, scale, cv::INTER_AREA);
      cv::resize(r, small_r, cv::Size(), scale, scale, cv::INTER_AREA);
      l = small_l; r = small_r;
    }
    if (l.cols <= disparities_ + block_ || l.rows <= block_)
      throw std::runtime_error("disparity range/block is too large for the working image");
    cv::Mat disparity, depth(l.size(), CV_16UC1, cv::Scalar(0));
    const auto match_start = Clock::now();
    matcher_->compute(l, r, disparity);
    const double match_ms = std::chrono::duration<double, std::milli>(Clock::now() - match_start).count();
    size_t valid = 0;
    for (int y = 0; y < l.rows; ++y)
    {
      const auto *d = disparity.ptr<int16_t>(y);
      auto *z = depth.ptr<uint16_t>(y);
      for (int x = 0; x < l.cols; ++x)
      {
        if (d[x] <= 0) continue;
        const double corrected = d[x] / 16.0 - principal_offset;
        if (corrected <= 0.0) continue;
        const double metres = fx * baseline / corrected;
        if (!std::isfinite(metres) || metres < min_depth_ || metres > max_depth_) continue;
        z[x] = static_cast<uint16_t>(std::lround(metres * 1000.0));
        ++valid;
      }
    }
    Info info;
    info.header = left->header;
    info.width = l.cols; info.height = l.rows;
    info.distortion_model = "plumb_bob";
    info.d.assign(5, 0.0);
    info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    info_pub_->publish(info);
    depth_pub_->publish(*cv_bridge::CvImage(left->header, "16UC1", depth).toImageMsg());
    const double elapsed = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    const double latency = (now() - rclcpp::Time(left->header.stamp)).seconds() * 1000.0;
    std::lock_guard<std::mutex> lock(mutex_);
    ++processed_;
    width_ = l.cols; height_ = l.rows;
    match_times_.push_back(match_ms); process_times_.push_back(elapsed);
    latencies_.push_back(latency); valid_ratios_.push_back(double(valid) / l.total());
  }

  static double mean(const std::vector<double> &values)
  {
    double sum = 0; for (double value : values) sum += value;
    return values.empty() ? 0.0 : sum / values.size();
  }
  static double percentile(std::vector<double> values, double q)
  {
    if (values.empty()) return 0;
    std::sort(values.begin(), values.end());
    return values[static_cast<size_t>((values.size() - 1) * q)];
  }
  void statistics()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto current = Clock::now();
    const double dt = std::chrono::duration<double>(current - stats_started_).count();
    std::ostringstream text;
    text << std::fixed << std::setprecision(3)
         << "{\"method\":\"" << method_ << "\",\"width\":" << width_ << ",\"height\":" << height_
         << ",\"threads\":" << threads_ << ",\"pairs\":" << pairs_ << ",\"processed\":" << processed_
         << ",\"overwritten\":" << overwritten_ << ",\"rejected\":" << rejected_
         << ",\"output_hz\":" << process_times_.size() / dt
         << ",\"match_ms_mean\":" << mean(match_times_)
         << ",\"process_ms_mean\":" << mean(process_times_)
         << ",\"process_ms_p50\":" << percentile(process_times_, 0.50)
         << ",\"process_ms_p95\":" << percentile(process_times_, 0.95)
         << ",\"latency_ms_mean\":" << mean(latencies_)
         << ",\"latency_ms_p95\":" << percentile(latencies_, 0.95)
         << ",\"valid_fraction\":" << mean(valid_ratios_) << "}";
    std_msgs::msg::String message; message.data = text.str(); stats_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "%s", message.data.c_str());
    match_times_.clear(); process_times_.clear(); latencies_.clear(); valid_ratios_.clear();
    stats_started_ = current;
  }

  std::string method_;
  int disparities_, block_, downsample_, threads_, width_{0}, height_{0};
  double min_depth_, max_depth_, max_age_;
  cv::Ptr<cv::StereoMatcher> matcher_;
  message_filters::Subscriber<Image> left_sub_, right_sub_;
  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;
  rclcpp::Subscription<Info>::SharedPtr left_info_sub_, right_info_sub_;
  rclcpp::Publisher<Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<Info>::SharedPtr info_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::thread worker_;
  bool stopping_{false};
  Info::ConstSharedPtr left_info_, right_info_;
  Image::ConstSharedPtr pending_left_, pending_right_;
  uint64_t pairs_{0}, processed_{0}, overwritten_{0}, rejected_{0};
  Clock::time_point stats_started_;
  std::vector<double> match_times_, process_times_, latencies_, valid_ratios_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<StereoDepth>()); }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("stereo_depth_node"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown();
  return 0;
}
