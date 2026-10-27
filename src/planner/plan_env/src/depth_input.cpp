#include "plan_env/grid_map.h"
#include <tf2/exceptions.h>

bool GridMap::readDepthImage(const sensor_msgs::msg::Image::ConstSharedPtr &image)
{
  if (image->encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
      image->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                        "Depth encoding must be 16UC1 (scaled) or 32FC1 (metres)");
    return false;
  }
  if (!image->width || !image->height) return false;
  try
  {
    md_.depth_image_ = cv_bridge::toCvCopy(image, image->encoding)->image;
    return true;
  }
  catch (const cv_bridge::Exception &error)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                        "Rejected malformed depth image: %s", error.what());
    return false;
  }
}

void GridMap::processPendingDepth()
{
  if (!pending_depth_) return;
  const auto image = pending_depth_;
  const bool expired = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - pending_received_).count() > mp_.tf_wait_timeout_;
  auto reject = [this, expired](const char *reason) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                        "Waiting for depth input: %s", reason);
    if (expired)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                          "Dropped depth: %s", reason);
      pending_depth_.reset();
    }
  };
  if (mp_.use_camera_info_)
  {
    if (!camera_info_ || camera_info_->header.frame_id != image->header.frame_id ||
        camera_info_->width != image->width || camera_info_->height != image->height)
    {
      reject("CameraInfo must match depth dimensions and optical frame");
      return;
    }
    const auto &p = camera_info_->p;
    if (!std::isfinite(p[0]) || !std::isfinite(p[5]) || !std::isfinite(p[2]) ||
        !std::isfinite(p[6]) || p[0] <= 0.0 || p[5] <= 0.0)
    {
      reject("invalid rectified CameraInfo projection matrix");
      return;
    }
    mp_.fx_ = p[0]; mp_.fy_ = p[5]; mp_.cx_ = p[2]; mp_.cy_ = p[6];
  }
  try
  {
    // Exact image time, no latest-transform fallback and no executor-blocking wait.
    const auto transform = depth_tf_buffer_->lookupTransform(
        mp_.frame_id_, image->header.frame_id, rclcpp::Time(image->header.stamp));
    if (depth_filter_enabled_)
    {
      try {
        const auto reference = depth_tf_buffer_->lookupTransform(
            mp_.frame_id_, filter_height_frame_, rclcpp::Time(image->header.stamp));
        const auto &q = reference.transform.rotation;
        map_from_height_.linear() = Eigen::Quaterniond(q.w,q.x,q.y,q.z).normalized().toRotationMatrix();
        const auto &t = reference.transform.translation;
        map_from_height_.translation() = Eigen::Vector3d(t.x,t.y,t.z);
      } catch (const tf2::TransformException &) {
        reject("no TF from depth filter height frame at exposure time");
        return;
      }
    }
    auto pose = std::make_shared<nav_msgs::msg::Odometry>();
    pose->header = image->header;
    pose->header.frame_id = mp_.frame_id_;
    pose->child_frame_id = image->header.frame_id;
    pose->pose.pose.position.x = transform.transform.translation.x;
    pose->pose.pose.position.y = transform.transform.translation.y;
    pose->pose.pose.position.z = transform.transform.translation.z;
    pose->pose.pose.orientation = transform.transform.rotation;
    pending_depth_.reset();
    depthPoseCallback(image, pose);
  }
  catch (const tf2::TransformException &)
  {
    reject("no TF from depth optical frame to planning frame at exposure time");
  }
}
