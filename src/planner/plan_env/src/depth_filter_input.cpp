#include "plan_env/grid_map.h"
#include <sstream>
#include <iomanip>

void GridMap::filterProjectedDepth()
{
  const auto start = std::chrono::steady_clock::now();
  // Shift/reset ring-buffer metadata before collecting this frame\'s confirmations.
  if (md_.proj_points_cnt > 0) updateSlidingMap(md_.ray_pos_);
  depth_filter_result_ = depth_filter_->process(md_.proj_points_, map_from_height_, depth_stamp_ns_);
  protected_depth_cells_.clear();
  if (!depth_filter_result_.accepted) { md_.proj_points_cnt = 0; return; }
  ++depth_filter_frames_;
  for (const auto &k : depth_filter_result_.protected_cells)
  {
    Eigen::Vector3i id(k.x,k.y,k.z);
    if (isInMap(id)) protected_depth_cells_.insert(toAddress(id));
  }
  depth_filter_ms_ = std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-start).count();
}
void GridMap::forgetDepthCell(int addr)
{
  obstacle_last_seen_.erase(addr);
  if (depth_filter_)
  {
    Eigen::Vector3i id;
    hashIdToGlobalIndex(addr,id);
    depth_filter_->forget({id.x(),id.y(),id.z()});
  }
}
void GridMap::expireDepthObstacles()
{
  const int64_t retention = static_cast<int64_t>(depth_filter_config_.retention_sec*1e9);
  std::vector<int> expired;
  for (const auto &cell : obstacle_last_seen_)
    if (depth_stamp_ns_-cell.second > retention && !protected_depth_cells_.count(cell.first))
      expired.push_back(cell.first);
  for (int addr : expired)
  {
    Eigen::Vector3i id;
    hashIdToGlobalIndex(addr,id);
    // Return expired cells to UNKNOWN, never fabricate observed-free evidence.
    applyOccupancyUpdate(id,mp_.clamp_min_log_-mp_.unknown_flag_);
    forgetDepthCell(addr);
    ++expired_obstacles_;
  }
}
void GridMap::publishDepthFilter()
{
  if (!depth_filter_result_.accepted) return;
  if (filtered_obstacles_pub_->get_subscription_count())
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int i=0;i<md_.proj_points_cnt;++i)
      if (depth_filter_result_.classes[i]==scan_planner::DepthPointClass::Obstacle)
      {
        const auto &p=md_.proj_points_[i];
        cloud.push_back(pcl::PointXYZ(p.x(),p.y(),p.z()));
      }
    sensor_msgs::msg::PointCloud2 msg;
    cloud.width=cloud.size();cloud.height=1;cloud.is_dense=true;
    pcl::toROSMsg(cloud,msg);msg.header.frame_id=mp_.frame_id_;
    msg.header.stamp=rclcpp::Time(depth_stamp_ns_);
    filtered_obstacles_pub_->publish(msg);
  }
  const auto now=std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now-filter_statistics_time_).count()<1.0) return;
  filter_statistics_time_=now;
  const auto &r=depth_filter_result_;
  std::ostringstream s;
  s << std::fixed << std::setprecision(3)
    << "{\"source_stamp_ns\":" << depth_stamp_ns_
    << ",\"frames\":" << depth_filter_frames_
    << ",\"valid_points\":" << r.valid_points
    << ",\"ground_points\":" << r.ground_points
    << ",\"height_rejected\":" << r.height_rejected
    << ",\"density_rejected\":" << r.density_rejected
    << ",\"cluster_rejected\":" << r.cluster_rejected
    << ",\"pending_points\":" << r.pending_points
    << ",\"obstacle_points\":" << r.obstacle_points
    << ",\"occupied_voxels\":" << obstacle_last_seen_.size()
    << ",\"expired_total\":" << expired_obstacles_
    << ",\"filter_ms\":" << depth_filter_ms_ << "}";
  std_msgs::msg::String msg;msg.data=s.str();filter_statistics_pub_->publish(msg);
}
