#include "plan_env/depth_obstacle_filter.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace scan_planner {
namespace {
VoxelKey key(const Eigen::Vector3d &p,double s) {
  return {static_cast<int>(std::floor(p.x()*(1.0/s))),
          static_cast<int>(std::floor(p.y()*(1.0/s))),
          static_cast<int>(std::floor(p.z()*(1.0/s)))};
}
struct Cell { std::vector<std::size_t> points; };
}
DepthObstacleFilter::DepthObstacleFilter(const DepthFilterConfig &c):config_(c) {
  if (!std::isfinite(c.voxel_size) || c.voxel_size<=0 ||
      !std::isfinite(c.tracking_resolution) || c.tracking_resolution<=0 ||
      !std::isfinite(c.obstacle_z_min) || !std::isfinite(c.obstacle_z_max) ||
      !std::isfinite(c.ground_z_min) || c.ground_z_min>=c.obstacle_z_min ||
      c.obstacle_z_min>=c.obstacle_z_max || c.min_points_per_column<1 ||
      c.min_voxels_per_cluster<1 || c.confirm_frames<1 ||
      !std::isfinite(c.retention_sec) || c.retention_sec<=0 ||
      !std::isfinite(c.max_confirmation_gap_sec) || c.max_confirmation_gap_sec<=0)
    throw std::invalid_argument("Invalid depth obstacle filter configuration");
}
void DepthObstacleFilter::reset() { tracks_.clear();frame_=0;last_stamp_=0; }
void DepthObstacleFilter::forget(const VoxelKey &cell) { tracks_.erase(cell); }

DepthFilterResult DepthObstacleFilter::process(
    const std::vector<Eigen::Vector3d> &points,
    const Eigen::Isometry3d &map_from_height,int64_t stamp_ns) {
  DepthFilterResult out;
  out.classes.assign(points.size(),DepthPointClass::Ignore);
  if (stamp_ns<=last_stamp_ || stamp_ns<=0) return out;
  out.accepted=true;last_stamp_=stamp_ns;++frame_;
  const auto height_from_map=map_from_height.inverse();
  std::unordered_map<VoxelKey,Cell,VoxelHash> cells;
  std::unordered_map<VoxelKey,std::size_t,VoxelHash> columns;
  for (std::size_t i=0;i<points.size();++i) {
    if (!points[i].allFinite()) continue;
    ++out.valid_points;
    const double z=(height_from_map*points[i]).z();
    if (z>=config_.ground_z_min && z<config_.obstacle_z_min) {
      out.classes[i]=DepthPointClass::Ground;++out.ground_points;
    } else if (z>=config_.obstacle_z_min && z<=config_.obstacle_z_max) {
      auto k=key(points[i],config_.voxel_size);
      cells[k].points.push_back(i);
      ++columns[{k.x,k.y,0}];
    } else ++out.height_rejected;
  }
  std::unordered_set<VoxelKey,VoxelHash> supported;
  for (const auto &c:cells) {
    if (columns.at({c.first.x,c.first.y,0})>=
        static_cast<std::size_t>(config_.min_points_per_column)) supported.insert(c.first);
    else out.density_rejected+=c.second.points.size();
  }
  std::unordered_set<VoxelKey,VoxelHash> visited, spatial, current;
  for (const auto &k:supported) {
    if (!visited.insert(k).second) continue;
    std::vector<VoxelKey> component{k};
    for (std::size_t i=0;i<component.size();++i) {
      const auto c=component[i];
      for (int x=-1;x<=1;++x) for (int y=-1;y<=1;++y) for (int z=-1;z<=1;++z) {
        if (x==0&&y==0&&z==0) continue;
        VoxelKey next{c.x+x,c.y+y,c.z+z};
        if (supported.count(next) && visited.insert(next).second) component.push_back(next);
      }
    }
    if (component.size()<static_cast<std::size_t>(config_.min_voxels_per_cluster))
      for (const auto &c:component) out.cluster_rejected+=cells.at(c).points.size();
    else for (const auto &c:component) {
      spatial.insert(c);
      for (auto i:cells.at(c).points) current.insert(key(points[i],config_.tracking_resolution));
    }
  }
  const int64_t retention=static_cast<int64_t>(config_.retention_sec*1e9);
  const int64_t gap=static_cast<int64_t>(config_.max_confirmation_gap_sec*1e9);
  for (auto it=tracks_.begin();it!=tracks_.end();) {
    if (stamp_ns-it->second.last_seen>retention ||
        (it->second.hits<config_.confirm_frames && !current.count(it->first)))
      it=tracks_.erase(it);
    else ++it;
  }
  std::unordered_set<VoxelKey,VoxelHash> confirmed;
  for (const auto &k:current) {
    auto found=tracks_.find(k);
    if (found==tracks_.end()) found=tracks_.emplace(k,Track{stamp_ns,frame_,1}).first;
    else {
      auto &t=found->second;
      if (t.hits<config_.confirm_frames)
        t.hits=(t.frame+1==frame_ && stamp_ns-t.last_seen<=gap) ? t.hits+1 : 1;
      t.last_seen=stamp_ns;t.frame=frame_;
    }
    if (found->second.hits>=config_.confirm_frames) confirmed.insert(k);
    out.protected_cells.push_back(k);
  }
  for (const auto &k:spatial) for (auto i:cells.at(k).points) {
    if (confirmed.count(key(points[i],config_.tracking_resolution))) {
      out.classes[i]=DepthPointClass::Obstacle;++out.obstacle_points;
    } else {out.classes[i]=DepthPointClass::Pending;++out.pending_points;}
  }
  return out;
}
}  // namespace scan_planner
