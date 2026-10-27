#ifndef SCAN_DEPTH_OBSTACLE_FILTER_H
#define SCAN_DEPTH_OBSTACLE_FILTER_H
#include <Eigen/Geometry>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace scan_planner {
struct VoxelKey {
  int x, y, z;
  bool operator==(const VoxelKey &o) const { return x==o.x && y==o.y && z==o.z; }
};
struct VoxelHash {
  std::size_t operator()(const VoxelKey &k) const {
    std::size_t h=std::hash<int>{}(k.x);
    h^=std::hash<int>{}(k.y)+0x9e3779b9+(h<<6)+(h>>2);
    return h^(std::hash<int>{}(k.z)+0x9e3779b9+(h<<6)+(h>>2));
  }
};
struct DepthFilterConfig {
  double voxel_size{0.05}, tracking_resolution{0.05};
  double obstacle_z_min{0.08}, obstacle_z_max{0.65}, ground_z_min{-0.10};
  int min_points_per_column{8}, min_voxels_per_cluster{4}, confirm_frames{2};
  double retention_sec{5.0}, max_confirmation_gap_sec{0.5};
};
enum class DepthPointClass : uint8_t { Ignore, Ground, Pending, Obstacle };
struct DepthFilterResult {
  bool accepted{false};
  std::vector<DepthPointClass> classes;
  std::vector<VoxelKey> protected_cells;
  std::size_t valid_points{0}, ground_points{0}, height_rejected{0};
  std::size_t density_rejected{0}, cluster_rejected{0}, pending_points{0}, obstacle_points{0};
};
// Pure geometry and source-time state; does not depend on ROS or device drivers.
class DepthObstacleFilter {
 public:
  explicit DepthObstacleFilter(const DepthFilterConfig &config = {});
  DepthFilterResult process(const std::vector<Eigen::Vector3d> &points,
                            const Eigen::Isometry3d &map_from_height, int64_t stamp_ns);
  void reset();
  void forget(const VoxelKey &cell);
 private:
  struct Track { int64_t last_seen; uint64_t frame; int hits; };
  DepthFilterConfig config_;
  uint64_t frame_{0};
  int64_t last_stamp_{0};
  std::unordered_map<VoxelKey,Track,VoxelHash> tracks_;
};
}  // namespace scan_planner
#endif
