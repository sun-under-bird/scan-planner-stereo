#include <gtest/gtest.h>
#include <plan_env/depth_obstacle_filter.h>
#include <limits>
using namespace scan_planner;
namespace {
std::vector<Eigen::Vector3d> patch(double z=0.2) {
  std::vector<Eigen::Vector3d> p;
  for (int x=0;x<2;++x) for (int y=0;y<2;++y) for (int i=0;i<8;++i)
    p.emplace_back(1.011+x*.05+i*.00001,.011+y*.05,z);
  return p;
}
constexpr int64_t t=1000000000LL,dt=66666667LL;
}
TEST(DepthFilter, RequiresTwoDistinctFrames) {
  DepthObstacleFilter f;auto p=patch();
  EXPECT_EQ(f.process(p,Eigen::Isometry3d::Identity(),t).pending_points,p.size());
  EXPECT_FALSE(f.process(p,Eigen::Isometry3d::Identity(),t).accepted);
  EXPECT_EQ(f.process(p,Eigen::Isometry3d::Identity(),t+dt).obstacle_points,p.size());
}
TEST(DepthFilter, GroundRemainsValidWithoutObstacleHits) {
  DepthObstacleFilter f;
  auto r=f.process({{1,0,0},{1,.1,-.05},{1,.2,.079}},Eigen::Isometry3d::Identity(),t);
  EXPECT_EQ(r.valid_points,3u);EXPECT_EQ(r.ground_points,3u);EXPECT_EQ(r.obstacle_points,0u);
}
TEST(DepthFilter, UsesGroundReferenceInsteadOfOdomZZero) {
  DepthObstacleFilter f;auto tf=Eigen::Isometry3d::Identity();tf.translation().z()=-.36;
  auto p=patch(0);p.emplace_back(1,0,-.36);
  auto r=f.process(p,tf,t);
  EXPECT_EQ(r.ground_points,1u);EXPECT_EQ(r.pending_points,p.size()-1);
}
TEST(DepthFilter, AppliesFullHeightFrameTransform) {
  DepthObstacleFilter f;auto tf=Eigen::Isometry3d::Identity();
  tf.linear()=Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitY()).toRotationMatrix();
  auto p=patch();for(auto &v:p)v=tf*v;
  auto r=f.process(p,tf,t);EXPECT_EQ(r.height_rejected,0u);
}
TEST(DepthFilter, RejectsSparseAndIsolatedArtifacts) {
  DepthObstacleFilter f;auto p=patch();p.resize(7);
  EXPECT_EQ(f.process(p,Eigen::Isometry3d::Identity(),t).density_rejected,7u);
  p=patch();p.resize(8);
  EXPECT_EQ(f.process(p,Eigen::Isometry3d::Identity(),t+dt).cluster_rejected,8u);
}
TEST(DepthFilter, PreservesMultipleHeightsInSamePlanarColumn) {
  DepthObstacleFilter f;std::vector<Eigen::Vector3d> p;
  for(int z=0;z<4;++z)for(int i=0;i<2;++i)p.emplace_back(1.011+i*.00001,.011,.11+z*.05);
  auto r=f.process(p,Eigen::Isometry3d::Identity(),t);
  EXPECT_EQ(r.pending_points,8u);
  r=f.process(p,Eigen::Isometry3d::Identity(),t+dt);
  EXPECT_EQ(r.obstacle_points,8u);EXPECT_EQ(r.protected_cells.size(),4u);
}
TEST(DepthFilter, KeepsSupportedSmallObstacleAndHeightBoundaries) {
  DepthObstacleFilter f;
  auto p=patch(.08),q=patch(.65);p.insert(p.end(),q.begin(),q.end());
  EXPECT_EQ(f.process(p,Eigen::Isometry3d::Identity(),t).pending_points,p.size());
  EXPECT_EQ(f.process(p,Eigen::Isometry3d::Identity(),t+dt).obstacle_points,p.size());
}
TEST(DepthFilter, InvalidPointsDoNotBecomeGroundOrObstacles) {
  DepthObstacleFilter f;
  auto r=f.process({{NAN,0,0},{1,0,-.2},{1,0,.8}},Eigen::Isometry3d::Identity(),t);
  EXPECT_EQ(r.valid_points,2u);EXPECT_EQ(r.height_rejected,2u);EXPECT_EQ(r.ground_points,0u);
}
TEST(DepthFilter, MissingFrameBreaksUnconfirmedRun) {
  DepthObstacleFilter f;auto p=patch();
  f.process(p,Eigen::Isometry3d::Identity(),t);
  f.process({},Eigen::Isometry3d::Identity(),t+dt);
  EXPECT_EQ(f.process(p,Eigen::Isometry3d::Identity(),t+2*dt).pending_points,p.size());
}
TEST(DepthFilter, LongGapRestartsConfirmation) {
  DepthObstacleFilter f;auto p=patch();f.process(p,Eigen::Isometry3d::Identity(),t);
  EXPECT_EQ(f.process(p,Eigen::Isometry3d::Identity(),t+600000000LL).pending_points,p.size());
}
TEST(DepthFilter, ConfirmedMemoryExpiresAndCanBeCleared) {
  DepthObstacleFilter f;auto p=patch();
  f.process(p,Eigen::Isometry3d::Identity(),t);
  f.process(p,Eigen::Isometry3d::Identity(),t+dt);
  EXPECT_EQ(f.process(p,Eigen::Isometry3d::Identity(),t+dt*3).obstacle_points,p.size());
  EXPECT_EQ(f.process(p,Eigen::Isometry3d::Identity(),t+6000000000LL).pending_points,p.size());
  f.reset();
  EXPECT_EQ(f.process(p,Eigen::Isometry3d::Identity(),t).pending_points,p.size());
}
TEST(DepthFilter, TrackingMatchesMapResolutionAndClearingForgetsConfirmation) {
  DepthFilterConfig c;c.tracking_resolution=.1;
  DepthObstacleFilter f(c);auto p=patch();
  f.process(p,Eigen::Isometry3d::Identity(),t);
  auto r=f.process(p,Eigen::Isometry3d::Identity(),t+dt);
  EXPECT_EQ(r.protected_cells.size(),1u);
  f.forget(r.protected_cells.front());
  EXPECT_EQ(f.process(p,Eigen::Isometry3d::Identity(),t+2*dt).pending_points,p.size());
}
TEST(DepthFilter, RejectsInvalidConfiguration) {
  DepthFilterConfig c;c.obstacle_z_min=.7;EXPECT_THROW(DepthObstacleFilter f(c),std::invalid_argument);
}

TEST(DepthFilter, TrackingKeysMatchMapIndexAtDecimalBoundaries) {
  DepthFilterConfig c;c.min_points_per_column=1;c.min_voxels_per_cluster=1;
  DepthObstacleFilter f(c);
  const std::vector<Eigen::Vector3d> p{{2.8,-2.8,.3}};
  auto r=f.process(p,Eigen::Isometry3d::Identity(),t);
  ASSERT_EQ(r.protected_cells.size(),1u);
  EXPECT_EQ(r.protected_cells[0].x,static_cast<int>(std::floor(2.8*20.0)));
  EXPECT_EQ(r.protected_cells[0].y,static_cast<int>(std::floor(-2.8*20.0)));
}
