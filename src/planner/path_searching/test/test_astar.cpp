
#include <gtest/gtest.h>
#include <path_searching/dyn_a_star.h>
#include <cmath>

class AstarRegression : public ::testing::Test {
protected:
  std::shared_ptr<rclcpp::Node> node;
  GridMap::Ptr map;
  AStar search;
  void SetUp() override {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
      {"grid_map.resolution", 0.05},
      {"grid_map.sliding_map_size_x", 10.0}, {"grid_map.sliding_map_size_y", 10.0},
      {"grid_map.sliding_map_size_z", 2.0},
      {"grid_map.obstacles_inflation_z_up", 0.1}, {"grid_map.obstacles_inflation_z_down", 0.4},
      {"grid_map.double_cylinder_radius", 0.1}, {"grid_map.double_cylinder_offset", 0.0},
      {"grid_map.map_sliding_en", false}, {"grid_map.p_hit", 0.85},
      {"grid_map.p_miss", 0.3}, {"grid_map.p_min", 0.12},
      {"grid_map.p_max", 0.98}, {"grid_map.p_occ", 0.8}});
    node = std::make_shared<rclcpp::Node>("astar_regression", opts);
    map = std::make_shared<GridMap>(); map->initMap(node.get());
    search.initGridMap(map, Eigen::Vector3i(100,100,5));
  }
  void block(double x1,double x2,double y1,double y2) {
    for (double x=x1;x<=x2+1e-6;x+=0.05)
      for (double y=y1;y<=y2+1e-6;y+=0.05)
        map->setOccupied(Eigen::Vector3d(x,y,0.5));
  }
};

TEST_F(AstarRegression, FreeEndpointsAreNotShiftedByNegativeIndexTruncation) {
  const Eigen::Vector3d start(-0.8,0,0.5), end(0.8,0,0.5);
  ASSERT_EQ(search.AstarSearch(0.05,start,end), ASTAR_RET::SUCCESS);
  const auto path=search.getPath();
  ASSERT_GE(path.size(),2u);
  EXPECT_LT((path.front()-start).norm(),1e-8);
  EXPECT_LT((path.back()-end).norm(),1e-8);
}

TEST_F(AstarRegression, OccupiedEndUsesNearbySideExitInsteadOfUnboundedRay) {
  block(0.6,3.0,-0.05,0.05);
  const Eigen::Vector3d end(0.8,0,0.5);
  ASSERT_EQ(search.AstarSearch(0.05,Eigen::Vector3d(-0.8,0,0.5),end), ASTAR_RET::SUCCESS);
  const auto path=search.getPath();
  ASSERT_GE(path.size(),2u);
  EXPECT_LE((path.back()-end).norm(),0.5+1e-6);
  for (size_t i=1;i<path.size();++i) {
    const auto delta=path[i]-path[i-1];
    EXPECT_EQ(map->getInflateOccupancy(path[i],std::atan2(delta.y(),delta.x())),0);
  }
}

TEST_F(AstarRegression, BlockedEndpointsFailWithoutReturningPreviousPath) {
  ASSERT_EQ(search.AstarSearch(0.05,{-0.8,0,0.5},{0.8,0,0.5}), ASTAR_RET::SUCCESS);
  block(0.25,3.0,-1.0,1.0);
  EXPECT_EQ(search.AstarSearch(0.05,{-0.8,0,0.5},{0.8,0,0.5}), ASTAR_RET::INIT_ERR);
  EXPECT_TRUE(search.getPath().empty());
}
