#include <gtest/gtest.h>
#include <plan_env/grid_map.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <chrono>
#include <thread>

class DepthMapTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    // Isolate even when invoked through ctest in a production shell.
    setenv("ROS_DOMAIN_ID","229",1);
    setenv("RMW_IMPLEMENTATION","rmw_cyclonedds_cpp",1);
    unsetenv("ROS_LOCALHOST_ONLY");
    setenv("CYCLONEDDS_URI","<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer address=\"127.0.0.1\"/></Peers></Discovery></Domain></CycloneDDS>",1);
    if(!rclcpp::ok()) rclcpp::init(0,nullptr);
  }
  static void TearDownTestSuite() { if(rclcpp::ok()) rclcpp::shutdown(); }
  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<GridMap> map;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_sub;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf;
  std::vector<bool> ready;
  std::vector<sensor_msgs::msg::PointCloud2> clouds;
  std::string world,optical,ground;
  bool dynamic_ground{false},missing_ground{false},sliding{false};
  double retention{5.0};
  void start() {
    if(!rclcpp::ok()) rclcpp::init(0,nullptr);
    static int id=0;const auto suffix=std::to_string(++id);
    world="map_"+suffix;optical="optical_"+suffix;ground="ground_"+suffix;
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
      {"grid_map.resolution",.05},{"grid_map.sliding_map_size_x",6.0},
      {"grid_map.sliding_map_size_y",6.0},{"grid_map.sliding_map_size_z",2.0},
      {"grid_map.ground_height",-1.0},{"grid_map.map_sliding_en",sliding},
      {"grid_map.map_sliding_thresh",.1},
      {"grid_map.obstacles_inflation_z_up",.1},{"grid_map.obstacles_inflation_z_down",.1},
      {"grid_map.double_cylinder_radius",.025},{"grid_map.double_cylinder_offset",0.0},
      {"grid_map.sensor_type",std::string("depth")},{"grid_map.depth_pose_source",std::string("tf")},
      {"grid_map.frame_id",world},{"grid_map.need_extrinsic",false},
      {"grid_map.use_camera_info",true},{"grid_map.invalid_depth_is_free",false},
      {"grid_map.skip_pixel",1},{"grid_map.depth_filter_margin",0},
      {"grid_map.depth_filter_mindist",.3},{"grid_map.depth_filter_maxdist",3.0},
      {"grid_map.k_depth_scaling_factor",1000.0},{"grid_map.max_ray_length",3.0},
      {"grid_map.p_hit",.85},{"grid_map.p_miss",.3},{"grid_map.p_min",.12},
      {"grid_map.p_max",.98},{"grid_map.p_occ",.8},
      {"grid_map.depth_filter.enabled",true},{"grid_map.depth_filter.height_frame",ground},
      {"grid_map.depth_filter.retention_sec",retention}});
    node=std::make_shared<rclcpp::Node>("filter_map_"+suffix,"/filter_"+suffix,opts);
    map=std::make_unique<GridMap>();map->initMap(node.get());executor.add_node(node);
    depth_pub=node->create_publisher<sensor_msgs::msg::Image>("depth",rclcpp::SensorDataQoS());
    info_pub=node->create_publisher<sensor_msgs::msg::CameraInfo>("depth_camera_info",rclcpp::SensorDataQoS());
    ready_sub=node->create_subscription<std_msgs::msg::Bool>("grid_map/depth_ready",1,
      [this](std_msgs::msg::Bool::ConstSharedPtr m){ready.push_back(m->data);});
    filtered_sub=node->create_subscription<sensor_msgs::msg::PointCloud2>("grid_map/filtered_obstacles",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){clouds.push_back(*m);});
    static_tf=std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
    publishTransforms(0);
    spin(.25);
  }
  void publishTransforms(double x) {
    geometry_msgs::msg::TransformStamped camera;
    camera.header.frame_id=world;camera.child_frame_id=optical;camera.header.stamp=node->now();
    camera.transform.translation.x=x;camera.transform.translation.z=.5;
    camera.transform.rotation.x=-.5;camera.transform.rotation.y=.5;
    camera.transform.rotation.z=-.5;camera.transform.rotation.w=.5;
    std::vector<geometry_msgs::msg::TransformStamped> ts{camera};
    if(!missing_ground&&!dynamic_ground) {
      geometry_msgs::msg::TransformStamped floor;
      floor.header.frame_id=world;floor.child_frame_id=ground;floor.header.stamp=node->now();
      floor.transform.rotation.w=1;ts.push_back(floor);
    }
    static_tf->sendTransform(ts);
  }
  void spin(double sec) {
    auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(sec);
    while(std::chrono::steady_clock::now()<end) {
      executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  sensor_msgs::msg::Image image(float z) {
    sensor_msgs::msg::Image m;m.header.frame_id=optical;m.header.stamp=node->now();
    m.width=80;m.height=60;m.encoding="32FC1";m.step=m.width*sizeof(float);
    m.data.resize(m.height*m.step);std::fill(reinterpret_cast<float*>(m.data.data()),
      reinterpret_cast<float*>(m.data.data())+m.width*m.height,z);return m;
  }
  sensor_msgs::msg::Image floorImage() {
    auto m=image(NAN);auto *d=reinterpret_cast<float*>(m.data.data());
    for(int v=48;v<60;++v)for(int u=0;u<80;++u)d[v*80+u]=.5f/((v-30)/100.f);
    return m;
  }
  void feed(sensor_msgs::msg::Image m,int count=1,bool fresh=true) {
    for(int i=0;i<count;++i) {
      if(fresh)m.header.stamp=node->now();
      sensor_msgs::msg::CameraInfo c;c.header=m.header;c.width=m.width;c.height=m.height;
      c.p={100.,0.,40.,0.,0.,100.,30.,0.,0.,0.,1.,0.};
      info_pub->publish(c);depth_pub->publish(m);spin(.085);
    }
  }
  void wall(){feed(image(1),2);}
  Eigen::Vector3d point(double z=.3){return {1.01,.02,z};}
  void TearDown() override {
    if(node)executor.remove_node(node);
    map.reset();static_tf.reset();node.reset();
  }
};

TEST_F(DepthMapTest, TwoFramesAndDuplicateSourceTime) {
  start();auto m=image(1);feed(m,1,false);EXPECT_EQ(map->getOccupancy(point()),0);
  feed(m,1,false);EXPECT_EQ(map->getOccupancy(point()),0);
  feed(image(1));EXPECT_EQ(map->getOccupancy(point()),1);
  ASSERT_FALSE(clouds.empty());EXPECT_GT(clouds.back().width,0u);
  EXPECT_EQ(clouds.back().header.frame_id,world);
}
TEST_F(DepthMapTest, ValidFloorIsHealthyWithoutObstacles) {
  start();feed(floorImage(),3);
  ASSERT_FALSE(ready.empty());EXPECT_TRUE(ready.back());
  ASSERT_FALSE(clouds.empty());EXPECT_EQ(clouds.back().width,0u);
  EXPECT_EQ(map->getOccupancy(Eigen::Vector3d(2.01,0,.01)),0);
}
TEST_F(DepthMapTest, ZeroNanAndMissingInputDoNotEraseHistory) {
  retention=.2;start();wall();ASSERT_EQ(map->getOccupancy(point()),1);
  feed(image(0),4);feed(image(NAN),4);
  EXPECT_EQ(map->getOccupancy(point()),1);
  ASSERT_FALSE(ready.empty());EXPECT_FALSE(ready.back());
  // Reacquisition may need confirmation again after the invalid gap; current
  // supported points must protect the old obstacle from expiring in that gap.
  feed(image(1));EXPECT_EQ(map->getOccupancy(point()),1);
  feed(image(1));EXPECT_EQ(map->getOccupancy(point()),1);
}
TEST_F(DepthMapTest, RejectedSparseFarPointDoesNotClearNearObstacle) {
  start();wall();auto m=image(NAN);
  reinterpret_cast<float*>(m.data.data())[50*80+38]=2.f;
  feed(m,4);
  EXPECT_EQ(map->getOccupancy(point()),1);
  ASSERT_FALSE(clouds.empty());EXPECT_EQ(clouds.back().width,0u);
}
TEST_F(DepthMapTest, ValidFarSurfaceClearsObservedFreeSpace) {
  start();wall();ASSERT_EQ(map->getOccupancy(point()),1);
  feed(image(2));EXPECT_EQ(map->getOccupancy(point()),1);
  feed(image(2));EXPECT_EQ(map->getOccupancy(point()),0);
  // Nearby unobserved wall cells may still inflate this location. Check raw
  // observed-free evidence without silently demanding neighbour deletion.
  EXPECT_FALSE(map->isUnknown(point()));
}
TEST_F(DepthMapTest, FiveSecondExpirationReturnsUnknownAndRemovesInflation) {
  start();wall();ASSERT_EQ(map->getOccupancy(point(.55)),1);
  ASSERT_EQ(map->getInflateOccupancy(point(.55),0),1);
  const auto before=std::chrono::steady_clock::now();
  while(std::chrono::duration<double>(std::chrono::steady_clock::now()-before).count()<4.5)
    feed(floorImage());
  EXPECT_EQ(map->getOccupancy(point(.55)),1);
  while(std::chrono::duration<double>(std::chrono::steady_clock::now()-before).count()<5.4)
    feed(floorImage());
  EXPECT_EQ(map->getOccupancy(point(.55)),0);
  EXPECT_TRUE(map->isUnknown(point(.55)));
  EXPECT_EQ(map->getInflateOccupancy(point(.55),0),0);
}
TEST_F(DepthMapTest, MissingHeightTransformRejectsFusion) {
  missing_ground=true;start();feed(image(1),4);
  EXPECT_EQ(map->getOccupancy(point()),0);EXPECT_TRUE(clouds.empty());
  ASSERT_FALSE(ready.empty());EXPECT_FALSE(ready.back());
}
TEST_F(DepthMapTest, HeightTransformUsesExposureTime) {
  dynamic_ground=true;start();tf2_ros::TransformBroadcaster dynamic(node);
  const auto stamp=node->now()-rclcpp::Duration::from_seconds(.1);
  geometry_msgs::msg::TransformStamped a,b;
  a.header.frame_id=world;a.child_frame_id=ground;a.header.stamp=stamp;
  a.transform.rotation.w=1;b=a;
  b.header.stamp=stamp+rclcpp::Duration::from_seconds(.3);b.transform.translation.z=1;
  dynamic.sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{a,b});spin(.04);
  auto m=image(1);m.header.stamp=stamp;feed(m,1,false);
  m.header.stamp=stamp+rclcpp::Duration::from_seconds(.001);feed(m,1,false);
  EXPECT_EQ(map->getOccupancy(point()),1);
}
TEST_F(DepthMapTest, SlidingResetDoesNotReuseOldConfirmationOrInflation) {
  sliding=true;start();wall();ASSERT_EQ(map->getOccupancy(point()),1);
  publishTransforms(8);spin(.05);feed(image(1));
  EXPECT_EQ(map->getOccupancy(Eigen::Vector3d(9.01,.02,.3)),0);
  feed(image(1));EXPECT_EQ(map->getOccupancy(Eigen::Vector3d(9.01,.02,.3)),1);
}
