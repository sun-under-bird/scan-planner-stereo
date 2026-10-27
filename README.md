<div align="center">
  <h1>SCAN-Planner Stereo · ROS 2</h1>
  <h2>面向路线引导四足长程导航的空间碰撞感知局部规划器</h2>
  <a href="https://arxiv.org/abs/2606.19555"><img alt="论文" src="https://img.shields.io/badge/论文-arXiv-b31b1b?logo=arxiv&logoColor=white"/></a>
  <a href="https://www.bilibili.com/video/BV15a7P6UEXb/"><img alt="视频" src="https://img.shields.io/badge/视频-Bilibili-FB7299?logo=bilibili&logoColor=white"/></a>
  <a href="https://wuyi2121.github.io/SCAN-Planner/"><img alt="项目主页" src="https://img.shields.io/badge/项目主页-Website-4A90E2?logo=googlechrome&logoColor=white"/></a>
</div>

<p align="center">
  <img src="assets/images/abstract_real.jpg" width="100%"/>
</p>

SCAN-Planner 是一款面向四足机器人导航的空间碰撞感知局部规划器。本分支是原生 ROS 2 自移植版本，适配 Ubuntu 22.04、ROS 2 Humble、C++17 和 `colcon` 构建系统。

本仓库是 [wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 的衍生 ROS 2 移植版。核心算法、项目设计与原始研究工作归功于 Han Zheng、Zhe Chen、Yiwen Fu、Ming Yang 和 Tong Qin；ROS 2 适配由本仓库维护者完成，不代表原作者的官方发布或认可。



## 当前独立版本

本仓库保存 2026-10-27 RK3588 板端当前 SCAN 实现：被动双目 BM 深度、曝光时间 TF、/odom 接入、短轨迹与 A* 修复、点云过滤和历史障碍处理。业务源码、启动文件与运行参数保持板端快照；仅独立使用文档、Git 忽略规则和测试工作区路径作整理。

- [独立部署与参数说明](docs/独立部署与参数说明.md)：构建、相机/TF 前置条件、启动、目标点和关键参数。
- [验证记录](docs/验证记录_2026-10-27.md)：本次提交核对、已停止的构建尝试及历史验证，实机未验收项分别记录。
- 场地 map.pcd、build/install/log 和设备运行环境不随源码提交。
- 当前仍采用一次有效自由射线清障和 5 秒历史过期；转向门槛为 0.8 rad。这些运行策略没有在本次提交中修改。

## D435i 被动双目与 /odom 接入（2026-10-24）

新增独立 `scan_stereo_depth` 包，使用 D435i 左右校正红外图计算深度；默认 BM 快速模式。深度模式可从匹配的 CameraInfo 读取内参，按图像时间戳直接查询现有 TF，并将标准机身速度转换到规划参考系。无效深度不再作为自由空间证据；图像投影缓冲区按实际采样数分配。

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
# 相机、/odom 和完整 TF 由已有系统提供；默认只启动深度和规划。
ros2 launch scan_planner passive_stereo.launch.py
# 需要复用已有速度桥时启用仓库闭环控制器：
ros2 launch scan_planner passive_stereo.launch.py start_controller:=true
```

这两条为二选一的启动方式，不同时启动。新入口不启动机器人模型、相机驱动或底盘桥，不重复发布相机内部 TF。机身输入默认为 `/odom`，规划参考系默认为 `odom`，启用控制器后输出 `geometry_msgs/Twist` 到 `/cmd_vel`。控制器会在深度/TF 或里程计失效时冻结轨迹并发布零速度。

独立节点用法见 [软件双目说明](src/sensors/scan_stereo_depth/README.md)。当前板端部署、性能数据和验证边界见 [独立部署与参数说明](docs/独立部署与参数说明.md)。旧 `run.launch.py` 保留原激光/仿真启动约定；D435i 被动双目使用新入口。


## 被动双目障碍点处理（2026-10-26）

passive_stereo.yaml 默认启用 C++ 点云过滤，位于 plan_env 的深度投影和占用融合之间。流程为：

深度图 + CameraInfo → 曝光时刻相机/地面 TF → 三维投影 → 地面参考高度裁剪 → 5 cm XY 柱点数 → 三维 26 邻域连通簇 → 两帧确认 → 三维占用与膨胀。

- 高度在 base_footprint 下计算，保留 0.08～0.65 m 障碍点；-0.10～0.08 m 的有效地面点仅提供自由射线。必须存在图像曝光时刻的地面参考 TF；不使用 odom 的 z=0 猜地面，也不修改安装外参。
- 5 cm XY 柱至少 8 个采样点；每个三维连通簇至少 4 个体素。保留完整 XYZ 和同柱多个高度，未把三维地图压成二维。
- 障碍需要两个不同源时间戳的连续有效帧确认。重复帧不计数；待确认障碍保护其位置，不用未确认点或被滤除噪声清障。
- 有效地面或已确认远表面提供自由射线；至少一条有效射线经过且无当前障碍支持时清障（兼容原有射线去重）。障碍射线在表面前 0.10 m 停止，当前命中优先。
- 5 秒未再观测的历史障碍在新有效深度到来时退回未知并撤销膨胀。全零、NaN、缺失 TF 或断流不驱动过期清障，深度健康状态按原超时失效。
- 没有增加整帧有效深度比例门槛，没有调整 BM、相机、机器人尺寸、速度控制或 Nav2。固定高度过滤不能替代坡地地面分割。

参数统一使用 grid_map.depth_filter 前缀，定义及默认值见 src/planner/plan_manage/config/passive_stereo.yaml。通用激光/仿真配置默认不启用该过滤。

可观测接口（相对规划器命名空间）：

| 话题 | 类型 | 含义 |
| --- | --- | --- |
| grid_map/filtered_obstacles | sensor_msgs/msg/PointCloud2 | 本帧确认后的三维障碍点，规划 frame，源曝光时间戳；有人订阅才序列化 |
| grid_map/depth_filter_statistics | std_msgs/msg/String | 每秒最多一条 JSON，含有效点、地面、各过滤阶段数量、待确认/障碍点、历史体素及过滤耗时 |
| grid_map/depth_ready | std_msgs/msg/Bool | 原深度健康接口；有效地面帧没有障碍也可以健康 |

隔离的软件验证不连接真实相机或底盘：
~~~bash
# 已 source ROS 和本工作区环境
cd /home/marsdog/SCAN-Planner
colcon test --packages-select plan_env --event-handlers console_direct+
python3 src/planner/plan_manage/test/verify_passive_stereo.py --point-filter --goal-distance 1.0
python3 src/planner/plan_manage/test/benchmark_depth_filter.py --seconds 10
~~~
地图单元测试强制 domain 229、回环网络；链路验证 domain 228，性能测试 domain 230。性能输入为合成深度，不是实机相机或行走验收。

## 构建

安装 ROS 2 Humble 及包依赖后，在工作空间根目录下执行构建：

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install libarmadillo-dev libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

默认构建 CPU 端本地感知后端，如需构建 OpenGL 后端可执行：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_GPU=ON
```
仓库不再链接自带的 x86_64 架构 GLFW 动态库，GPU 构建依赖系统安装的 GLFW、GLEW 和 OpenGL 相关包。

## 快速启动

启动自定义确定性仿真器与规划器：

```bash
source install/setup.bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=1 sensor_type:=lidar \
  controller_mode:=closed_loop use_gpu:=false
```


```bash
source install/setup.bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=1 sensor_type:=lidar \
  controller_mode:=closed_loop use_gpu:=false \
  use_pcd_map:=true pcd_map_file:=/absolute/path/to/your-map.pcd
```



在另一个终端启动 RViz2：

```bash
source install/setup.bash
ros2 launch scan_planner rviz.launch.py
```

RViz2 配置已适配 ROS 2 Humble：Go2 的 RobotModel 使用现有的 `meshes/base.dae`，Sliding Map Bounds 订阅 `/grid_map/sliding_map_bbox`，Goal 订阅 `/goal_point`。



导航模式说明：
- `navi_mode:=1`：使用 RViz2 的 2D 目标点工具选择导航目标
- `navi_mode:=2`：按照 ROS 2 参数文件中预设的 `fsm.waypoints` 路径点序列导航
- `navi_mode:=3`：订阅 `initial_path` 话题获取全局路径，并在局部范围内进行避障

控制器模式分为 `open_loop`（开环）和 `closed_loop`（闭环）两种。本次移植保留的核心启动参数包括：`is_real_world`、`navi_mode`、`sensor_type`、`controller_mode`、`use_gpu`、`use_pcd_map` 和 `pcd_map_file`。

当 `use_pcd_map:=true` 时，必须提供已有的 PCD 点云地图文件：

```bash
ros2 launch scan_planner run.launch.py \
  use_pcd_map:=true pcd_map_file:=/absolute/path/to/map.pcd
```

实际硬件部署时，激光惯导里程计（LIO）、相机和宇树（Unitree）驱动均为外部依赖，默认启动会将规划器输入映射到 `/LIO/odom_vehicle`、`/LIO/odom_imu`、`/LIO/clouds_lidar` 话题以及 RealSense 对齐深度图话题，可根据实际安装的驱动栈修改话题重映射配置。

## 配置与接口

规划器、控制器和仿真器的参数分别位于：
- `src/planner/plan_manage/config/planner.yaml`
- `src/planner/plan_manage/config/controllers.yaml`
- `src/planner/plan_manage/config/simulator.yaml`

ROS 2 参数名称使用点号分隔，例如 `grid_map.resolution`。预设路径点是由 xyz 三元组组成的浮点数组：

```yaml
scan_planner_node:
  ros__parameters:
    fsm.navi_mode: 2
    fsm.waypoints: [0.0, 0.0, 0.3, 5.0, 1.0, 0.3]
```

自定义消息类型为 `scan_planner_msgs/msg/Bspline` 和 `scan_planner_msgs/msg/DataDisp`。规划器相关话题均为相对话题，支持重映射，核心输出话题包括 `planning/bspline`、`planning/data_display` 和 `planning/go2_execution_frozen`。

关键点记录器现在是原生的 `rclpy` 可执行程序：

```bash
ros2 run scan_planner keypoint_recorder.py --output keypoints.yaml
```

## Gazebo Fortress / Go2 仿真

Go2 四足机器人物理模型基于 Gazebo Fortress、`ros_gz_sim` 和 `gz_ros2_control` 构建，对外提供 12 关节的 `joint_trajectory_controller`、`/joint_states` 话题、IMU 数据、四个足端接触力话题以及 `/clock` 时钟话题：

```bash
ros2 launch go2_description go2_sim.launch.py
```

如需在不启动物理仿真时查看模型，可运行：

```bash
ros2 launch go2_description go2_rviz.launch.py
```
旧版 Gazebo Classic 的轨迹/力可视化插件、外力插件以及宇树 ROS 1 专用插件已不在本 ROS 2 仿真版本中提供。


## 致谢

首先感谢原项目 [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 的作者 Han Zheng、Zhe Chen、Yiwen Fu、Ming Yang 和 Tong Qin 开源其研究成果与实现。本仓库在保留原项目 Apache-2.0 许可证的前提下完成 ROS 2 移植，完整署名见 [NOTICE](NOTICE)。

SCAN-Planner 的实现借鉴了 EGO-Planner、ROG-Map、MARSIM、Mockamap 和 Leg-KILO 的算法思路与开源代码，真实机器人定位方案基于 Elevator-LIO / FAST-LIO2 实现。

## 许可证

本仓库遵循 [Apache License 2.0](LICENSE)。分发或派生本项目时，请保留 [NOTICE](NOTICE) 中的原项目署名与许可证信息。
