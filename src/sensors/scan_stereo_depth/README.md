# D435i 软件双目深度节点

独立 ament/C++ 包，输入已校正、同步的左右 `mono8` 图像及各自 CameraInfo，输出左目光学坐标系下的 `16UC1` 毫米深度及匹配的 CameraInfo。无需硬件深度流，不开启相机驱动，不发布 TF 或运动指令。

```bash
source /opt/ros/humble/setup.bash
source /home/marsdog/SCAN-Planner/install/local_setup.bash
ros2 launch scan_stereo_depth stereo_depth.launch.py
```

默认输入：`/camera/camera/infra1/image_rect_raw`、`infra2/image_rect_raw` 及对应 `camera_info`。输出：`/scan/stereo/depth`、`/scan/stereo/camera_info`、`/scan/stereo/statistics`。

原分辨率快速模式为 BM、96 个视差、15 像素窗口、OpenCV 两线程。低负载模式：

```bash
ros2 launch scan_stereo_depth stereo_depth.launch.py downsample:=2 num_disparities:=48
```

`method:=sgbm` 可选择 SGBM；实际相机性能记录显示 640×480 时它低于 30 Hz，不作为快速默认模式。参数配置在 `config/stereo.yaml`。

匹配使用最新图像对工作线程，积压时替换待处理图像对；输入订阅深度为 2，同步队列为 3，默认左右时间差最多 10 ms，丢弃超过 200 ms 的输入。输出保留左图曝光时间戳，不用处理完成时间替代。

基线由左右校正 P 的平移项计算，包含左右主点差校正，OpenCV 固定点视差除以 16 后计算 Z。缩小图像时同步调整焦距、主点及输出 CameraInfo。右 CameraInfo 的 frame 可为右光学 frame，也可为 RealSense 使用的左参考 frame；不根据 TF 重复计算基线。

输出距离默认 0.3–3.0 m，匹配失败、非正视差和超出输出范围的像素为 0，不做孔洞填充。统计里的有效比例是当前视野中该范围内有效深度的覆盖率，不是精度分数。实际环境的弱纹理、光照和反光会改变结果。

性能实测见用户指南 `D:\work\doc\SCAN-Planner单核编译与被动双目深度接入指南.md`。独立性能脚本的 Windows 保存位置为 `D:\work\scripts\scan-planner\benchmark_stereo_depth.py`，板端为工作区 `tools/benchmark_stereo_depth.py`。测试程序会关闭自己启动的相机与计算节点，不启动底盘控制器。
