# revo_ugv_ros2

基于现有 `xa_revosdk_ugv` Python SDK 的 ROS 2 桥接包。

## 功能
- 订阅 `cmd_vel`，通过 UDP 发送线速度/角速度控制到底盘。
- 订阅底盘推送：姿态 -> `odom_raw`，电池 -> `battery_state`，系统状态 -> `revo_status`。
- 心跳由 SDK 维持，断链会在 `revo_status` 发布原因并写日志。
- 提供 URDF（含 ros2_control 占位）和组合 launch，方便与 Astra 摄像头一起跑 SLAM/导航。

## 快速使用
1. 确认已安装底盘 SDK（本仓库 `python/` 下的包）：`pip install -e python`。
2. 构建并运行：
   ```bash
   cd RevoSDK_ws
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch revo_ugv_ros2 revo_ugv_bridge.launch.py host:=192.168.234.1 port:=10151
   ```
3. 发布速度命令：
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"
   ```

4. 一键带起底盘桥 + URDF 发布 + Astra 摄像头 + depthimage_to_laserscan（可选 SLAM）：
   ```bash
   ros2 launch revo_ugv_ros2 revo_ugv_bringup.launch.py \
     host:=192.168.234.1 port:=10151 use_ros2_control:=true use_laserscan:=true \
     slam_params_file:=/path/to/slam_toolbox_params.yaml   # 留空则不启动 SLAM
   ```
   - URDF：`share/revo_ugv_ros2/urdf/revo_ugv.urdf.xacro`，长 2m、宽 1m、轮宽 0.5m，摄像头装在前部上方 0.5m。
   - ros2_control 使用 GenericSystem 占位，`diff_drive_controller` 已配置（命令/里程计仍由 SDK 桥节点处理，后续可替换为真实硬件接口）。
   - `depthimage_to_laserscan` 默认打开，话题 `/scan` 可直接给 SLAM/Nav2 用。

## 话题
- 输入：`cmd_vel` (`geometry_msgs/Twist`)
- 输出：`odom_raw` (`nav_msgs/Odometry`)，`battery_state` (`sensor_msgs/BatteryState`)，`revo_status` (`std_msgs/String`)

## 注意
- 底盘推送的经纬度未在 `Odometry` 中转换，当前仅传递高度和航向；如需定位，请结合 GNSS/IMU 处理自行替换填充。
- 协议中速度单位：线速度 1/100 m/s，角速度 1/1000 rad/s，代码已做缩放与限幅。
