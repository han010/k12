import os
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 将仓库自带的 Python SDK (../python) 加入 PYTHONPATH，避免未安装 pip 包时报错
    # 尝试从当前文件向上找到工作区目录 RevoSDK_ws，然后指向其上级的 python 目录
    resolved = Path(__file__).resolve()
    sdk_dir = None
    for p in resolved.parents:
        if p.name == "RevoSDK_ws":
            sdk_dir = p.parent / "python"
            break
    # 找不到则退回已存在的 PYTHONPATH
    pythonpath = f"{sdk_dir}:{os.environ.get('PYTHONPATH', '')}" if sdk_dir else os.environ.get("PYTHONPATH", "")

    # 确保 ROS 运行时库路径在环境变量中，防止找不到 librcl_action.so 等问题
    ros_libs = ["/opt/ros/humble/lib", "/opt/ros/humble/lib/x86_64-linux-gnu"]
    existing_ld = os.environ.get("LD_LIBRARY_PATH", "")
    ld_paths = ":".join([p for p in ros_libs if p]) + (f":{existing_ld}" if existing_ld else "")

    # 设置日志目录，避免 rcutils_expand_user 失败
    ros_log_dir = "/tmp/ros_logs"

    return LaunchDescription(
        [
            Node(
                package="revo_ugv_ros2",
                executable="revo_ugv_bridge",
                name="revo_ugv_bridge",
                output="screen",
                env={
                    "PYTHONPATH": pythonpath,
                    "LD_LIBRARY_PATH": ld_paths,
                    "ROS_LOG_DIR": ros_log_dir,
                    "HOME": os.environ.get("HOME", "/home/ros"),
                },
                parameters=[
                    {"host": "192.168.234.1"},
                    {"port": 10151},
                    {"client_name": "ros2_bridge"},
                    {"odom_frame": "odom"},
                    {"base_frame": "base_link"},
                    {"max_linear_ms": 2.0},
                    {"max_angular_rads": 2.0},
                ],
            )
        ]
    )
