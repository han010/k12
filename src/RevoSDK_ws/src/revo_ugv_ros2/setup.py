from setuptools import setup

package_name = "revo_ugv_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/revo_ugv_bridge.launch.py"]),
    ],
    install_requires=["setuptools", "rclpy", "geometry_msgs", "nav_msgs", "sensor_msgs", "std_msgs"],
    zip_safe=True,
    maintainer="Revo Team",
    maintainer_email="support@xarevo.com",
    description="ROS 2 bridge node for Revo UGV SDK.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "revo_ugv_bridge = revo_ugv_ros2.bridge_node:main",
        ],
    },
)
