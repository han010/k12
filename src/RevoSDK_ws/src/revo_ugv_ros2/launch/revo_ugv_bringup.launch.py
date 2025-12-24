from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Arguments
    host_arg = DeclareLaunchArgument("host", default_value="192.168.234.1")
    port_arg = DeclareLaunchArgument("port", default_value="10151")
    use_ros2_control_arg = DeclareLaunchArgument("use_ros2_control", default_value="true")
    use_laserscan_arg = DeclareLaunchArgument("use_laserscan", default_value="true")
    slam_params_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value="",
        description="Optional slam_toolbox params file; when empty, slam_toolbox is not started.",
    )

    # Robot description
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("revo_ugv_ros2"), "urdf", "revo_ugv.urdf.xacro"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", urdf_path]),
            value_type=str,
        )
    }

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # Revo bridge (existing launch)
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("revo_ugv_ros2"), "launch", "revo_ugv_bridge.launch.py"])
        ),
        launch_arguments={"host": LaunchConfiguration("host"), "port": LaunchConfiguration("port")}.items(),
    )

    # Astra camera
    astra_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("astra_camera"), "launch", "astra.launch.xml"])
        )
    )

    # ros2_control + controllers
    controller_config = PathJoinSubstitution([FindPackageShare("revo_ugv_ros2"), "config", "controllers.yaml"])
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
    )
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
    )
    diff_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
    )

    # Depth image -> laser scan for SLAM
    depth_to_scan = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        remappings=[
            ("/depth", "/camera/depth/image_raw"),
            ("/depth_camera_info", "/camera/depth/camera_info"),
            ("/scan", "/scan"),
        ],
        parameters=[{"output_frame": "camera_link"}],
        condition=IfCondition(LaunchConfiguration("use_laserscan")),
    )

    # Optional slam_toolbox start when params file is provided
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"])
        ),
        launch_arguments={"slam_params_file": LaunchConfiguration("slam_params_file")}.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("slam_params_file"), "' != ''"])
        ),
    )

    return LaunchDescription(
        [
            host_arg,
            port_arg,
            use_ros2_control_arg,
            use_laserscan_arg,
            slam_params_arg,
            rsp_node,
            bridge_launch,
            astra_launch,
            control_node,
            jsb_spawner,
            diff_spawner,
            depth_to_scan,
            slam_launch,
        ]
    )
