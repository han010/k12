import math
import sys
import threading
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from xa_revosdk_ugv.sdk import RevoSDK
from xa_revosdk_ugv.protocol import PoseData, SystemStatus, BatteryData


def yaw_to_quaternion(yaw: float):
    """Convert yaw (rad) to quaternion tuple."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class RevoBridgeNode(Node):
    """
    ROS 2 bridge node: subscribes cmd_vel, sends UDP control, publishes odom/battery/status.
    """

    def __init__(self):
        super().__init__("revo_ugv_bridge")

        # Parameters for network and frame IDs
        self.declare_parameter("host", "192.168.234.1")
        self.declare_parameter("port", 10151)
        self.declare_parameter("client_name", "ros2_bridge")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("max_linear_ms", 2.0)
        self.declare_parameter("max_angular_rads", 2.0)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("command_timeout_sec", 1.0)

        self.host = self.get_parameter("host").get_parameter_value().string_value
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        self.client_name = self.get_parameter("client_name").get_parameter_value().string_value
        self.odom_frame = self.get_parameter("odom_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.max_linear_ms = float(self.get_parameter("max_linear_ms").get_parameter_value().double_value)
        self.max_angular_rads = float(self.get_parameter("max_angular_rads").get_parameter_value().double_value)
        self.command_rate_hz = float(self.get_parameter("command_rate_hz").get_parameter_value().double_value)
        self.command_timeout_sec = float(self.get_parameter("command_timeout_sec").get_parameter_value().double_value)

        self.sdk = RevoSDK()
        self._connected = False
        self._connection_lock = threading.Lock()
        self._cmd_lock = threading.Lock()
        self._desired_linear_raw = 0
        self._desired_angular_raw = 0
        self._last_cmd_time = None

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "odom_raw", 10)
        self.battery_pub = self.create_publisher(BatteryState, "battery_state", 10)
        self.status_pub = self.create_publisher(String, "revo_status", 10)

        # Subscribers
        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 10)

        # Connect to chassis
        self._connect_and_register()

        # Set callbacks for push messages
        self.sdk.subscribe_pose(True, self._on_pose)
        self.sdk.subscribe_system_status(True, self._on_system_status)
        self.sdk.subscribe_battery(True, self._on_battery)
        self.sdk.set_disconnect_callback(self._on_disconnect)

        # Continuous command timer to keep sending latest velocity
        if self.command_rate_hz > 0:
            period = 1.0 / self.command_rate_hz
            self.create_timer(period, self._command_timer_cb)

    def _connect_and_register(self):
        """Establish UDP session and acquire control."""
        with self._connection_lock:
            if self._connected:
                return

            if not self.sdk.connect(host=self.host, port=self.port):
                self.get_logger().error("Failed to connect to Revo chassis")
                return

            if not self.sdk.register(self.client_name):
                self.get_logger().error("Failed to register session")
                return

            if not self.sdk.acquire_control():
                self.get_logger().error("Failed to acquire control authority")
                return

            if not self.sdk.unlock(True):
                self.get_logger().warn("Unlock command failed; chassis may remain locked")

            self._connected = True
            self.status_pub.publish(String(data="connected"))
            self.get_logger().info("Revo UGV connected and control acquired")

    def _cmd_vel_callback(self, msg: Twist):
        """Handle incoming velocity commands (store, timer will send)."""
        # Clamp to configured limits and convert to protocol units
        linear = max(min(msg.linear.x, self.max_linear_ms), -self.max_linear_ms)
        angular = max(min(msg.angular.z, self.max_angular_rads), -self.max_angular_rads)

        linear_raw = int(linear * 100.0)  # 1/100 m/s
        angular_raw = int(angular * 1000.0)  # 1/1000 rad/s

        with self._cmd_lock:
            self._desired_linear_raw = linear_raw
            self._desired_angular_raw = angular_raw
            self._last_cmd_time = self.get_clock().now()

    def _command_timer_cb(self):
        """Periodically send the latest commanded velocity."""
        with self._connection_lock:
            connected = self._connected
        if not connected:
            return

        with self._cmd_lock:
            linear_raw = self._desired_linear_raw
            angular_raw = self._desired_angular_raw
            last_time = self._last_cmd_time

        # Apply timeout to stop the robot if no new command is received
        if last_time is None:
            return
        age = (self.get_clock().now() - last_time).nanoseconds / 1e9
        if self.command_timeout_sec > 0 and age > self.command_timeout_sec:
            linear_raw = 0
            angular_raw = 0

        ok = self.sdk.send_control_command(linear_raw, angular_raw)
        if not ok:
            self.get_logger().warn("Failed to send control command")

    def _on_pose(self, pose: PoseData):
        """Convert pose push to Odometry."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        # Without a map conversion, leave position at origin; yaw is provided.
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = pose.get_altitude_meters()

        qx, qy, qz, qw = yaw_to_quaternion(pose.get_yaw_rad())
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = pose.get_fused_linear_velocity_ms()
        msg.twist.twist.angular.z = pose.get_fused_angular_velocity_rads()

        self.odom_pub.publish(msg)

    def _on_system_status(self, status: SystemStatus):
        """Publish system status as a human-readable string."""
        text = (
            f"control_mode={status.control_mode} "
            f"positioning_status={status.positioning_status} "
            f"battery_status={status.battery_status} "
            f"chassis_status={status.chassis_status} "
            f"motor_status={status.motor_status}"
        )
        self.status_pub.publish(String(data=text))

    def _on_battery(self, battery: BatteryData):
        """Publish battery state."""
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = battery.remaining_capacity / 100.0
        msg.present = battery.battery_count > 0
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.battery_pub.publish(msg)

    def _on_disconnect(self, reason: str):
        """Handle disconnect event from SDK."""
        with self._connection_lock:
            self._connected = False
        self.status_pub.publish(String(data=f"disconnected: {reason}"))
        self.get_logger().warn(f"Disconnected from Revo chassis: {reason}")

    def destroy_node(self):
        """Ensure we release control and unregister cleanly."""
        try:
            if self.sdk.can_control():
                self.sdk.release_control()
            self.sdk.unregister()
            self.sdk.disconnect()
        finally:
            super().destroy_node()


def main(args: Optional[list] = None):
    rclpy.init(args=args)
    node = RevoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
