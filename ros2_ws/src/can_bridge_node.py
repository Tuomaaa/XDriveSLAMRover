"""
can_bridge_node.py
ROS2 bridge between the CAN bus (STM32 firmware) and the ROS graph.

Responsibilities:
  - Send heartbeat (0x300) so the STM32 keeps its motors enabled.
  - Forward velocity commands (topic 'rpm_cmd' -> CAN 0x100).
  - Read encoder frames (0x200/0x201), feed OdometryEstimator, and publish
    nav_msgs/Odometry on 'odom' plus the odom->base_link TF.
  - Republish raw encoder ticks on 'encoder_raw' for debugging.

Run (flat scripts, no colcon package):
    source /opt/ros/jazzy/setup.bash
    python3 can_bridge_node.py

Imports are flat (from protocol import ...) to match every other script in
this directory -- there is no can_bridge/ package on disk.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from protocol import (
    MsgId, EncoderMsg, ErrorMsg, AckMsg,
    HEARTBEAT_PERIOD_MS,
    encode_vel_cmd, encode_heartbeat, decode,
)
from can_interface import CanInterface
from odometry import OdometryEstimator


# RPi -> STM32 IDs. SocketCAN loops frames from other local sockets back to
# us; these are commands we send, not telemetry to decode, so we skip them.
_OUTGOING_IDS = frozenset((
    MsgId.VEL_CMD,    # 0x100
    MsgId.HEARTBEAT,  # 0x300
    MsgId.PID_KP,     # 0x400
    MsgId.PID_KI,     # 0x401
    MsgId.PID_KD,     # 0x402
))


class CanBridgeNode(Node):

    def __init__(self):
        super().__init__('can_bridge')

        # ── CAN bus ──
        channel = self.declare_parameter('can_channel', 'can0').value
        bitrate = self.declare_parameter('can_bitrate', 500_000).value
        self._can = CanInterface(channel=channel, bitrate=bitrate)
        self.get_logger().info(f'CAN bus opened: {channel} @ {bitrate} bps')

        # ── Odometry ──
        self._odo = OdometryEstimator()
        # Latest cumulative ticks per motor index; update() fires once all
        # four (from 0x200 + 0x201) are present.
        self._ticks = {}
        self._odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self._base_frame = self.declare_parameter('base_frame', 'base_link').value
        self._log_pose = self.declare_parameter('log_pose', True).value
        self._update_count = 0

        # ── Publishers ──
        self._encoder_pub = self.create_publisher(
            Int32MultiArray, 'encoder_raw', 10)
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        # ── Subscribers ──
        self._rpm_sub = self.create_subscription(
            Int16MultiArray, 'rpm_cmd', self._rpm_cmd_callback, 10)

        # ── Timers ──
        self._heartbeat_timer = self.create_timer(
            HEARTBEAT_PERIOD_MS / 1000.0, self._heartbeat_callback)
        self._can_recv_timer = self.create_timer(
            0.001, self._can_recv_callback)  # 1kHz polling

    # ── Callbacks ──

    def _heartbeat_callback(self):
        self._can.send(MsgId.HEARTBEAT, encode_heartbeat())

    def _rpm_cmd_callback(self, msg: Int16MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn(f'rpm_cmd expects 4 values, got {len(msg.data)}')
            return
        data = encode_vel_cmd(msg.data[0], msg.data[1], msg.data[2], msg.data[3])
        self._can.send(MsgId.VEL_CMD, data)

    def _can_recv_callback(self):
        msg = self._can.recv(timeout=0)
        if msg is None:
            return

        # SocketCAN echoes frames sent by other local sockets (e.g. the
        # PS2 driver transmitting 0x100/0x300) onto this socket too. Those
        # are RPi->STM32 commands, not something we decode -- skip them
        # quietly so they don't flood the log as "unknown ID".
        if msg.arbitration_id in _OUTGOING_IDS:
            return

        result = decode(msg.arbitration_id, msg.data)

        if isinstance(result, EncoderMsg):
            # Frame 0 (0x200) -> motors 0,1 ; Frame 1 (0x201) -> motors 2,3.
            base = result.motor_id * 2
            self._ticks[base] = result.motor_a_ticks
            self._ticks[base + 1] = result.motor_b_ticks

            # Debug passthrough of raw ticks.
            out = Int32MultiArray()
            out.data = [result.motor_id, result.motor_a_ticks, result.motor_b_ticks]
            self._encoder_pub.publish(out)

            # Fire odometry once we have a full set of 4. Trigger on the
            # frame that completes the set (normally 0x201, sent right after
            # 0x200 by the STM32). Use the CAN frame's own arrival timestamp
            # so bus/scheduling jitter isn't absorbed as a velocity bias.
            if result.motor_id == 1 and len(self._ticks) == 4:
                self._on_encoder_complete(msg.timestamp)

        elif isinstance(result, ErrorMsg):
            self.get_logger().error(
                f'STM32 error 0x{result.error_code:02X}: {result.data.hex()}')

        elif isinstance(result, AckMsg):
            self.get_logger().debug(
                f'ACK for cmd 0x{result.cmd_id:02X}: {result.data.hex()}')

        elif result is None:
            self.get_logger().warn(
                f'Unknown CAN ID: 0x{msg.arbitration_id:03X}')

    def _on_encoder_complete(self, timestamp: float):
        result = self._odo.update(dict(self._ticks), timestamp)
        if result is None:
            return  # first sample or out-of-order frame

        x, y, theta, vx, vy, omega = result
        self._publish_odom(x, y, theta, vx, vy, omega)

        if self._log_pose:
            self._update_count += 1
            if self._update_count % 25 == 0:  # ~2Hz at 50Hz encoder rate
                self.get_logger().info(
                    f'pose  x={x:+.3f} y={y:+.3f} theta={math.degrees(theta):+6.1f}deg  '
                    f'| vx={vx:+.3f} vy={vy:+.3f} omega={omega:+.3f}')

    def _publish_odom(self, x, y, theta, vx, vy, omega):
        now = self.get_clock().now().to_msg()

        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega
        self._odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)

    def destroy_node(self):
        self._can.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
