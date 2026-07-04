

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int32MultiArray

from can_bridge.protocol import (
    MsgId, EncoderMsg, ErrorMsg, AckMsg,
    HEARTBEAT_PERIOD_MS,
    encode_vel_cmd, encode_heartbeat, decode,
)
from can_bridge.can_interface import CanInterface


class CanBridgeNode(Node):

    def __init__(self):
        super().__init__('can_bridge')

        # ── CAN bus ──
        channel = self.declare_parameter('can_channel', 'can0').value
        bitrate = self.declare_parameter('can_bitrate', 500_000).value
        self._can = CanInterface(channel=channel, bitrate=bitrate)
        self.get_logger().info(f'CAN bus opened: {channel} @ {bitrate} bps')

        # ── Publishers ──
        self._encoder_pub = self.create_publisher(
            Int32MultiArray, 'encoder_raw', 10)

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

        result = decode(msg.arbitration_id, msg.data)

        if isinstance(result, EncoderMsg):
            out = Int32MultiArray()
            out.data = [result.motor_id, result.motor_a_ticks, result.motor_b_ticks]
            self._encoder_pub.publish(out)

        elif isinstance(result, ErrorMsg):
            self.get_logger().error(
                f'STM32 error 0x{result.error_code:02X}: {result.data.hex()}')

        elif isinstance(result, AckMsg):
            self.get_logger().debug(
                f'ACK for cmd 0x{result.cmd_id:02X}: {result.data.hex()}')

        elif result is None:
            self.get_logger().warn(
                f'Unknown CAN ID: 0x{msg.arbitration_id:03X}')

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