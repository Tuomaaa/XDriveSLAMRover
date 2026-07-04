
import can
from typing import Optional


class CanInterface:

    def __init__(self, channel: str = 'can0', bitrate: int = 500_000):
        """
        Open a SocketCAN bus.

        Args:
            channel: SocketCAN interface name (e.g. 'can0', 'vcan0' for testing)
            bitrate: CAN bus bitrate in bps (default 500 kbps)
        """
        self._bus = can.Bus(
            channel=channel,
            interface='socketcan',
            bitrate=bitrate,
        )

    def send(self, arbitration_id: int, data: bytes) -> None:
        """
        Send a CAN frame.

        Args:
            arbitration_id: 11-bit standard CAN ID
            data: payload bytes (0-8 bytes)
        """
        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False,
        )
        self._bus.send(msg)

    def recv(self, timeout: Optional[float] = None) -> Optional[can.Message]:
        """
        Receive a CAN frame.

        Args:
            timeout: seconds to wait. None = block forever, 0 = non-blocking.

        Returns:
            can.Message if received, None on timeout.
        """
        return self._bus.recv(timeout=timeout)

    def shutdown(self) -> None:
        """Close the CAN bus."""
        self._bus.shutdown()