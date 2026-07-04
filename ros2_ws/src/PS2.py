"""
PS2 Controller driver for Raspberry Pi 4.

Zero-dependency GPIO bit-bang using /dev/mem + mmap.
No RPi.GPIO, no gpiod, no pip — just Python 3 stdlib.
Requires root (sudo) for /dev/mem access.

Wiring (BCM numbering):
    RPi 3.3V  → VCC
    RPi GND   → GND
    GPIO17    → CLK
    GPIO27    → CMD  (RPi → controller)
    GPIO22    → DAT  (controller → RPi, internal pull-up)
    GPIO23    → CS   (ATT, active low)

IMPORTANT: Power the PS2 receiver at 3.3V, NOT 5V.

Timing note: DAT is read during CLK LOW phase (not rising edge).
The PS2 controller updates DAT on the falling edge of CLK,
so we read after the LOW has settled.
"""

import mmap
import struct
import time

# ── BCM2711 (RPi 4) GPIO registers ──

_GPIO_BASE = 0xFE200000
_BLOCK_SIZE = 4096

_GPFSEL0   = 0x00
_GPSET0    = 0x1C
_GPCLR0    = 0x28
_GPLEV0    = 0x34
_PUP_PDN0  = 0xE4


class _RawGPIO:
    """Minimal direct-register GPIO for BCM2711."""

    def __init__(self):
        f = open('/dev/mem', 'r+b', buffering=0)
        self._mem = mmap.mmap(f.fileno(), _BLOCK_SIZE, offset=_GPIO_BASE)
        f.close()

    def _rd(self, off):
        return struct.unpack_from('<I', self._mem, off)[0]

    def _wr(self, off, val):
        struct.pack_into('<I', self._mem, off, val)

    def set_output(self, pin):
        reg = _GPFSEL0 + (pin // 10) * 4
        v = self._rd(reg)
        s = (pin % 10) * 3
        v = (v & ~(7 << s)) | (1 << s)
        self._wr(reg, v)

    def set_input(self, pin):
        reg = _GPFSEL0 + (pin // 10) * 4
        v = self._rd(reg)
        s = (pin % 10) * 3
        v = v & ~(7 << s)
        self._wr(reg, v)

    def set_pull_up(self, pin):
        reg = _PUP_PDN0 + (pin // 16) * 4
        v = self._rd(reg)
        s = (pin % 16) * 2
        v = (v & ~(3 << s)) | (1 << s)
        self._wr(reg, v)

    def high(self, pin):
        off = _GPSET0 + (4 if pin >= 32 else 0)
        self._wr(off, 1 << (pin % 32))

    def low(self, pin):
        off = _GPCLR0 + (4 if pin >= 32 else 0)
        self._wr(off, 1 << (pin % 32))

    def read(self, pin):
        off = _GPLEV0 + (4 if pin >= 32 else 0)
        return (self._rd(off) >> (pin % 32)) & 1

    def close(self):
        self._mem.close()


# ── Pin defaults (BCM) ──

CLK_PIN = 17
CMD_PIN = 27
DAT_PIN = 22
CS_PIN  = 23

# Half-clock delay: 50us → ~10kHz clock (PS2 tolerates slower)
_DELAY = 0.000050

# ── Button masks for btn1 (byte 4) ──
BTN_SELECT = 0x01
BTN_JOYR   = 0x02
BTN_JOYL   = 0x04
BTN_START  = 0x08
BTN_UP     = 0x10
BTN_RIGHT  = 0x20
BTN_DOWN   = 0x40
BTN_LEFT   = 0x80

# ── Button masks for btn2 (byte 5) ──
BTN_L2       = 0x01
BTN_R2       = 0x02
BTN_L1       = 0x04
BTN_R1       = 0x08
BTN_TRIANGLE = 0x10
BTN_CIRCLE   = 0x20
BTN_CROSS    = 0x40
BTN_SQUARE   = 0x80


class PS2Controller:

    def __init__(self, clk=CLK_PIN, cmd=CMD_PIN, dat=DAT_PIN, cs=CS_PIN):
        self.clk = clk
        self.cmd = cmd
        self.dat = dat
        self.cs  = cs

        self._gpio = _RawGPIO()
        self._gpio.set_output(clk)
        self._gpio.set_output(cmd)
        self._gpio.set_input(dat)
        self._gpio.set_pull_up(dat)
        self._gpio.set_output(cs)

        # Idle state
        self._gpio.high(clk)
        self._gpio.high(cmd)
        self._gpio.high(cs)

    def _transfer_byte(self, tx):
        """Exchange one byte, LSB first. Read DAT during CLK LOW phase."""
        gpio = self._gpio
        clk, cmd, dat = self.clk, self.cmd, self.dat
        rx = 0
        for i in range(8):
            # Drive CMD bit, then pull CLK low
            if (tx >> i) & 1:
                gpio.high(cmd)
            else:
                gpio.low(cmd)
            gpio.low(clk)
            time.sleep(_DELAY)

            # Read DAT while CLK is still LOW
            if gpio.read(dat):
                rx |= (1 << i)

            # Pull CLK high
            gpio.high(clk)
            time.sleep(_DELAY)

        return rx

    def read(self):
        """
        Poll controller once.

        Returns dict: mode, btn1, btn2, rx, ry, lx, ly
        Returns None if no controller / bad response.
        """
        gpio = self._gpio
        gpio.low(self.cs)
        time.sleep(0.0001)

        self._transfer_byte(0x01)
        mode = self._transfer_byte(0x42)
        ready = self._transfer_byte(0x00)

        if ready != 0x5A:
            gpio.high(self.cs)
            return None

        btn1 = self._transfer_byte(0x00)
        btn2 = self._transfer_byte(0x00)

        if mode in (0x73, 0x53):
            rx = self._transfer_byte(0x00)
            ry = self._transfer_byte(0x00)
            lx = self._transfer_byte(0x00)
            ly = self._transfer_byte(0x00)
        else:
            rx = ry = lx = ly = 0x80

        gpio.high(self.cs)
        time.sleep(0.001)

        return {
            'mode': mode,
            'btn1': btn1, 'btn2': btn2,
            'rx': rx, 'ry': ry,
            'lx': lx, 'ly': ly,
        }

    def is_pressed(self, btn_byte, mask):
        """Active low: 0 = pressed."""
        return (btn_byte & mask) == 0

    def cleanup(self):
        self._gpio.close()


# ── Test ──

if __name__ == '__main__':
    ps2 = PS2Controller()
    print("PS2 Controller Test — Ctrl+C to exit")
    try:
        while True:
            data = ps2.read()
            if data is None:
                print("No controller detected")
            else:
                print(
                    f"MODE:0x{data['mode']:02X}  "
                    f"BTN1:0x{data['btn1']:02X}  BTN2:0x{data['btn2']:02X}  "
                    f"LX:{data['lx']:3d}  LY:{data['ly']:3d}  "
                    f"RX:{data['rx']:3d}  RY:{data['ry']:3d}"
                )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nExiting")
    finally:
        ps2.cleanup()