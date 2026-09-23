#!/usr/bin/env python3
"""
amiga_can_sniffer.py  —  ROS Noetic node
Sniffs all frames on the Amiga CAN bus and prints them to the terminal.

Use this FIRST to verify:
  1. The Peak PCAN-USB cable is connected and recognised by the OS.
  2. The Amiga is powered on and broadcasting.
  3. The CAN bit-rate is correct (should be 250 kbps).
  4. The Amiga TPDO1 frame layout matches what amiga_twist_to_can.py expects.

Output format (one line per frame):
  HH:MM:SS.mmm  ID=0x18E  DLC=8  03 01 F4 00 64 00 00 00  | Amiga TPDO1 ...

Known Amiga frame IDs (verify against farm-ng firmware):
  0x18E  — TPDO1: dashboard → all  (measured speed / ang-rate / state)
  0x20E  — RPDO1: commander → dashboard (cmd speed / ang-rate / state_req)
  0x70E  — NMT heartbeat from dashboard node (0x0E)
"""

import argparse
import struct
import sys
import time
from datetime import datetime

import can
try:
    import rospy
    _HAS_ROS = True
except ImportError:
    _HAS_ROS = False


def _log(msg: str, error: bool = False):
    if _HAS_ROS:
        (rospy.logerr if error else rospy.loginfo)("%s", msg)
    else:
        print(msg, file=sys.stderr if error else sys.stdout, flush=True)

# ── Amiga frame ID map ────────────────────────────────────────────────────────
AMIGA_NODE_ID      = 0x0E
AMIGA_TPDO1_COB_ID = 0x180 + AMIGA_NODE_ID   # 0x18E  dashboard → everyone
AMIGA_RPDO1_COB_ID = 0x200 + AMIGA_NODE_ID   # 0x20E  commander → dashboard
AMIGA_HB_COB_ID    = 0x700 + AMIGA_NODE_ID   # 0x70E  NMT heartbeat

_AMIGA_STATE_NAMES = {
    0: 'BOOT',
    1: 'MANUAL_READY',
    2: 'MANUAL_ACTIVE',
    3: 'CC_ACTIVE',
    4: 'AUTO_READY',
    5: 'AUTO_ACTIVE',
    6: 'ESTOPPED',
}

# ── Decoding helpers ──────────────────────────────────────────────────────────

def _decode_tpdo1(data: bytes) -> str:
    """Dashboard status broadcast (measured speed, ang-rate, state)."""
    if len(data) < 5:
        return "(too short)"
    state    = data[0]
    speed    = struct.unpack_from('<h', data, 1)[0] / 1000.0   # mm/s → m/s
    ang_rate = struct.unpack_from('<h', data, 3)[0] / 1000.0   # mrad/s → rad/s
    name     = _AMIGA_STATE_NAMES.get(state, f'0x{state:02X}')
    return f"Amiga TPDO1  state={name}  meas_speed={speed:+.3f} m/s  meas_ang={ang_rate:+.4f} rad/s"


def _decode_rpdo1(data: bytes) -> str:
    """Velocity command sent by the controller (our own frames echo back)."""
    if len(data) < 5:
        return "(too short)"
    state    = data[0]
    speed    = struct.unpack_from('<h', data, 1)[0] / 1000.0
    ang_rate = struct.unpack_from('<h', data, 3)[0] / 1000.0
    name     = _AMIGA_STATE_NAMES.get(state, f'0x{state:02X}')
    return f"Amiga RPDO1  state_req={name}  cmd_speed={speed:+.3f} m/s  cmd_ang={ang_rate:+.4f} rad/s"


def _decode_nmt_hb(data: bytes) -> str:
    """CANopen NMT heartbeat — byte 0 is the NMT state."""
    _NMT = {0x00: 'BOOT', 0x04: 'STOPPED', 0x05: 'OPERATIONAL', 0x7F: 'PRE-OP'}
    state = data[0] if data else 0xFF
    return f"NMT heartbeat  state={_NMT.get(state, f'0x{state:02X}')}"


_DECODERS = {
    AMIGA_TPDO1_COB_ID: _decode_tpdo1,
    AMIGA_RPDO1_COB_ID: _decode_rpdo1,
    AMIGA_HB_COB_ID:    _decode_nmt_hb,
}


def _format_frame(msg: can.Message) -> str:
    """Format one CAN frame as a readable terminal line."""
    ts  = datetime.fromtimestamp(msg.timestamp).strftime('%H:%M:%S.') + \
          f"{int(msg.timestamp * 1000) % 1000:03d}"
    hex_bytes = ' '.join(f'{b:02X}' for b in msg.data)
    line = f"{ts}  ID=0x{msg.arbitration_id:03X}  DLC={msg.dlc}  {hex_bytes:<27}"

    decoder = _DECODERS.get(msg.arbitration_id)
    if decoder:
        try:
            line += f"  | {decoder(msg.data)}"
        except Exception as exc:
            line += f"  | decode error: {exc}"

    return line


# ── Node ──────────────────────────────────────────────────────────────────────

class AmigaCanSnifferNode:

    def __init__(self, channel='can0', interface='socketcan',
                 bitrate=250000, filter_id=-1, rate_hz=50.0):

        if _HAS_ROS:
            rospy.init_node('amiga_can_sniffer', anonymous=False)
            channel   = rospy.get_param('~can_channel',   channel)
            interface = rospy.get_param('~can_interface', interface)
            bitrate   = rospy.get_param('~can_bitrate',   bitrate)
            filter_id = rospy.get_param('~filter_id',     filter_id)
            rate_hz   = rospy.get_param('~max_print_hz',  rate_hz)

        _log(f"[sniffer] opening {interface} on {channel} (bitrate={bitrate})")

        self._bus = can.interface.Bus(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
        )

        self._filter_id    = int(filter_id)
        self._min_interval = 1.0 / rate_hz
        self._last_print   = 0.0

        print("\n" + "─" * 80)
        print("  Amiga CAN bus sniffer — listening on", channel)
        if self._filter_id >= 0:
            print(f"  Filter: ID=0x{self._filter_id:03X}")
        else:
            print("  Filter: all frames")
        print("─" * 80)
        print(f"  {'Timestamp':<15}  {'ID':<7}  {'DLC':<4}  {'Data bytes':<27}  Interpretation")
        print("─" * 80 + "\n")

        self._run()

    def _run(self):
        shutdown = rospy.is_shutdown if _HAS_ROS else lambda: False
        while not shutdown():
            try:
                msg = self._bus.recv(timeout=0.1)
            except can.CanError as exc:
                _log(f"[sniffer] CAN receive error: {exc}", error=True)
                time.sleep(0.5)
                continue

            if msg is None:
                continue  # timeout — loop back and check shutdown flag

            if self._filter_id >= 0 and msg.arbitration_id != self._filter_id:
                continue

            now = time.monotonic()

            if now - self._last_print >= self._min_interval:
                print(_format_frame(msg), flush=True)
                self._last_print = now

        print("\n[sniffer] shutdown")
        self._bus.shutdown()


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    ap = argparse.ArgumentParser(
        description="Amiga CAN bus sniffer (ROS node or standalone)"
    )
    ap.add_argument('--channel',   default='can0',       help='SocketCAN interface name (default: can0)')
    ap.add_argument('--interface', default='socketcan',  help='python-can backend (default: socketcan)')
    ap.add_argument('--bitrate',   default=250000, type=int, help='CAN bitrate in bps (default: 250000)')
    ap.add_argument('--filter',    default=-1,    type=lambda x: int(x, 0),
                    help='Only show this CAN ID (hex ok, e.g. 0x18E). Default: show all.')
    ap.add_argument('--hz',        default=50.0,  type=float, help='Max print rate Hz (default: 50)')
    args = ap.parse_args()

    try:
        AmigaCanSnifferNode(
            channel=args.channel,
            interface=args.interface,
            bitrate=args.bitrate,
            filter_id=args.filter,
            rate_hz=args.hz,
        )
    except KeyboardInterrupt:
        print("\n[sniffer] interrupted")
