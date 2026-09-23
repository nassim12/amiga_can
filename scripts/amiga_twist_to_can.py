#!/usr/bin/env python3
"""
amiga_twist_to_can.py  —  ROS Noetic node
Converts /cmd_vel (geometry_msgs/Twist) to Amiga CAN velocity frames
via a Peak PCAN-USB adapter using python-can + SocketCAN.

Pipeline:
  TAD-Policy / NoMaD-RED (Jetson)
    → /cmd_vel (Twist)
      → this node
        → SocketCAN (can0)
          → Peak PCAN-USB
            → Amiga CAN bus
              → motor controllers

Dependencies:
  pip install python-can
  # Peak Linux driver + SocketCAN setup (one-time, run as root):
  #   modprobe peak_usb
  #   ip link set can0 up type can bitrate 250000
  #   ip link set can0 txqueuelen 1000

CAN PROTOCOL — VERIFY BEFORE CONNECTING TO HARDWARE:
  The Amiga dashboard (node 0xE = 14) accepts RPDO1 on COB-ID 0x20E.
  Consult the farm-ng Amiga firmware / canbus.proto for the authoritative
  byte layout. The encoding below matches the known AmigaRpdo1 structure
  packed into 8 bytes for classic CAN:

    Byte  0      uint8   state_req    (AmigaControlState, see below)
    Bytes 1-2    int16   cmd_speed    signed, unit = mm/s  (m/s × 1000)
    Bytes 3-4    int16   cmd_ang_rate signed, unit = mrad/s (rad/s × 1000)
    Bytes 5-7    ---     reserved / padding

Safety:
  A watchdog timer sends IDLE + zero velocity if no /cmd_vel arrives
  within `watchdog_timeout` seconds (default 0.5 s).
  On node shutdown a final IDLE frame is always sent.
"""

import struct
import threading

import can
import rospy
from geometry_msgs.msg import Twist

# ── Amiga CAN constants ───────────────────────────────────────────────────────
# Farm-ng Amiga dashboard is CANopen node 0xE.
# RPDO1 COB-ID = 0x200 + node_id  (CANopen RPDO1 function code)
AMIGA_NODE_ID      = 0x0E
AMIGA_RPDO1_COB_ID = 0x200 + AMIGA_NODE_ID   # = 0x20E


class AmigaState:
    """AmigaControlState — matches farm-ng packet.py (STATE_* enum)."""
    MANUAL_READY  = 1   # STATE_MANUAL_READY   (pendant idle)
    MANUAL_ACTIVE = 2   # STATE_MANUAL_ACTIVE  (pendant joystick)
    CC_ACTIVE     = 3   # STATE_CC_ACTIVE      (cruise control)
    AUTO_READY    = 4   # STATE_AUTO_READY     (ready for CAN autonomous cmd)
    AUTO_ACTIVE   = 5   # STATE_AUTO_ACTIVE    (executing CAN autonomous cmd)
    ESTOPPED      = 6   # STATE_ESTOPPED
    IDLE          = MANUAL_READY   # alias for watchdog send


# ── Velocity safety limits ────────────────────────────────────────────────────
# Must match the limits set in navigate.py / replay_policy_on_bag.py
MAX_V  = 0.5   # m/s    forward / reverse
MAX_W  = 1.0   # rad/s  angular

# ── int16 scale factors (classic CAN: 8 bytes max, no room for float32×2) ────
SPEED_SCALE = 1000.0   # 1 LSB = 1 mm/s  →  0.5 m/s = 500 (fits int16)
ANG_SCALE   = 1000.0   # 1 LSB = 1 mrad/s → 1.0 rad/s = 1000 (fits int16)

# struct: little-endian | uint8 | int16 | int16 | 3-byte pad → 8 bytes
_FRAME_FMT = '<Bhh3x'


def _pack(state: int, v_ms: float, w_rads: float) -> bytes:
    v = max(-MAX_V, min(MAX_V, v_ms))
    w = max(-MAX_W, min(MAX_W, w_rads))
    return struct.pack(_FRAME_FMT,
                       state,
                       int(v * SPEED_SCALE),
                       int(w * ANG_SCALE))


# ── Node ──────────────────────────────────────────────────────────────────────

class AmigaTwistToCanNode:

    def __init__(self):
        rospy.init_node('amiga_twist_to_can', anonymous=False)

        # ROS parameters
        channel    = rospy.get_param('~can_channel',       'can0')
        interface  = rospy.get_param('~can_interface',     'socketcan')
        bitrate    = rospy.get_param('~can_bitrate',       250000)
        topic      = rospy.get_param('~cmd_vel_topic',     '/cmd_vel')
        self._wd_timeout = rospy.get_param('~watchdog_timeout', 0.5)  # seconds

        rospy.loginfo(
            "[amiga_can] opening %s on %s (bitrate=%d)",
            interface, channel, bitrate,
        )

        # Open CAN bus (SocketCAN on Linux with Peak driver)
        # For native Peak API instead of SocketCAN, change interface to 'pcan'
        # and channel to 'PCAN_USBBUS1'.
        self._bus = can.interface.Bus(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
        )

        self._lock          = threading.Lock()
        self._last_cmd_time = None   # None = never received
        self._last_v        = 0.0
        self._last_w        = 0.0

        # Heartbeat at 20 Hz: re-sends the latest command continuously so the
        # Amiga dashboard watchdog never drops AUTO_ACTIVE back to AUTO_READY.
        # (Dashboard requires ≥ 5 Hz RPDO1; sending only on /cmd_vel callbacks
        #  is insufficient when the policy publishes at lower rates.)
        self._heartbeat = rospy.Timer(
            rospy.Duration(1.0 / 20.0),
            self._heartbeat_cb,
        )

        self._sub = rospy.Subscriber(
            topic, Twist, self._cmd_vel_cb, queue_size=1,
        )

        rospy.on_shutdown(self._shutdown)
        rospy.loginfo(
            "[amiga_can] ready — listening on %s, watchdog %.1f s, heartbeat 20 Hz",
            topic, self._wd_timeout,
        )

    # ── /cmd_vel callback ─────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        with self._lock:
            self._last_cmd_time = rospy.Time.now()
            self._last_v        = msg.linear.x
            self._last_w        = msg.angular.z

    # ── Heartbeat (20 Hz) ─────────────────────────────────────────────────────

    def _heartbeat_cb(self, _event):
        with self._lock:
            t = self._last_cmd_time
            v = self._last_v
            w = self._last_w
        if t is None:
            return   # never received a cmd_vel — stay silent
        age = (rospy.Time.now() - t).to_sec()
        if age > self._wd_timeout:
            rospy.logwarn_throttle(2.0, "[amiga_can] watchdog: no cmd_vel — IDLE")
            self._send(AmigaState.IDLE, 0.0, 0.0)
        else:
            self._send(AmigaState.AUTO_ACTIVE, v, w)

    # ── CAN send ──────────────────────────────────────────────────────────────

    def _send(self, state: int, v: float, w: float):
        frame = can.Message(
            arbitration_id=AMIGA_RPDO1_COB_ID,
            data=_pack(state, v, w),
            is_extended_id=False,
        )
        try:
            self._bus.send(frame, timeout=0.02)
        except can.CanError as exc:
            rospy.logerr_throttle(5.0, "[amiga_can] send error: %s", exc)

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def _shutdown(self):
        rospy.loginfo("[amiga_can] shutdown — sending IDLE")
        self._send(AmigaState.IDLE, 0.0, 0.0)
        self._bus.shutdown()


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    try:
        AmigaTwistToCanNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
