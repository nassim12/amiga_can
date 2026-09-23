#!/usr/bin/env python3
"""
amiga_keyboard_teleop.py  —  standalone (no ROS required)
Keyboard teleoperation for the Farm-ng Amiga via Peak PCAN-USB / SocketCAN.

Architecture (three threads):
  main         — reads keyboard, updates command state, draws status line
  recv_thread  — reads TPDO1, updates robot state
  send_thread  — sends RPDO1 at a fixed clock-based rate (default 20 Hz)

The send thread is decoupled from the keyboard loop so frame delivery is
consistent even when Python scheduling delays the main loop.  The dashboard
requires ≥ 5 Hz RPDO1; consistent 20 Hz prevents the AUTO_READY watchdog
drop-out that occurs with timer-coupled sends.

State machine:
  IDLE ──(E)──► AUTO_READY ──(dashboard confirms)──► AUTO_ACTIVE
  AUTO_ACTIVE ──(Space / Q)──► IDLE

Keys:
  E           engage auto mode
  W / ↑       forward         S / ↓   backward
  A / ←       turn left       D / →   turn right
  Space       stop and disengage (→ IDLE)
  + / =       increase max speed    - / _   decrease max speed
  Q / Esc     quit

Note: physical pendant must be in auto mode.
"""

import argparse
import struct
import sys
import termios
import threading
import time
import tty
import select

import can

# ── Amiga CAN constants ───────────────────────────────────────────────────────
AMIGA_NODE_ID      = 0x0E
AMIGA_RPDO1_COB_ID = 0x200 + AMIGA_NODE_ID   # 0x20E  we send
AMIGA_TPDO1_COB_ID = 0x180 + AMIGA_NODE_ID   # 0x18E  dashboard sends

SPEED_SCALE = 1000.0
ANG_SCALE   = 1000.0
_FMT        = '<Bhh3x'   # uint8 + int16 + int16 + 3 pad = 8 bytes


class St:
    """AmigaControlState — matches farm-ng packet.py exactly."""
    BOOT          = 0   # STATE_BOOT
    MANUAL_READY  = 1   # STATE_MANUAL_READY   (pendant idle)
    MANUAL_ACTIVE = 2   # STATE_MANUAL_ACTIVE  (pendant joystick driving)
    CC_ACTIVE     = 3   # STATE_CC_ACTIVE      (cruise control)
    AUTO_READY    = 4   # STATE_AUTO_READY     (ready for CAN autonomous cmd)
    AUTO_ACTIVE   = 5   # STATE_AUTO_ACTIVE    (executing CAN autonomous cmd)
    ESTOPPED      = 6   # STATE_ESTOPPED


_ST_NAME = {
    St.BOOT:          'BOOT',
    St.MANUAL_READY:  'MANUAL_READY',
    St.MANUAL_ACTIVE: 'MANUAL_ACTIVE',
    St.CC_ACTIVE:     'CC_ACTIVE',
    St.AUTO_READY:    'AUTO_READY',
    St.AUTO_ACTIVE:   'AUTO_ACTIVE',
    St.ESTOPPED:      'ESTOPPED',
}

MAX_V_MIN = 0.1
MAX_V_MAX = 0.8
MAX_W     = 1.0

# ── Key definitions ───────────────────────────────────────────────────────────
_KEY_W     = {b'w', b'\x1b[A'}
_KEY_S     = {b's', b'\x1b[B'}
_KEY_A     = {b'a', b'\x1b[D'}
_KEY_D     = {b'd', b'\x1b[C'}
_KEY_STOP  = {b' '}
_KEY_QUIT  = {b'q', b'Q', b'\x1b\x1b', b'\x03'}
_KEY_PLUS  = {b'+', b'='}
_KEY_MINUS = {b'-', b'_'}
_KEY_ENG   = {b'e', b'E'}


def _pack(state: int, v: float, w: float) -> bytes:
    v = max(-MAX_V_MAX, min(MAX_V_MAX, v))
    w = max(-MAX_W,     min(MAX_W,     w))
    return struct.pack(_FMT, state, int(v * SPEED_SCALE), int(w * ANG_SCALE))


def _read_key() -> bytes:
    """Non-blocking key read with 50 ms timeout."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rdy, _, _ = select.select([sys.stdin], [], [], 0.05)
        if not rdy:
            return b''
        ch = sys.stdin.buffer.read(1)
        if ch == b'\x1b':
            rdy2, _, _ = select.select([sys.stdin], [], [], 0.02)
            if rdy2:
                ch += sys.stdin.buffer.read(2)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ── Shared state ──────────────────────────────────────────────────────────────

class _Robot:
    """Written by recv_thread, read by main loop."""
    def __init__(self):
        self.lock      = threading.Lock()
        self.state     = St.MANUAL_READY
        self.speed     = 0.0
        self.ang       = 0.0
        self.last_tpdo = 0.0   # monotonic


class _Cmd:
    """Written by main loop, read by send_thread."""
    def __init__(self):
        self.lock      = threading.Lock()
        self.req_state = None   # None = silent (no frames sent)
        self.v         = 0.0
        self.w         = 0.0


# ── Background threads ────────────────────────────────────────────────────────

def _recv_thread(bus: can.BusABC, robot: _Robot, stop: threading.Event):
    while not stop.is_set():
        try:
            msg = bus.recv(timeout=0.1)
        except can.CanError:
            continue
        if msg is None:
            continue
        if msg.arbitration_id == AMIGA_TPDO1_COB_ID and len(msg.data) >= 5:
            state = msg.data[0]
            speed = struct.unpack_from('<h', msg.data, 1)[0] / 1000.0
            ang   = struct.unpack_from('<h', msg.data, 3)[0] / 1000.0
            with robot.lock:
                robot.state     = state
                robot.speed     = speed
                robot.ang       = ang
                robot.last_tpdo = time.monotonic()


def _send_thread(bus: can.BusABC, cmd: _Cmd, stop: threading.Event, rate_hz: float):
    """
    Clock-based RPDO1 sender.  Uses a monotonic deadline to correct for any
    drift so actual delivery stays within ±1 ms of the nominal interval.
    Decoupled from the keyboard loop — jitter in the main thread does not
    affect CAN frame timing.
    """
    interval    = 1.0 / rate_hz
    next_send   = time.monotonic() + interval

    while not stop.is_set():
        # Drift-correcting sleep: sleep only the remaining fraction of the
        # current interval, clamped to ≥ 0 so we never block on overruns.
        sleep = next_send - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        next_send += interval

        with cmd.lock:
            req  = cmd.req_state
            v    = cmd.v
            w    = cmd.w

        if req is None:
            continue   # silent until E is pressed

        send_v = v if req == St.AUTO_ACTIVE else 0.0
        send_w = w if req == St.AUTO_ACTIVE else 0.0

        try:
            bus.send(can.Message(
                arbitration_id = AMIGA_RPDO1_COB_ID,
                data           = _pack(req, send_v, send_w),
                is_extended_id = False,
            ), timeout=0.01)
        except can.CanError as exc:
            sys.stdout.write(f"\n[CAN send error] {exc}\n")


# ── Main loop ─────────────────────────────────────────────────────────────────

def run(channel: str, interface: str, bitrate: int, rate_hz: float):
    print(f"\nOpening {interface} on {channel} …")
    bus   = can.interface.Bus(interface=interface, channel=channel, bitrate=bitrate)
    robot = _Robot()
    cmd   = _Cmd()
    stop  = threading.Event()

    threading.Thread(target=_recv_thread, args=(bus, robot, stop), daemon=True).start()
    threading.Thread(target=_send_thread, args=(bus, cmd,   stop, rate_hz), daemon=True).start()

    print("\n  ┌──────────────────────────────────────────────────┐")
    print("  │  E: engage    Space: stop/disengage   Q: quit    │")
    print("  │  W/↑ fwd   S/↓ back   A/← left   D/→ right     │")
    print("  │  +/- : adjust max speed                          │")
    print("  └──────────────────────────────────────────────────┘\n")

    max_v   = 0.4
    decay   = 0.80
    engaged = False

    try:
        while True:
            key = _read_key()

            # ── Keyboard → command state ───────────────────────────────────────
            if key in _KEY_QUIT:
                break

            elif key in _KEY_ENG:
                if not engaged:
                    engaged = True
                    with robot.lock:
                        current_robot = robot.state
                    with cmd.lock:
                        cmd.v = cmd.w = 0.0
                        cmd.req_state = (St.AUTO_ACTIVE
                                         if current_robot == St.AUTO_ACTIVE
                                         else St.AUTO_READY)

            elif key in _KEY_STOP:
                engaged = False
                with cmd.lock:
                    cmd.req_state = St.MANUAL_READY
                    cmd.v = cmd.w = 0.0

            elif engaged:
                driven = False
                with cmd.lock:
                    if   key in _KEY_W: cmd.v =  max_v; cmd.w = 0.0;    driven = True
                    elif key in _KEY_S: cmd.v = -max_v; cmd.w = 0.0;    driven = True
                    elif key in _KEY_A: cmd.v = 0.0;    cmd.w =  MAX_W; driven = True
                    elif key in _KEY_D: cmd.v = 0.0;    cmd.w = -MAX_W; driven = True
                    elif key in _KEY_PLUS:
                        max_v = min(MAX_V_MAX, round(max_v + 0.05, 2))
                    elif key in _KEY_MINUS:
                        max_v = max(MAX_V_MIN, round(max_v - 0.05, 2))
                    elif key == b'':   # no key — decay
                        cmd.v *= decay; cmd.w *= decay
                        if abs(cmd.v) < 0.01: cmd.v = 0.0
                        if abs(cmd.w) < 0.01: cmd.w = 0.0

            # ── State machine advancement ──────────────────────────────────────
            with robot.lock:
                rs   = robot.state
                rv   = robot.speed
                ra   = robot.ang
                tage = time.monotonic() - robot.last_tpdo

            with cmd.lock:
                req = cmd.req_state

            if engaged:
                if req == St.AUTO_READY and rs in (St.AUTO_READY, St.AUTO_ACTIVE):
                    # Dashboard confirmed AUTO_READY (or already AUTO_ACTIVE) — advance
                    with cmd.lock:
                        cmd.req_state = St.AUTO_ACTIVE
                elif req == St.AUTO_ACTIVE and rs not in (St.AUTO_READY, St.AUTO_ACTIVE):
                    # Dashboard dropped out (ESTOPPED / MANUAL) — disengage
                    engaged = False
                    with cmd.lock:
                        cmd.req_state = None
                        cmd.v = cmd.w = 0.0

            # ── Status line ────────────────────────────────────────────────────
            with cmd.lock:
                req_disp = cmd.req_state
                sv, sw   = cmd.v, cmd.w

            tpdo_str  = f"age={tage*1000:.0f}ms" if robot.last_tpdo else "no TPDO1"
            req_label = _ST_NAME.get(req_disp, '?') if req_disp is not None else 'SILENT'
            sys.stdout.write(
                f"\r\033[K"
                f"  req={req_label:<11}"
                f"  robot={_ST_NAME.get(rs,'?'):<11}"
                f"  cmd v={sv:+.2f} w={sw:+.3f}"
                f"  meas v={rv:+.2f} w={ra:+.3f}"
                f"  max_v={max_v:.2f}"
                f"  [{tpdo_str}]"
            )
            sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        # Send IDLE once before closing
        try:
            bus.send(can.Message(
                arbitration_id = AMIGA_RPDO1_COB_ID,
                data           = _pack(St.MANUAL_READY, 0.0, 0.0),
                is_extended_id = False,
            ), timeout=0.1)
        except can.CanError:
            pass
        bus.shutdown()
        print("\n\nStopped. CAN bus closed.")


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description="Amiga keyboard teleop over CAN")
    ap.add_argument('--channel',   default='can0',      help='SocketCAN interface (default: can0)')
    ap.add_argument('--interface', default='socketcan', help='python-can backend (default: socketcan)')
    ap.add_argument('--bitrate',   default=250000, type=int)
    ap.add_argument('--rate',      default=20.0,   type=float, help='Send rate Hz (default: 20)')
    args = ap.parse_args()
    run(args.channel, args.interface, args.bitrate, args.rate)
