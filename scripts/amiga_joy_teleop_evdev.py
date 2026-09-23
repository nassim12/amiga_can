#!/usr/bin/env python3
"""
amiga_joy_teleop_evdev.py — standalone joystick teleop for Farm-ng Amiga
Reads the gamepad via python-evdev (/dev/input/eventX) instead of the legacy
/dev/input/jsX interface, which the Jetson kernel does not expose for the F710.

Logitech F710 (XInput mode) default mapping:
  ABS_Y  (left stick Y)  -> linear velocity
  ABS_RX (right stick X) -> angular velocity
  BTN_TL (L1)            -> dead-man (hold to move)
  BTN_START             -> engage auto
  BTN_SELECT/BACK       -> disengage
Use --list to print all event codes from your pad so you can remap.
"""

import argparse
import struct
import sys
import threading
import time

import can
from evdev import InputDevice, ecodes, list_devices

# ── Amiga CAN constants ───────────────────────────────────────────────────────
AMIGA_NODE_ID      = 0x0E
AMIGA_RPDO1_COB_ID = 0x200 + AMIGA_NODE_ID   # 0x20E
AMIGA_TPDO1_COB_ID = 0x180 + AMIGA_NODE_ID   # 0x18E

SPEED_SCALE = 1000.0
ANG_SCALE   = 1000.0
_FMT        = '<Bhh3x'

MAX_W = 0.2


class St:
    BOOT = 0; MANUAL_READY = 1; MANUAL_ACTIVE = 2
    CC_ACTIVE = 3; AUTO_READY = 4; AUTO_ACTIVE = 5; ESTOPPED = 6


_ST_NAME = {0:'BOOT',1:'MANUAL_READY',2:'MANUAL_ACTIVE',
            3:'CC_ACTIVE',4:'AUTO_READY',5:'AUTO_ACTIVE',6:'ESTOPPED'}


def _pack(state, v, w, max_v):
    v = max(-max_v, min(max_v, v))
    w = max(-MAX_W, min(MAX_W, w))
    return struct.pack(_FMT, state, int(v*SPEED_SCALE), int(w*ANG_SCALE))


class _Robot:
    def __init__(self):
        self.lock = threading.Lock()
        self.state = St.MANUAL_READY
        self.speed = 0.0; self.ang = 0.0; self.last_tpdo = 0.0


class _Joy:
    def __init__(self):
        self.lock = threading.Lock()
        self.axis_lin = 0.0    # normalized -1..1
        self.axis_ang = 0.0
        self.deadman  = False
        self.engage   = False
        self.disengage = False
        self.last = 0.0


class _Cmd:
    def __init__(self):
        self.lock = threading.Lock()
        self.req_state = None; self.v = 0.0; self.w = 0.0


def _find_gamepad(name_hint="F710"):
    for path in list_devices():
        try:
            d = InputDevice(path)
            if name_hint.lower() in d.name.lower():
                return d
        except Exception:
            continue
    return None


def _recv_thread(bus, robot, stop):
    while not stop.is_set():
        try:
            msg = bus.recv(timeout=0.1)
        except can.CanError:
            continue
        if msg is None:
            continue
        if msg.arbitration_id == AMIGA_TPDO1_COB_ID and len(msg.data) >= 5:
            with robot.lock:
                robot.state = msg.data[0]
                robot.speed = struct.unpack_from('<h', msg.data, 1)[0]/1000.0
                robot.ang   = struct.unpack_from('<h', msg.data, 3)[0]/1000.0
                robot.last_tpdo = time.monotonic()


def _joy_thread(device_path, joy, stop, code_lin, code_ang,
                code_dead, code_engage, code_disengage, name_hint):
    while not stop.is_set():
        dev = None
        # Try the fixed device path first, but VERIFY it is actually the gamepad
        # (event numbers can shuffle between boots / replugs).
        if device_path:
            try:
                cand = InputDevice(device_path)
                if name_hint.lower() in cand.name.lower():
                    dev = cand          # event path is correct AND is the F710
                else:
                    dev = None          # something else is at this event number
            except Exception:
                dev = None
        # If the fixed path was wrong/missing, find the gamepad by name.
        if dev is None:
            dev = _find_gamepad(name_hint)
            if dev is None:
                print(f"[joy] no gamepad matching '{name_hint}' — retry 1s", flush=True)
                time.sleep(1.0); continue
            print(f"[joy] using auto-detected {dev.path} ({dev.name})", flush=True)

        print(f"[joy] opened {dev.path}  ({dev.name})", flush=True)

        # cache absinfo ranges for axis normalization
        absinfo = {}
        caps = dev.capabilities().get(ecodes.EV_ABS, [])
        for code, info in caps:
            absinfo[code] = info

        def norm(code, value):
            info = absinfo.get(code)
            if not info:
                return 0.0
            lo, hi = info.min, info.max
            if hi == lo:
                return 0.0
            n = (value - lo) / (hi - lo) * 2.0 - 1.0   # -> -1..1
            if abs(n) < 0.08:   # deadzone
                n = 0.0
            return n

        try:
            for event in dev.read_loop():
                if stop.is_set():
                    break
                if event.type == ecodes.EV_ABS:
                    with joy.lock:
                        if event.code == code_lin:
                            joy.axis_lin = norm(code_lin, event.value)
                        elif event.code == code_ang:
                            joy.axis_ang = norm(code_ang, event.value)
                        joy.last = time.monotonic()
                elif event.type == ecodes.EV_KEY:
                    with joy.lock:
                        if event.code == code_dead:
                            joy.deadman = bool(event.value)
                        elif event.code == code_engage:
                            joy.engage = bool(event.value)
                        elif event.code == code_disengage:
                            joy.disengage = bool(event.value)
                        joy.last = time.monotonic()
        except OSError as e:
            print(f"[joy] device lost: {e} — retry 1s", flush=True)
            time.sleep(1.0)


def _send_thread(bus, cmd, stop, rate_hz, max_v):
    interval = 1.0/rate_hz
    nxt = time.monotonic()+interval
    while not stop.is_set():
        s = nxt - time.monotonic()
        if s > 0:
            time.sleep(s)
        nxt += interval
        with cmd.lock:
            req = cmd.req_state; v = cmd.v; w = cmd.w
        if req is None:
            continue
        sv = v if req == St.AUTO_ACTIVE else 0.0
        sw = w if req == St.AUTO_ACTIVE else 0.0
        try:
            bus.send(can.Message(arbitration_id=AMIGA_RPDO1_COB_ID,
                                 data=_pack(req, sv, sw, max_v),
                                 is_extended_id=False), timeout=0.01)
        except can.CanError as e:
            sys.stdout.write(f"\n[CAN error] {e}\n")


def run(a):
    print(f"\nOpening {a.interface} on {a.channel} …")
    bus = can.interface.Bus(interface=a.interface, channel=a.channel, bitrate=a.bitrate)
    robot=_Robot(); joy=_Joy(); cmd=_Cmd(); stop=threading.Event()

    threading.Thread(target=_recv_thread, args=(bus,robot,stop), daemon=True).start()
    threading.Thread(target=_joy_thread,
                     args=(a.device, joy, stop, a.code_linear, a.code_angular,
                           a.code_deadman, a.code_engage, a.code_disengage, a.name),
                     daemon=True).start()
    threading.Thread(target=_send_thread, args=(bus,cmd,stop,a.rate,a.max_v), daemon=True).start()

    lin_sign = -1.0 if a.invert_linear else 1.0
    ang_sign = 1.0 if a.invert_angular else -1.0   # turn corrected: default inverted

    print(f"\n  Device  : {a.device or '(auto: '+a.name+')'}")
    print(f"  Linear  : ABS code {a.code_linear}  (invert={a.invert_linear})")
    print(f"  Angular : ABS code {a.code_angular}  (invert={a.invert_angular})")
    print(f"  Dead-man: KEY code {a.code_deadman}  (hold to send velocity)")
    print(f"  Engage  : KEY code {a.code_engage}   Disengage: KEY code {a.code_disengage}")
    print(f"\n  Amiga must be in AUTO mode (TPDO1 state=4) before engaging.\n")

    engaged=False; loop_hz=20.0; period=1.0/loop_hz
    try:
        while True:
            t0=time.monotonic()
            with joy.lock:
                ax_lin=joy.axis_lin; ax_ang=joy.axis_ang
                b_dead=joy.deadman;  b_eng=joy.engage; b_dis=joy.disengage
            with robot.lock:
                rs=robot.state; rv=robot.speed; ra=robot.ang; tage=time.monotonic()-robot.last_tpdo

            if b_eng and not engaged:
                engaged=True
                with cmd.lock:
                    cmd.v=cmd.w=0.0
                    cmd.req_state=St.AUTO_ACTIVE if rs==St.AUTO_ACTIVE else St.AUTO_READY
            if b_dis and engaged:
                engaged=False
                with cmd.lock:
                    cmd.req_state=St.MANUAL_READY; cmd.v=cmd.w=0.0

            with cmd.lock:
                req=cmd.req_state
            if engaged:
                if req==St.AUTO_READY and rs in (St.AUTO_READY,St.AUTO_ACTIVE):
                    with cmd.lock: cmd.req_state=St.AUTO_ACTIVE
                elif req==St.AUTO_ACTIVE and rs not in (St.AUTO_READY,St.AUTO_ACTIVE):
                    engaged=False
                    with cmd.lock: cmd.req_state=None; cmd.v=cmd.w=0.0

            if engaged:
                with cmd.lock:
                    if b_dead:
                        cmd.v=lin_sign*ax_lin*a.max_v
                        cmd.w=ang_sign*ax_ang*MAX_W
                    else:
                        cmd.v=0.0; cmd.w=0.0

            with cmd.lock:
                req_disp=cmd.req_state; sv,sw=cmd.v,cmd.w
            tpdo_str=f"tpdo={tage*1000:.0f}ms" if robot.last_tpdo else "no TPDO1"
            req_label=_ST_NAME.get(req_disp,'?') if req_disp is not None else 'SILENT'
            dead_str='DEAD-MAN' if b_dead else '        '
            sys.stdout.write(f"\r\033[K  req={req_label:<12} robot={_ST_NAME.get(rs,'?'):<12} {dead_str}"
                             f"  cmd v={sv:+.2f} w={sw:+.3f}  meas v={rv:+.2f} w={ra:+.3f}  [{tpdo_str}]")
            sys.stdout.flush()

            sl=period-(time.monotonic()-t0)
            if sl>0: time.sleep(sl)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        try:
            bus.send(can.Message(arbitration_id=AMIGA_RPDO1_COB_ID,
                                 data=_pack(St.MANUAL_READY,0.0,0.0,a.max_v),
                                 is_extended_id=False), timeout=0.1)
        except can.CanError:
            pass
        bus.shutdown()
        print("\n\nStopped. CAN bus closed.")


if __name__ == '__main__':
    # default evdev codes for F710 in XInput mode
    ap=argparse.ArgumentParser(description="Amiga joystick teleop over CAN (evdev)")
    ap.add_argument('--channel', default='can0')
    ap.add_argument('--interface', default='socketcan')
    ap.add_argument('--bitrate', default=250000, type=int)
    ap.add_argument('--rate', default=20.0, type=float)
    ap.add_argument('--device', default='/dev/input/event8', help='evdev path; empty for auto-detect by name')
    ap.add_argument('--name', default='F710', help='name hint for auto-detect')
    ap.add_argument('--code-linear',   default=ecodes.ABS_Y,  type=int, help='ABS code fwd/back')
    ap.add_argument('--code-angular',  default=ecodes.ABS_X,  type=int, help='ABS code turn (ABS_X=left stick X for single-stick drive)')
    ap.add_argument('--code-deadman',  default=ecodes.BTN_TL, type=int, help='KEY code dead-man (L1)')
    ap.add_argument('--code-engage',   default=315,  type=int, help='KEY code engage (315=BTN_START on F710 XInput)')
    ap.add_argument('--code-disengage',default=ecodes.BTN_SELECT, type=int)
    ap.add_argument('--max-v', default=0.2, type=float)
    ap.add_argument('--invert-linear', dest='invert_linear', action='store_true', default=True,
                    help='Invert linear axis (F710 stick up = -1; default True)')
    ap.add_argument('--no-invert-linear', dest='invert_linear', action='store_false',
                    help='Disable linear inversion')
    ap.add_argument('--invert-angular', action='store_true')
    ap.add_argument('--list', action='store_true', help='print device capabilities and exit')
    a, _unknown = ap.parse_known_args()   # ignore ROS-injected __name/__log

    if a.list:
        dev = InputDevice(a.device) if a.device else _find_gamepad(a.name)
        print(f"Device: {dev.path}  {dev.name}\n")
        print("ABS axes:")
        for code,info in dev.capabilities().get(ecodes.EV_ABS, []):
            print(f"  code {code:3d}  {ecodes.ABS.get(code,'?'):12s}  min={info.min} max={info.max}")
        print("\nKEY buttons:")
        for code in dev.capabilities().get(ecodes.EV_KEY, []):
            names = ecodes.BTN.get(code) or ecodes.KEY.get(code) or '?'
            print(f"  code {code:3d}  {names}")
        sys.exit(0)

    run(a)
