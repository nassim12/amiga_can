# amiga_can

ROS Noetic package for autonomous and joystick teleoperation of the [Farm-ng Amiga](https://farm-ng.com/products/amiga) wheeled robot via a Peak PCAN-USB adapter and SocketCAN on a Jetson AGX Orin.

Developed at the **Precision Agriculture Laboratory, Central State University**.

---

## System overview

```
(navigate.py) or / joystick (F710) current method
    └─ /cmd_vel  (geometry_msgs/Twist)
         └─ amiga_twist_to_can  (ROS node, 20 Hz heartbeat)
              └─ SocketCAN  (can0, 250 kbps)
                   └─ Peak PCAN-USB
                        └─ Farm-ng Amiga CAN bus
                             └─ Dashboard MCU  (CANopen node 0x0E)
                                  └─ Motor controllers
```

For joystick teleop, `amiga_joy_teleop_evdev.py` replaces the ROS node — it reads the gamepad directly via `evdev` and writes CAN frames at 20 Hz without needing a ROS master.

### Why direct CAN instead of the farm-ng gRPC bridge?

The farm-ng `amiga_ros_bridge` requires the Amiga's onboard gRPC service and adds a process dependency. Direct SocketCAN is lower-latency, requires no network configuration, and is fully self-contained. The dashboard firmware accepts RPDO1 state and velocity commands from any authorised CAN node.

### Why 20 Hz heartbeat?

The Amiga dashboard has a watchdog: if RPDO1 frames stop or drop below ~5 Hz, it reverts from `AUTO_ACTIVE` to `AUTO_READY` and the robot stops. Sending only on `/cmd_vel` callbacks is insufficient when the navigation policy publishes at 3–4 Hz. The node re-broadcasts the last command at 20 Hz and sends `MANUAL_READY` if no fresh command arrives within 0.5 s.

### Why evdev instead of `/dev/input/js0`?

The Jetson AGX Orin kernel does not expose a `jsX` device for the Logitech F710 in XInput mode. The `evdev` interface (`/dev/input/eventX`) works correctly and also provides calibrated axis ranges for normalisation and deadzone handling.

---

## Hardware

| Component | Specification |
|---|---|
| Robot | Farm-ng Amiga (CAN bus, 250 kbps) |
| Compute | NVIDIA Jetson AGX Orin, JetPack 5.x |
| CAN adapter | Peak PCAN-USB (`peak_usb` kernel module) |
| Gamepad | Logitech F710, **XInput mode** (switch on back of pad) |

---

## Software dependencies

```bash
sudo apt-get install -y python3-catkin-tools can-utils ros-noetic-topic-tools
sudo pip3 install python-can python-evdev
```

---

## Installation

```bash
# Copy package into existing catkin workspace
cp -r amiga_can ~/catkin_ws/src/

cd ~/catkin_ws
catkin build amiga_can
source devel/setup.bash
```

---

## CAN interface setup

Run once after each reboot (requires sudo):

```bash
sudo bash ~/catkin_ws/src/amiga_can/setup_can.sh
```

To run automatically at boot, add to `/etc/rc.local`:

```bash
bash /home/<user>/catkin_ws/src/amiga_can/setup_can.sh
```

---

## Running

### Joystick teleop ✓ — confirmed working on Jetson

```bash
roslaunch amiga_can amiga_joy_evdev.launch
```

Or standalone (no ROS master required):

```bash
python3 scripts/amiga_joy_teleop_evdev.py --max-v 0.4
```

**Logitech F710 controls (XInput mode):**

| Input | Action |
|---|---|
| **Start** | Engage — request `AUTO_ACTIVE` |
| **Back** | Disengage — return to `MANUAL_READY` |
| **LB (L1)** | Dead-man switch — **hold** to send velocity |
| Left stick Y | Forward / back |
| Left stick X | Turn left / right |

> The Amiga **pendant must be in auto mode** before engaging (TPDO1 state = 4 `AUTO_READY`).

To list all axis/button codes for a different pad:

```bash
python3 scripts/amiga_joy_teleop_evdev.py --list
```

### Autonomous navigation (ROS) ⚠️ — not yet field-tested

Start the CAN bridge node alongside your navigation policy:

```bash
roslaunch amiga_can amiga_can.launch
```

The node subscribes to `/cmd_vel` and forwards commands to the Amiga at 20 Hz.
Verify on the bench first: confirm TPDO1 state reaches 5 (`AUTO_ACTIVE`) and the measured velocity tracks `cmd_vel` before running in the field.

### CAN sniffer (diagnostics) ⚠️ — untested

```bash
# All frames
python3 scripts/amiga_can_sniffer.py

# Dashboard status (TPDO1 — robot state, measured speed)
python3 scripts/amiga_can_sniffer.py --filter 0x18E

# Our commands (RPDO1 — state request, commanded speed)
python3 scripts/amiga_can_sniffer.py --filter 0x20E
```

---

## CAN protocol reference

### AmigaControlState

| Value | Name | Description |
|---|---|---|
| 1 | `MANUAL_READY` | Pendant idle |
| 2 | `MANUAL_ACTIVE` | Pendant joystick driving |
| 3 | `CC_ACTIVE` | Cruise control |
| **4** | **`AUTO_READY`** | Pendant in auto mode, ready for CAN commands |
| **5** | **`AUTO_ACTIVE`** | Executing CAN velocity commands |
| 6 | `ESTOPPED` | E-stop active |

### Frame layout (RPDO1 `0x20E` / TPDO1 `0x18E`)

| Bytes | Type | Field | Scale |
|---|---|---|---|
| 0 | `uint8` | state / state_req | — |
| 1–2 | `int16 LE` | speed | mm/s → ÷ 1000 = m/s |
| 3–4 | `int16 LE` | angular rate | mrad/s → ÷ 1000 = rad/s |
| 5 | `uint8` | PTO bits | — |
| 6 | `uint8` | H-bridge bits | — |
| 7 | `uint8` | padding (RPDO1) / SoC % (TPDO1) | — |

---

## Troubleshooting

**Robot stays at `AUTO_READY`, does not accept `AUTO_ACTIVE`**
- Confirm pendant is in auto mode — sniffer should show TPDO1 `state=4`
- Check RPDO1 is being sent: `python3 scripts/amiga_can_sniffer.py --filter 0x20E`

**`can0` not found after `modprobe peak_usb`**
- Check USB connection: `lsusb | grep -i peak`
- Re-run `setup_can.sh`

**F710 not detected**
- Confirm XInput mode (green LED on pad, not red)
- Check event device: `python3 scripts/amiga_joy_teleop_evdev.py --list`

**Robot stops mid-navigation**
- Watchdog triggered: navigation policy publish rate dropped below heartbeat tolerance
- Reduce watchdog timeout: `roslaunch amiga_can amiga_can.launch watchdog_timeout:=1.0`

```

## License

MIT © Precision Agriculture Laboratory, Central State University
