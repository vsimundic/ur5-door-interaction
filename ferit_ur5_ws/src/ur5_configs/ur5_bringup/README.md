# UR5 Hardware Bringup

Full hardware bringup for the UR5 arm + all peripherals:
- **UR5** robot driver
- **MoveIt** planning execution
- **XELA USP44** tactile sensor (via CAN-over-serial)
- **Robotiq FT sensor** streamer
- **RealSense** depth camera

---

## Prerequisites (one-time, on the host)

### 1 — Install the XELA udev rule

This creates a stable `/dev/ttyXELA` symlink for the XELA USB-CAN adapter regardless of port order.

```bash
sudo cp /path/to/ferit_ur5_ws/src/ur5_configs/xela_server_ros/scripts/99-xela-can.rules \
        /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

ls -la /dev/ttyXELA   # verify symlink was created
```

> **Adapter details:** FTDI FT231X — Vendor `0403`, Product `6015`, Serial `D30F21NT` (VScom USB-CAN Plus)

---

## Every-session workflow

### Step 1 — On the HOST: bring up the CAN interface

Plug in the XELA USB-CAN adapter, then run:

```bash
src/ur5_configs/xela_server_ros/scripts/setup_xela_can.sh
```

This script:
1. Validates `/dev/ttyXELA` exists
2. Tears down any leftover `slcan0` interface / `slcand` process
3. Starts `slcand -o -s8 -t hw -S 3000000 /dev/ttyXELA slcan0`
4. Brings up the `slcan0` network interface

You can also pass an explicit device if needed:
```bash
src/ur5_configs/xela_server_ros/scripts/setup_xela_can.sh /dev/ttyUSB2
```

### Step 2 — Inside the container: launch everything

```bash
roslaunch ur5_bringup hardware_launch.launch
```

This starts all five subsystems in one shot. Commonly overridden arguments:

| Argument | Default | Description |
|---|---|---|
| `robot_ip` | `192.168.88.245` | UR5 controller IP |
| `pc_ip` | `192.168.88.249` | This machine's IP (for XELA server) |
| `kinematics_config` | `~/ferit_ur5_ws/ur5_calibration.yaml` | Calibration file |
| `xserv_ini` | `/etc/xela/xServ.ini` | XELA server config |
| `xserv_port` | `5000` | XELA server port |

Example with overrides:
```bash
roslaunch ur5_bringup hardware_launch.launch robot_ip:=192.168.88.245 pc_ip:=192.168.88.249
```

---

## File layout

```
ur5_configs/
├── ur5_bringup/
│   ├── launch/
│   │   └── hardware_launch.launch   ← main launch file (this package)
│   └── README.md                    ← this file
└── xela_server_ros/
    └── scripts/
        ├── 99-xela-can.rules        ← udev rule  (install once on host)
        └── setup_xela_can.sh        ← CAN setup  (run each session on host)
```

---

## Troubleshooting

**`/dev/ttyXELA` not found**
- Check the adapter is plugged in: `lsusb | grep FTDI`
- Verify the udev rule is installed: `ls /etc/udev/rules.d/99-xela-can.rules`
- Re-trigger udev: `sudo udevadm trigger`

**`slcan0` already exists / slcand won't start**
- `setup_xela_can.sh` handles this automatically, but if needed manually:
  ```bash
  sudo ip link set slcan0 down
  sudo pkill slcand
  ```

**XELA server fails inside container**
- Confirm `slcan0` is up on the host: `ip link show slcan0`
- The `slcan0` interface must exist on the host **before** the container starts (or the container must share the host network namespace).

**UR5 driver can't connect**
- Verify robot is powered on and reachable: `ping 192.168.88.245`
- Check the external control program is loaded and running on the teach pendant.
