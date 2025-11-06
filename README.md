# ecal_odometry_mavlink_bridge

A tiny bridge that converts **eCAL VIO odometry** into **MAVLink** messages and transports them over **serial**, **UDP**, or **TCP** using MAVSDK.

---

## TL;DR

```bash
# Serial to an autopilot @ 921600 baud (VK180)
./ecal_odometry_mavlink_bridge serial:///dev/ttyUSB0:921600 0

# UDP to a simulator on the same machine (ArduPilot / PX4 SITL)
./ecal_odometry_mavlink_bridge udp://:14540 0

# TCP to an autopilot
./ecal_odometry_mavlink_bridge tcp://127.0.0.1:5760 1

# With optional GCS forwarding (3rd arg):
./ecal_odometry_mavlink_bridge udp://:14540 0 udp://127.0.0.1:14550
```

* **2nd arg**: `0` for **VK180**, `1` for **VK180P** (used as `tf_prefix = "S<arg>/"`, e.g., `S0/`).
* **3rd arg (optional)**: a **secondary connection** (e.g., a GCS). If present, the bridge forwards MAVLink to it as well.

---

## Usage

```
Usage : ./ecal_odometry_mavlink_bridge <connection_url> <0|1> [<gcs_url>]
Connection URL format:
  TCP    : tcp://[server_host][:server_port]
  UDP    : udp://[bind_host][:bind_port]
  Serial : serial:///path/to/serial/dev[:baudrate]
Example : ./ecal_odometry_mavlink_bridge udp://:14540 0
```

### Connection examples

* **PX4 SITL (default UDP port)**

  ```bash
  ./ecal_odometry_mavlink_bridge udp://:14540 0
  ```
* **ArduPilot SITL**

  ```bash
  ./ecal_odometry_mavlink_bridge udp://:14550 0
  ```
* **Serial to hardware autopilot**

  ```bash
  ./ecal_odometry_mavlink_bridge serial:///dev/ttyACM0:115200 1
  ```
* **Forward to a GCS as well**

  ```bash
  ./ecal_odometry_mavlink_bridge udp://:14540 0 udp://127.0.0.1:14550
  ```

---

## What it does (data flow)

```
eCAL (VIO)  -->  Bridge  -->  MAVSDK  -->  Autopilot (UART / UDP / TCP)

Topics read from eCAL (with tf prefix S<id>/):
  S<id>/local_position_ned        (vkc::Odometry3d)
  S<id>/local_position            (vkc::Odometry3d)
  S<id>/vio_odom_ned              (vkc::Odometry3d)

Topics emitted to eCAL:
  S<id>/mav_state                 (vkc::MavState)
```

* `id` is `0` for VK180, `1` for VK180P.
* The bridge subscribes to MAVSDK **telemetry** (e.g., NED position, quaternion, odometry, armed state, flight mode, battery) and republishes a compact **MavState** to eCAL.
* The bridge waits for **MAVSDK time sync** before streaming.

---

## Build & Package (simple)

1. Initialize submodules

```bash
git submodule update --init --recursive
```

2. Cache/build MAVSDK and other deps

```bash
python3 pre_build.py
```

3. Package the .deb

```bash
cd ecal_mavlink_bridge
mkdir build && cd build
cmake ..
make
make package
```

---

## Runtime notes

* The program blocks until **time sync converges**:

  ```cpp
  while (!system->is_timesync_converged()) { sleep(1); }
  ```
* `tf_prefix` is created from the 2nd arg: `"S" + argv[2] + "/"` (e.g., `S0/`).
* If a 3rd URL is provided, the bridge opens a **second MAVLink connection** (useful for GCS).

---

## Troubleshooting

* **No data / stuck at ~hundreds of ms offset**: ensure only one clock discipliner is active (stop chrony/ntpd during tests) and let MAVSDK time sync finish.
* **Linker error: *file in wrong format***: wipe `cache/MAVSDK-install` and rebuild MAVSDK for the **current architecture**.
* **Serial not opening**: check permissions (`sudo usermod -aG dialout $USER`), correct `/dev/tty*` and baud.
* **No VIO topics**: verify your eCAL sources publish `S<id>/vio_odom_ned` with `vkc::Odometry3d` schema.

