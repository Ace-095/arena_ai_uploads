# Simulation Setup — Gazebo Harmonic + ArduPilot SITL + ROS 2

> **Platform:** Ubuntu 22.04 (dev machine only — nothing here runs on the Raspberry Pi).
> **Audience:** any developer who needs to run the Phase 1+ simulation environment.

---

## Prerequisites

### 1. ArduPilot SITL

Install ArduPilot and its Python toolchain. Follow the official guide:
<https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html>

After installation, confirm:
```bash
sim_vehicle.py --help | head -5
```

### 2. Gazebo Harmonic (LTS)

```bash
export GZ_VERSION=harmonic

# Gazebo Harmonic system libs
sudo apt install libgz-sim8-dev rapidjson-dev

# GStreamer + OpenCV (needed by ardupilot_gazebo plugin)
sudo apt install libopencv-dev \
                 libgstreamer1.0-dev \
                 libgstreamer-plugins-base1.0-dev \
                 gstreamer1.0-plugins-bad \
                 gstreamer1.0-libav \
                 gstreamer1.0-gl
```

Verify:
```bash
gz sim --version        # should print Gazebo Harmonic (8.x)
```

### 3. ardupilot_gazebo plugin

This is the **current** official plugin. Do not use tutorials referencing the
old `gazebo_sitl` package — those are stale.

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

Add to your shell profile (`~/.bashrc` or `~/.zshrc`):
```bash
# Replace /path/to with the actual clone location
export AP_GZ_DIR=/path/to/ardupilot_gazebo
export GZ_SIM_SYSTEM_PLUGIN_PATH=$AP_GZ_DIR/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$AP_GZ_DIR/models:$AP_GZ_DIR/worlds:$GZ_SIM_RESOURCE_PATH
```

Also add our repo's sim assets to the resource path:
```bash
export REPO_ROOT=/path/to/m1      # this repository
export GZ_SIM_RESOURCE_PATH=$REPO_ROOT/sim/models:$GZ_SIM_RESOURCE_PATH
```

Source your profile:
```bash
source ~/.bashrc
```

### 4. ROS 2 Humble + gz-ros-bridge (camera path only)

> **Scope:** ROS 2 is a **simulation-only, developer-machine dependency**.
> It is never installed on the Raspberry Pi. Omit this section if you don't
> need camera frame testing.

```bash
# Install ROS 2 Humble per https://docs.ros.org/en/humble/Installation.html
# (Ubuntu 22.04 binary packages recommended)

# Add Gazebo Harmonic rosdep source (non-default pairing)
sudo wget \
  https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list \
  -O /etc/ros/rosdep/sources.list.d/00-gazebo.list
rosdep update

# Install gz-ros2-bridge
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim
```

---

## Running the Simulation

### Step 1 — Generate the QR texture (first time only)

```bash
cd $REPO_ROOT/sim/textures
python3 generate_qr_texture.py
# Output: sim/models/qr_box/materials/textures/qr_code.png
# Verify the QR decodes: zbarimg ../models/qr_box/materials/textures/qr_code.png
```

This step is **only needed once** (or when the payload changes). The PNG is
committed to version control for reproducibility.

### Step 2 — Start SITL with Gazebo physics

In terminal 1:
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

This starts ArduPilot SITL using Gazebo's physics engine instead of its built-in
model. The stock Gazebo `iris` airframe is an **approximation** for software
testing — it is not a substitute for real-frame flight dynamics validation.

SITL still exposes MAVLink on UDP (default `127.0.0.1:14550`). `MavlinkBus`
connects to it exactly as it would connect to plain SITL or a real Pixhawk.

### Step 3 — Launch the world

In terminal 2:
```bash
gz sim $REPO_ROOT/sim/worlds/qr_search_field.sdf
```

You should see:
- Green 60m × 60m field
- Iris vehicle at the origin
- White QR target box at (10m East, 5m North)

### Step 4 — Connect MavlinkBus to SITL

```bash
export MAVLINK_DEVICE=udp:127.0.0.1:14550
# From the backend/ directory:
python3 - <<'EOF'
from mavlink_bus import MavlinkBus
bus = MavlinkBus("udp:127.0.0.1:14550", 115200, timeout_s=10.0)
bus.start()
print(f"Connected. target_system={bus.conn.target_system}, "
      f"target_component={bus.conn.target_component}")
bus.stop()
EOF
```

Both `target_system` and `target_component` must be non-zero for the link to be
live. If you see `None` for either, the heartbeat exchange failed — check that
SITL is running and the UDP port matches.

### Step 5 — Camera topic (requires ROS 2 + gz-ros-bridge)

In terminal 3:
```bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /iris/downward_cam/image_raw@sensor_msgs/msg/Image[gz.msgs.Image
```

In terminal 4:
```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep image
# This shows the actual topic name — update ROS2_CAMERA_TOPIC to match.
ros2 topic echo /iris/downward_cam/image_raw --once
```

> **NEED TEST:** The exact topic name depends on the SDF world file and the
> gz-ros-bridge configuration. Run `ros2 topic list` after launching and
> confirm the name. Set it as:
> ```bash
> export ROS2_CAMERA_TOPIC=/iris/downward_cam/image_raw
> ```

---

## Phase 1 Smoke Test

Run the full Phase 1 acceptance test:
```bash
cd $REPO_ROOT
export MAVLINK_DEVICE=udp:127.0.0.1:14550
export CAMERA_SOURCE=synthetic     # runs without Gazebo
python3 backend/tests/test_phase1_sim_smoke.py
```

Expected output:
```
PASS  mavlink_bus heartbeat
PASS  target_system populated
PASS  SyntheticQRSource: frame shape (480, 640, 3)
PASS  SyntheticQRSource: QR decode matches payload
All Phase 1 smoke tests passed.
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `gz sim` exits immediately | Plugin not found | Check `GZ_SIM_SYSTEM_PLUGIN_PATH` |
| Iris model not found | Resource path missing | Add `ardupilot_gazebo/models` to `GZ_SIM_RESOURCE_PATH` |
| No MAVLink heartbeat | SITL not started | Run `sim_vehicle.py` first |
| `ros2 topic list` shows nothing | Bridge not running | Run the `parameter_bridge` command above |
| QR decode fails | Texture not generated | Run `generate_qr_texture.py` |

---

## Architecture Note

```
SITL (ArduPilot SITL process)
  ↕ UDP MAVLink (14550)
MavlinkBus (backend/mavlink_bus.py)    ← single connection owner

Gazebo (gz sim)
  ↕ gz-ros-bridge
ROS 2 topic (/iris/downward_cam/...)
  ↕ rclpy subscription
GazeboCameraSource (backend/camera_source.py)   ← sim-only, dev machine

SyntheticQRSource (backend/camera_source.py)    ← pure Python, no sim needed
```

The Pi in production only runs `MavlinkBus` + `SyntheticQRSource`/`Picamera2Source`.
No ROS 2 or Gazebo on the Pi, ever.
