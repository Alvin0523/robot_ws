# 🤖 Setup Guide: Autonomous Navigation Multi-Domain Robot

This guide walks you through setting up an agentic autonomous navigation system for a multi-domain robot using Isaac ROS, ArduPilot, and ROS2 by @Alvin_0523

---

## 📁 Step 1: Create Workspace Directory Structure

Create the following workspace folders:

- `ardu_ws` - ArduPilot ROS2 workspace
- `isaac_ros-dev` - Isaac ROS development workspace
- `robot_ws` - Robot-specific workspace

---

## ⚙️ Step 2: Configure Jetson Environment

Add the following to your `~/.bashrc` file on the host Jetson:

```bash
export ISAAC_ROS_WS=/home/agx/workspaces/isaac_ros-dev/
export ROS_DOMAIN_ID=30
```

---

## 📷 Step 3: Setup Isaac ROS (Jetson & Sensors)

### 3.1 Visual SLAM Setup

Install assets and run the command outside the container:

```bash
sudo apt install ros-humble-isaac-ros-visual-slam
```

Run the Docker container, build with colcon, and launch the file directly to test.

### 3.2 Nvblox & RealSense Setup

Follow the [RealSense Camera Examples](https://nvidia-isaac-ros.github.io/v/release-3.2/concepts/scene_reconstruction/nvblox/tutorials/tutorial_realsense.html) documentation.

Install assets, run the installation command, then launch the Docker container to run the `realsense_example`.

---

## 🚁 Step 4: Setup Pixhawk with ArduPilot

**Why ArduPilot?** We use ArduPilot (ArduRover) instead of PX4 because it supports encoder reading for wheeled AMRs.

### 4.1 Environment Setup

Follow the [ROS 2 — Dev documentation](https://ardupilot.org/dev/docs/ros2.html). Read the top section and ensure the environment is properly configured.

**⚠️ Important:** Enable DDS in the firmware to allow DDS communication between Pixhawk and Jetson. If your Pixhawk wasn't flashed with DDS enabled, follow the [Building Setup Guide](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux).

### 4.2 Docker Setup for ArduPilot

Follow the Docker installation section in the [ROS 2 documentation](https://ardupilot.org/dev/docs/ros2.html).

Run the following commands inside the container (Ubuntu setup):

```bash
sudo apt update
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# Tip: Add these to your Dockerfile for easier development
```

Test the installation:

```bash
microxrceddsgen -help
```

### 4.3 Build ArduPilot Workspace

```bash
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_dds_tests
```

### 4.4 Create Docker Launch Script

Create a script in `ardu_ws/scripts/` (e.g., `run_docker.sh`):

```bash
#!/bin/bash
### --- Configuration ---
IMAGE_NAME="ardupilot/ardupilot-dev-ros:latest"     # Your built image
CONTAINER_NAME="ardupilot_dds"                  # Container name
HOST_WS="$HOME/workspaces/ardu_ws"                             # Workspace on Jetson
INNER_WS="/workspaces/ardu_ws"                            # Workspace inside container

### --- 1. If container already running → attach ---
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "✅ Container '$CONTAINER_NAME' is already running."
    echo "🔗 Attaching to existing container..."
    docker exec -it $CONTAINER_NAME bash
    exit 0
fi

### --- 2. Remove stopped container (same name) ---
if [ "$(docker ps -aq -f status=exited -f name=$CONTAINER_NAME)" ]; then
    echo "🧹 Removing old stopped container '$CONTAINER_NAME'..."
    docker rm $CONTAINER_NAME > /dev/null
fi

### --- 3. Ensure workspace exists ---
mkdir -p "$HOST_WS"

### --- 4. Run fresh container ---
echo "🚀 Starting ArduPilot ROS2 Dev Container..."
docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --network host \
    --privileged \
    --ipc=host \
    --workdir /workspaces/ardu_ws \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.Xauthority:/root/.Xauthority:rw \
    -v "$HOST_WS":"$INNER_WS" \
    "$IMAGE_NAME"
```

**Note:** For ROS2 setup on Pixhawk firmware, refer to [Setting up the Build Environment](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux). **Ensure DDS is enabled!**

---

## 🤖 Step 5: Setup Robot Workspace

Clone the `robot_ws` repository and build:

```bash
cd ~/robot_ws
colcon build
```

You're now ready to run the full system! 🎉

---

# ▶️ Running the Full Setup

## 🚁 ArduPilot Workspace (`ardu_ws`)

### Terminal 1: Launch Docker Container

Run the Docker script:

```bash
./ardu_ws/scripts/run_docker.sh
```

### Terminal 2: Start Micro-ROS Agent

```bash
ros2 run micro_ros_agent micro_ros_agent serial -b 115200 -D /dev/ttyACM1
```

### Terminal 3: Configure Pixhawk

**Set to Guided Mode:**

```bash
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 15}"
```

**Arm Motors:**

```bash
ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"
```

### Optional: Run MAVProxy (CLI QGC)

```bash
mavproxy.py --master=/dev/ttyACM0 --baudrate 115200
```

---

## 🤖 Robot Workspace (`robot_ws`)

### Terminal 1: Publish Robot State

```bash
ros2 launch robot_bringup robot_state_publisher.launch.py
```

This publishes the TF of the robot frame.

### Terminal 2: Bridge Command Velocity

```bash
ros2 run robot_bridge cmd_velocity_bridge
```

Outputs `cmd_vel` to `/ap/cmd_vel`.

---

## 📷 Isaac ROS Docker

### Terminal 1: Launch RealSense with 3D Reconstruction

```bash
ros2 launch nvblox_example_bringup realsense_example.launch.py
```

### Terminal 2: Launch Nav2

```bash
ros2 launch nvblox_example_bringup htx_nav2.launch.py
```

---

## 🎮 Teleoperation (Optional)

To control the robot via keyboard, run in `robot_ws`:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

**✅ Setup Complete!** Your autonomous navigation system is now running.

---

## ❓ Troubleshooting FAQ

### ROS2 Topics Not Found or Weird Behavior

If all settings are correct but topics aren't appearing or behaving strangely, restart the ROS2 daemon:

```bash
ros2 daemon stop && ros2 daemon start
```

### Running MAVProxy CLI on Jetson

To use MAVProxy as a command-line ground control station:

```bash
mavproxy.py --master=/dev/ttyACM0 --baudrate 115200
```

---

# 🔧 robot_ws — Robot Base Workspace

**Purpose:** Provides the robot's *core infrastructure* — URDF description, TF tree, bringup scripts, teleoperation, and diagnostics — enabling the autonomy stack in `isaac_ros_ws` (Nav2, NVBlox, perception) to interface cleanly with hardware data from `ardu_ws` (ArduPilot AP_DDS).

> This workspace contains **no autonomy logic** (no mapping, planning, or decision-making). It is purely the robot's hardware abstraction layer and is reusable across different robot platforms.
> 

## 📦 Package Structure

```
robot_ws/
  src/
    robot_description/   # URDF/Xacro + meshes + RViz config; publishes robot_description via robot_state_publisher
    robot_bringup/       # Launch files for description + TF relays + teleop (twist_mux) + topic remapping
    robot_bridge/        # Bridges between standard ROS2 topics and ArduPilot-specific topic
```

## 🔗 Workspace Dependencies

You must source these upstream workspaces **before** building or running `robot_ws`:

- `isaac_ros_ws` — NVIDIA Isaac ROS stack (perception, Nav2, NVBlox, RealSense drivers)
- `ardu_ws` — ArduPilot DDS + micro-ROS agent + ArduPilot message definitions

---

## ⚙️ System Contracts (Must-Have Requirements)

These are **non-negotiable** requirements for proper integration with the autonomy stack:

### 1. TF Tree Structure

- **Required frames:** `map` → `odom` → `base_link`
- **Rule:** Only **one** node may publish the `odom` → `base_link` transform
- Additional frames: `base_footprint`, `imu_link`, `camera_link`, sensor frames

### 2. TF Topics

- All TF data must be published on `/tf` and `/tf_static`
- If ArduPilot publishes on `/ap/tf` and `/ap/tf_static`, relay to standard topic names

### 3. Odometry Topic

- Nav2 expects `/odom` (`nav_msgs/Odometry`)
- If ArduPilot doesn't provide standard odometry, create an adapter node

### 4. Command Velocity Mapping

- Autonomy/teleop outputs: `/cmd_vel` (`geometry_msgs/Twist`)
- ArduPilot expects: `/ap/cmd_vel`
- **Solution:** Remap or bridge `/cmd_vel` → `/ap/cmd_vel` in `robot_bridge`

### 5. Time Synchronization

- All nodes must use the same wall clock time
- Avoid mixing `use_sim_time:=true` and `use_sim_time:=false` on real hardware
- Ensure Jetson system time is synchronized (use NTP if needed)

---
