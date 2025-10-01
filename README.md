Create a pixhawk_ws for DDS communication by following ardupiulot/px4 website.

After setting up if u cant see ros2 topics even after the following:
1. Set up ROS_DOMAIN_ID
2. Set the correct Serial Port & Baud Rate
3. Ensure DDS is enabled

Try 

```bash 
ros2 daemon stop && ros2 daemon start
```

To run the microros agent
```bash
ros2 run micro_ros_agent micro_ros_agent serial -b 115200 -D /dev/ttyACM1
```
to run pavproxy
```bash
mavproxy.py --master=/dev/ttyACM0 --baudrate 115200
```

```bash
ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"
```

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
```

We will have 3 ws
1. base_ws (Robot Specific)
2. pixhawk_ws (PX4/Ardupilot)
3. isaac_ros_ws

```bash
base_ws/
  src/
    base_description/   # URDF/Xacro + meshes + RViz config; publishes robot_description via RSP
    base_bringup/       # Starts description + TF relays + teleop (twist_mux) + minimal remaps
    base_diagnostics/   # Health/heartbeat, topic-rate & TF checks, battery & thermals
```

Here’s a crisp README you can paste into your repo. It spells out **what each package does**, **what goes inside**, and **how this base workspace interfaces** with `isaac_ros_ws` (autonomy) and `ardu_ws` (AP\_DDS).

---

# base\_ws — Robot Base Workspace (no autonomy)

**Purpose:** Provide the robot’s *plumbing* only — description (URDF/TF), bringup, teleop, and health checks — so the autonomy stack in **`isaac_ros_ws`** (Nav2, NVBlox, perception) can run cleanly against hardware data coming from **`ardu_ws`** (ArduPilot AP\_DDS).

> No mapping, planning, or “intelligence” lives here. This workspace is reusable across robots.

## Workspace layout

```
base_ws/
  src/
    base_description/   # URDF/Xacro + meshes + RViz config; publishes robot_description via RSP
    base_bringup/       # Starts description + TF relays + teleop (twist_mux) + minimal remaps
    base_diagnostics/   # Health/heartbeat, topic-rate & TF checks, battery & thermals
```

Upstream workspaces you must source **before** `base_ws`:

* `isaac_ros_ws` — NVIDIA/Isaac ROS (perception, Nav2, NVBlox, RealSense wrappers, etc.)
* `ardu_ws` — AP\_DDS + micro-ROS agent + AP messages

```bash
source /workspaces/isaac_ros_ws/install/setup.bash
source /workspaces/ardu_ws/install/setup.bash
source /workspaces/base_ws/install/setup.bash
```

---

## Contracts (non-negotiable)

* **Frames:** continuous `map`→`odom` (optional if pure odom) and **always** `odom`→`base_link`.
  Only **one** node may publish `odom→base_link`.
* **TF topics:** Consumers expect `/tf` and `/tf_static`. If AP publishes `/ap/tf*`, relay to the standard names.
* **Odometry:** Autonomy expects `/odom` (`nav_msgs/Odometry`). If AP doesn’t provide it, a tiny adapter must.
* **Velocity:** Autonomy/teleop output `/cmd_vel` → **remap** to `/ap/cmd_vel` for ArduPilot.
* **Time:** All nodes on the same wall clock; avoid mixed `use_sim_time` on hardware.

---

## Package descriptions

### 1) `base_description` — Robot geometry & frames

**Responsibility**

* Define the robot model and canonical frames (`base_footprint`, `base_link`, `imu_link`, `camera_link`, etc.).
* Publish `robot_description` via `robot_state_publisher` (RSP).
* Provide an RViz view for quick visual sanity checks.

**Contains**

```
base_description/
  urdf/        # xacro/urdf files (e.g., ugv_common.xacro, ugv_rover.xacro)
  meshes/      # STL/DAE meshes referenced as package://base_description/meshes/...
  launch/      # view_robot.launch.py (RSP + optional RViz)
  rviz/        # view_description.rviz
  package.xml, CMakeLists.txt  # install urdf/meshes/launch/rviz to share/
```

**Does NOT**

* Publish odom/map TFs.
* Start sensors or autonomy nodes.

---

### 2) `base_bringup` — Minimal runtime wiring on the robot

**Responsibility**

* Start the description (RSP).
* Relay TF if AP publishes under `/ap/tf*` → `/tf*`.
* Set up teleop arbitration (`twist_mux`) and remap `/cmd_vel → /ap/cmd_vel`.
* Optionally include **sensor drivers only** if they aren’t already launched from `isaac_ros_ws` (keep autonomy there).

**Contains**

```
base_bringup/
  launch/
    robot_bringup.launch.py     # RSP + static TFs (if needed) + TF relays
    teleop.launch.py            # joy + teleop_twist_joy + twist_mux
    system.launch.py            # includes bringup + teleop (+ diagnostics)
  config/
    twist_mux.yaml              # teleop vs nav arbitration (if you use teleop)
```

**Typical inclusions**

* `robot_state_publisher` with your xacro.
* `topic_tools/relay` for:

  * `/ap/tf → /tf`
  * `/ap/tf_static → /tf_static`
* `twist_mux` outputting `/cmd_vel`, remapped to `/ap/cmd_vel`.
* (Optional) a 40-line odometry adapter if AP doesn’t publish `/odom`.

**Does NOT**

* Run Nav2, NVBlox, or perception graphs (those live in `isaac_ros_ws`).

---

### 3) `base_diagnostics` — Health & readiness

**Responsibility**

* Emit `/diagnostics` with clear **WARN/ERROR** when plumbing breaks *before* autonomy fails.
* Provide a single “am I ready?” gate for higher stacks.

**Checks worth implementing**

* **AP link:** rate & age on `/ap/twist/filtered` and `/ap/pose/filtered`; heartbeat on any `/ap/*` topic.
* **TF integrity:** single publisher for `odom→base_link`, no frame ID conflicts, TF age < 0.2 s.
* **Sensors:** RealSense/depth/pointcloud ≥ expected Hz; NVBlox inputs (if launched elsewhere) not stale.
* **Time skew:** header stamps not > ±200 ms from system time.
* **Power/Thermals:** parse `/ap/battery` → `sensor_msgs/BatteryState`; Jetson temps (tegrastats) + throttling flag.
* **Command path:** `twist_mux` active source + `/estop` lock line.

**Contains**

```
base_diagnostics/
  src/health_node.py            # diagnostic_updater producers
  config/diagnostics.yaml       # diagnostic_aggregator tree
  launch/diagnostics.launch.py
```

**Does NOT**

* Own business logic; it only reports and gates readiness.

---

## Launching (example)

```bash
# 1) Description + TF relays + teleop mux
ros2 launch base_bringup system.launch.py

# Autonomy runs elsewhere, e.g.:
# 2) Isaac/Perception/Nav (from isaac_ros_ws)
ros2 launch <your_isaac_nav_stack> nav_system.launch.py

# AP_DDS is normally a service/container from ardu_ws, already running.
```

---

## Topic & frame mapping (summary)

| What              | Producer (typical)       | Consumer expects    | Notes                                   |
| ----------------- | ------------------------ | ------------------- | --------------------------------------- |
| TF, TF static     | AP\_DDS → `/ap/tf*`      | `/tf`, `/tf_static` | Relay in `base_bringup` if names differ |
| Odom (`/odom`)    | AP\_DDS **or** adapter   | Nav/Isaac           | Provide `nav_msgs/Odometry`             |
| Cmd vel           | teleop / autonomy        | `/ap/cmd_vel`       | Use `twist_mux`; remap once             |
| Robot description | `base_description` (RSP) | everyone            | Source of truth for frames              |

**Rule:** exactly **one** publisher for `odom→base_link`.

---

## Common pitfalls (and how we avoid them here)

* **Duplicate `odom→base_link` TF** (AP + EKF both publishing) → Single owner policy; diagnostics flags duplicate pubs.
* **TF under `/ap/*`** unseen by other nodes → Relays to `/tf*` in bringup.
* **No Odometry** → Tiny adapter publishes `/odom` from AP pose+twist.
* **Nav2 inside base\_ws** → Forbidden by design; keep autonomy in `isaac_ros_ws`.

---

## Extending to another robot

Swap only:

* new xacro/meshes in `base_description`,
* updated static transforms in `base_bringup`,
* thresholds in `base_diagnostics`.

`isaac_ros_ws` and `ardu_ws` remain untouched. That’s the whole point.

---

## Quick build

```bash
# Underlays
source /workspaces/isaac_ros_ws/install/setup.bash
source /workspaces/ardu_ws/install/setup.bash

# Build & source base_ws
cd /workspaces/base_ws
colcon build --symlink-install
source install/setup.bash
```

---

If you want, I can also drop a minimal `system.launch.py` and `twist_mux.yaml` tailored to your current topics so you can run this end-to-end immediately.
