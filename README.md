# 🚁 ROS2 Autonomous Drone Delivery System

> A complete **ROS 2 Jazzy Jalisco** quadrotor simulation and autonomous delivery system built on **sjtu_drone** (Gazebo Harmonic) with a custom **AeroPin** coordinate mission controller for precision waypoint navigation and delivery operations.

---

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [How the Drone Works](#how-the-drone-works)
- [Control System — PID & PD Controllers](#control-system--pid--pd-controllers)
- [AeroPin Concept](#aeropin-concept)
- [Package Structure](#package-structure)
- [Prerequisites](#prerequisites)
- [Installation & Build](#installation--build)
- [Launch Commands](#launch-commands)
- [ROS 2 Topics Reference](#ros-2-topics-reference)
- [PID Tuning Parameters](#pid-tuning-parameters)
- [Troubleshooting](#troubleshooting)

---

## Overview

This repository combines the **sjtu_drone** ROS 2 Gazebo quadcopter simulator with a custom autonomous delivery package (`drone_delivery_system`) that implements the **AeroPin** mission model — a pin-point coordinate-based delivery paradigm where the drone autonomously navigates between world-frame waypoints, drops a payload, and returns to base.

**Stack:**

| Component | Version |
|---|---|
| ROS 2 | **Jazzy Jalisco** |
| Ubuntu | **24.04 LTS (Noble Numbat)** |
| Gazebo | **Harmonic** (gz-harmonic) |
| Python | 3.12 |

**Key capabilities:**
- Full 6-DOF quadrotor simulation in Gazebo Harmonic
- Cascaded PID/PD flight control loops (position → velocity → attitude)
- AeroPin waypoint mission planner with real-time state machine
- Autonomous takeoff, navigation, hover, delivery, and landing
- Sensor feedback: IMU, sonar, GPS, front/bottom cameras

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROS 2 Node Graph                         │
│                                                                 │
│  ┌──────────────┐     /drone/gt_pose      ┌──────────────────┐ │
│  │  Gazebo      │ ──────────────────────► │  coordinate_     │ │
│  │  Harmonic /  │     /drone/state        │  mission_        │ │
│  │  sjtu_drone  │ ──────────────────────► │  controller      │ │
│  │  plugin      │                         │  (AeroPin Node)  │ │
│  │              │ ◄────────────────────── │                  │ │
│  └──────────────┘   /drone/cmd_vel        └──────────────────┘ │
│         │           /drone/takeoff               │             │
│         │           /drone/land                  │             │
│         ▼                                        ▼             │
│  ┌──────────────┐                     ┌──────────────────────┐ │
│  │  gz sim UI   │                     │  Mission Waypoints   │ │
│  │  + RViz2     │                     │  (AeroPin Coords)    │ │
│  └──────────────┘                     └──────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

> **Jazzy Note:** Gazebo Harmonic uses the new `gz sim` command (not `gazebo` or `gzserver`). The `ros_gz` bridge handles topic translation between Gazebo and ROS 2.

---

## How the Drone Works

The quadrotor is a **four-rotor helicopter** (quadcopter). Each rotor spins at an independently controlled speed. By varying the relative RPM of the four motors, the drone achieves full 6-DOF control:

| Motion | Mechanism |
|---|---|
| **Throttle (Altitude)** | All 4 rotors increase/decrease together |
| **Roll (Left/Right)** | Left rotors vs right rotors differential |
| **Pitch (Forward/Backward)** | Front rotors vs rear rotors differential |
| **Yaw (Rotation)** | Diagonal pairs spin in opposite directions; changing pair balance rotates the body |

### Flight State Machine

```
LANDED ──► TAKEOFF ──► FLYING ──► HOVERING ──► MISSION ──► LANDING ──► LANDED
  (0)         │           (1)         (2)                              (0)
              │
              └── /drone/takeoff topic triggers transition
```

The `plugin_drone` Gazebo plugin simulates realistic aerodynamics including:
- Rotor thrust proportional to RPM²
- Drag forces in all axes
- Optional motion noise and drift noise for realistic sensor simulation

---

## Control System — PID & PD Controllers

The drone uses a **cascaded (nested) control loop** architecture — the outer loop generates velocity setpoints that the inner loop tracks, and the inner loop generates attitude setpoints that the attitude controller tracks.

### Cascade Control Architecture

```
                    OUTER LOOP                          INNER LOOP
                  (Position PID)                     (Velocity PD)
                                                                        
 Desired ──►  ┌────────────┐  velocity  ┌────────────┐  thrust/  ┌──────────┐
 Position     │ Position   │ setpoint ► │ Velocity   │ torque ►  │  Drone   │
              │ PID        │            │ PD         │           │  Plant   │
              └────────────┘            └────────────┘           └──────────┘
                    ▲                         ▲                       │
                    │  actual pose            │  actual velocity      │
                    └─────────────────────────┴───────────────────────┘
                                    Sensor Feedback (IMU + GPS)
```

### 1. Position Controller (PID — X, Y, Z)

Controls **where** the drone is in 3D space.

**For horizontal position (X, Y):**

```
error_xy     = desired_position_xy - actual_position_xy
velocity_cmd = Kp_xy × error_xy
             + Ki_xy × ∫error_xy dt         (Ki = 0.0, effectively PD)
             + Kd_xy × d(error_xy)/dt
```

**For vertical position (Z / altitude):**

```
error_z    = desired_altitude - actual_altitude
velocity_z = Kp_z × error_z
           + Ki_z × ∫error_z dt
           + Kd_z × d(error_z)/dt
```

| Parameter | Value | Description |
|---|---|---|
| `positionXYProportionalGain` | 1.1 | Horizontal stiffness — how hard it corrects XY error |
| `positionXYDifferentialGain` | 0.0 | Horizontal damping (0 = pure P for XY) |
| `positionXYIntegralGain` | 0.0 | Steady-state XY correction (off by default) |
| `positionXYLimit` | 5.0 | Max horizontal velocity command (m/s) |
| `positionZProportionalGain` | 1.0 | Altitude stiffness |
| `positionZDifferentialGain` | 0.2 | Altitude damping |
| `positionZIntegralGain` | 0.0 | Altitude steady-state correction |

### 2. Velocity Controller (PD — Vx, Vy, Vz)

Controls **how fast** the drone moves. Translates velocity commands from the position loop into attitude/thrust commands.

```
error_vel = desired_velocity - actual_velocity
output    = Kp_vel × error_vel + Kd_vel × d(error_vel)/dt
```

> **Why PD and not PID here?** The velocity loop intentionally has no integral term to avoid wind-up and ensure fast, responsive dynamics. The position loop above handles any steady-state error that builds up over time.

| Parameter | Value | Description |
|---|---|---|
| `velocityXYProportionalGain` | 5.0 | Horizontal velocity tracking gain |
| `velocityXYDifferentialGain` | 2.3 | Dampens oscillations in horizontal velocity |
| `velocityXYLimit` | 2.0 | Clamps the output roll/pitch angle command |
| `velocityZProportionalGain` | 5.0 | Vertical velocity tracking gain |
| `velocityZDifferentialGain` | 1.0 | Vertical velocity damping |
| `velocityZLimit` | -1 | Unlimited (negative = no cap) |

### 3. Attitude Controller (PD — Roll, Pitch, Yaw)

Controls the **tilt angle** of the drone. Runs at the fastest rate (innermost loop).

```
roll_cmd  = Kp_rp  × roll_error  + Kd_rp  × roll_rate_error
pitch_cmd = Kp_rp  × pitch_error + Kd_rp  × pitch_rate_error
yaw_cmd   = Kp_yaw × yaw_error   + Kd_yaw × yaw_rate_error
```

| Parameter | Value | Description |
|---|---|---|
| `rollpitchProportionalGain` | 10.0 | Attitude stiffness — higher = snappier response |
| `rollpitchDifferentialGain` | 5.0 | Attitude damping — prevents oscillation |
| `rollpitchLimit` | 0.5 | Max tilt angle (radians ≈ 28.6°) |
| `yawProportionalGain` | 2.0 | Yaw heading stiffness |
| `yawDifferentialGain` | 1.0 | Yaw rate damping |
| `yawLimit` | 1.5 | Max yaw rate (rad/s) |

### Why Cascaded PID?

```
Benefit                        Explanation
───────────────────────────────────────────────────────────────────
Separation of timescales  →   Position ~10 Hz, velocity ~50 Hz, attitude ~100 Hz
Easier to tune            →   Each loop is tuned independently
Built-in safety limits    →   Velocity/attitude limits prevent aggressive maneuvers
Physical intuition        →   Each layer maps to a measurable physical quantity
```

### Control Mode Switching

The drone supports two modes toggled via the `/drone/posctrl` topic:

- **`False` (default)** — Normal control: `cmd_vel` directly commands velocity or tilt.
- **`True`** — Position control: `cmd_vel` is interpreted as a desired *pose*; the cascaded PID loops close around position error.

The `coordinate_mission_controller` (AeroPin node) operates in **position control mode**, publishing target waypoints and letting the PID cascade fly the drone there automatically.

---

## AeroPin Concept

**AeroPin** is the waypoint mission paradigm implemented in the `drone_delivery_system` package. The idea is analogous to "dropping a pin" on a map — the operator defines world-frame pin coordinates and the drone autonomously navigates to each pin in sequence to complete a delivery mission.

### AeroPin State Machine

```
                    ┌─────────────┐
                    │    IDLE     │
                    │  (Waiting)  │
                    └──────┬──────┘
                           │ Mission Start
                           ▼
                    ┌─────────────┐
                    │   TAKEOFF   │ ──► pub /drone/takeoff
                    └──────┬──────┘
                           │ Altitude reached
                           ▼
              ┌────────────────────────┐
              │  NAVIGATE TO WAYPOINT  │ ──► pub /drone/cmd_vel
              │    (AeroPin #N)        │     (position control mode)
              └────────────┬───────────┘
                           │ Within arrival radius
                           ▼
                    ┌─────────────┐
                    │    HOVER    │ ──► Hold position
                    │  & DELIVER  │     Drop payload
                    └──────┬──────┘
                           │
              ┌────────────┴────────────┐
              │ More waypoints?         │
              │  YES ──► Next AeroPin   │
              │  NO  ──► Return Home    │
              └─────────────────────────┘
                           │
                           ▼
                    ┌─────────────┐
                    │   LANDING   │ ──► pub /drone/land
                    └─────────────┘
```

### AeroPin Coordinate System

Each "pin" is a 3D coordinate `(x, y, z)` in the Gazebo world frame:

```
World Frame (Gazebo Harmonic):
   Z ▲   ▲ Y
     │  /
     │ /
     └────► X

AeroPin[0] = (x0, y0, z0)   ← Home / Launch Pad
AeroPin[1] = (x1, y1, z1)   ← Delivery Point 1
AeroPin[2] = (x2, y2, z2)   ← Delivery Point 2
...
AeroPin[N] = (x0, y0, z0)   ← Return to Home
```

### AeroPin Arrival Detection

```python
distance = sqrt((current_x - pin_x)² + (current_y - pin_y)² + (current_z - pin_z)²)

if distance < ARRIVAL_RADIUS:   # typically 0.3–0.5 m
    advance_to_next_state()
```

---

## Package Structure

```
ros2_drone/
├── sjtu_drone_bringup/              # Launch files & simulation config
│   ├── launch/
│   │   └── sjtu_drone_bringup.launch.py   ← Main simulation launcher
│   └── config/
│       └── drone_params.yaml              ← PID gains & drone config
│
├── sjtu_drone_description/          # Drone URDF/SDF, Gazebo plugin, physics
│   ├── urdf/
│   │   └── sjtu_drone.urdf
│   └── src/
│       └── plugin_drone.cpp               ← Cascaded PID/PD implementation
│
├── sjtu_drone_control/              # Basic teleop & topic examples
│
└── drone_delivery_system/           # AeroPin mission system (custom)
    ├── drone_delivery_system/
    │   └── coordinate_mission_controller.py   ← AeroPin node
    ├── launch/
    └── config/
        └── mission_waypoints.yaml             ← Pin coordinates
```

---

## Prerequisites

| Requirement | Version |
|---|---|
| **Ubuntu** | **24.04 LTS (Noble Numbat)** |
| **ROS 2** | **Jazzy Jalisco** |
| **Gazebo** | **Harmonic (gz-harmonic)** |
| Python | 3.12 |
| `xterm` | Any |

> ⚠️ **Do NOT use Ubuntu 22.04 or Gazebo 11 (Classic) with this setup.** ROS 2 Jazzy officially pairs with **Gazebo Harmonic** (`gz sim`), not the old `gazebo`/`gzserver` commands.

### Install ROS 2 Jazzy

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install -y software-properties-common curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${VERSION_CODENAME})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ROS 2 Jazzy Desktop (includes Gazebo Harmonic)
sudo apt update && sudo apt install -y ros-jazzy-desktop

# Install colcon and rosdep
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init && rosdep update
```

### Install Gazebo Harmonic + ROS 2 bridge

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-rviz2 \
  xterm
```

### Source ROS 2 Jazzy (add to ~/.bashrc)

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Installation & Build

### 1. Set up your ROS 2 workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone the repository

```bash
git clone https://github.com/sachinmandal3580-rgb/ros2_drone.git .
```

### 3. Install package dependencies

```bash
cd ~/ros2_ws
rosdep install -r -y --from-paths src --ignore-src --rosdistro jazzy
```

### 4. Build all packages

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

> **Tip:** To build only the relevant packages:
> ```bash
> colcon build --symlink-install \
>   --packages-select sjtu_drone_bringup sjtu_drone_description \
>                     sjtu_drone_control drone_delivery_system
> ```

### 5. Source the workspace

```bash
source ~/ros2_ws/install/setup.bash
```

Add to `~/.bashrc` so it's always sourced:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Launch Commands

### Step 1 — Launch the Simulation

Open a terminal and run:

```bash
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
```

This single command:
- Starts **Gazebo Harmonic** (`gz sim`) with the drone world
- **Spawns the quadrotor** model with the `plugin_drone` cascaded PID controller
- Opens an **xterm teleop window** for optional manual keyboard control
- Launches **RViz2** with the drone sensor visualization preset

Wait until Gazebo fully loads and the drone model appears in the simulation before running Step 2.

---

### Step 2 — Run the AeroPin Mission Controller

Open a **new terminal** (workspace already sourced via `.bashrc`) and run:

```bash
ros2 run drone_delivery_system coordinate_mission_controller
```

This starts the AeroPin node which will autonomously:

1. Enable **position control mode** on the drone (`/drone/posctrl → true`)
2. Publish `/drone/takeoff` to command **autonomous takeoff**
3. Navigate through configured **AeroPin waypoints** one by one
4. **Hover and deliver** payload at each target coordinate
5. Return to home position and publish `/drone/land` to **land autonomously**

---

### Optional Manual Control Commands

These work in any terminal while the simulation is running:

```bash
# Takeoff
ros2 topic pub /drone/takeoff std_msgs/msg/Empty {} --once

# Land
ros2 topic pub /drone/land std_msgs/msg/Empty {} --once

# Enable position control mode (required before sending pose targets)
ros2 topic pub /drone/posctrl std_msgs/msg/Bool "data: true" --once

# Fly to a specific position (x=2, y=0, z=1.5 meters)
ros2 topic pub /drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 2.0, y: 0.0, z: 1.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# Check current flight state (0=landed, 1=flying, 2=hovering)
ros2 topic echo /drone/state

# View live position
ros2 topic echo /drone/gt_pose

# Reset drone to origin
ros2 topic pub /drone/reset std_msgs/msg/Empty {} --once
```

---

## ROS 2 Topics Reference

### Topics Published by the Drone

| Topic | Type | Description |
|---|---|---|
| `/drone/gt_pose` | `geometry_msgs/msg/Pose` | Ground truth 3D position and orientation |
| `/drone/gt_vel` | `geometry_msgs/msg/Twist` | Ground truth linear and angular velocity |
| `/drone/gt_acc` | `geometry_msgs/msg/Twist` | Ground truth acceleration |
| `/drone/state` | `std_msgs/msg/Int8` | Flight state: 0=landed, 1=flying, 2=hovering |
| `/drone/cmd_mode` | `std_msgs/msg/Bool` | Current control mode (position or normal) |
| `/drone/imu/out` | `sensor_msgs/msg/Imu` | IMU data (acceleration + angular velocity) |
| `/drone/sonar/out` | `sensor_msgs/msg/Range` | Sonar altitude (downward-facing) |
| `/drone/gps/nav` | `sensor_msgs/msg/NavSatFix` | GPS coordinates |
| `/drone/gps/vel` | `geometry_msgs/msg/TwistStamped` | GPS-derived velocity |
| `/drone/front/image_raw` | `sensor_msgs/msg/Image` | Front-facing camera feed |
| `/drone/bottom/image_raw` | `sensor_msgs/msg/Image` | Downward-facing camera feed |
| `/drone/joint_states` | `sensor_msgs/msg/JointState` | Rotor joint states |

### Topics Subscribed by the Drone

| Topic | Type | Description |
|---|---|---|
| `/drone/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity or position command |
| `/drone/takeoff` | `std_msgs/msg/Empty` | Trigger autonomous takeoff |
| `/drone/land` | `std_msgs/msg/Empty` | Trigger landing |
| `/drone/reset` | `std_msgs/msg/Empty` | Reset drone to world origin |
| `/drone/posctrl` | `std_msgs/msg/Bool` | Toggle position vs. velocity control mode |
| `/drone/dronevel_mode` | `std_msgs/msg/Bool` | Toggle velocity vs. tilt control |

### Topics Used by AeroPin Node

| Topic | Direction | Purpose |
|---|---|---|
| `/drone/gt_pose` | Subscribe | Read current position for waypoint tracking |
| `/drone/state` | Subscribe | Monitor flight state transitions |
| `/drone/cmd_vel` | Publish | Send position commands to each AeroPin |
| `/drone/takeoff` | Publish | Initiate autonomous takeoff |
| `/drone/land` | Publish | Land after mission completion |
| `/drone/posctrl` | Publish | Enable position control mode |

---

## PID Tuning Parameters

Configure gains in `sjtu_drone_bringup/config/drone_params.yaml`:

```yaml
/simple_drone:
  ros__parameters:
    # ── Attitude Control (PD) ──────────────────────────────────────
    rollpitchProportionalGain: 10.0    # Higher → snappier tilt response
    rollpitchDifferentialGain: 5.0     # Higher → less tilt oscillation
    rollpitchLimit: 0.5                # Max tilt angle (rad ≈ 28.6°)

    yawProportionalGain: 2.0           # Yaw tracking stiffness
    yawDifferentialGain: 1.0           # Yaw rate damping
    yawLimit: 1.5                      # Max yaw rate (rad/s)

    # ── Velocity Control (PD) ─────────────────────────────────────
    velocityXYProportionalGain: 5.0    # Horizontal velocity tracking
    velocityXYDifferentialGain: 2.3    # Damps XY velocity oscillations
    velocityXYLimit: 2.0               # Max horizontal speed (m/s)

    velocityZProportionalGain: 5.0     # Vertical velocity tracking
    velocityZDifferentialGain: 1.0     # Damps vertical oscillations
    velocityZLimit: -1                 # -1 = unlimited

    # ── Position Control (PID) ────────────────────────────────────
    positionXYProportionalGain: 1.1    # Horizontal position stiffness
    positionXYDifferentialGain: 0.0    # Horizontal position damping
    positionXYIntegralGain: 0.0        # Horizontal steady-state correction
    positionXYLimit: 5.0               # Max position correction output

    positionZProportionalGain: 1.0     # Altitude stiffness
    positionZDifferentialGain: 0.2     # Altitude damping
    positionZIntegralGain: 0.0         # Altitude steady-state correction
    positionZLimit: -1                 # -1 = unlimited

    # ── Physical Limits ───────────────────────────────────────────
    maxForce: 30.0                     # Max motor thrust (N)
    motionSmallNoise: 0.0              # Random noise magnitude
    motionDriftNoise: 0.0              # Drift noise magnitude
    motionDriftNoiseTime: 50           # Drift noise update interval
```

### Tuning Tips

| Symptom | Likely Cause | Fix |
|---|---|---|
| Drone overshoots target | Kp too high or Kd too low | Reduce `positionXYProportionalGain` or increase differential gain |
| Drone is slow to reach target | Kp too low | Increase `positionXYProportionalGain` |
| Drone oscillates at target | Kd too low | Increase differential gain at the oscillating loop |
| Drone drifts during hover | No integral, wind-up | Add small `positionXYIntegralGain` (e.g. 0.05) |
| Drone tilts too aggressively | `rollpitchLimit` too high | Reduce `rollpitchLimit` |
| Yaw spins out of control | `yawProportionalGain` too high | Reduce `yawProportionalGain` |

---

## Troubleshooting

**`gz sim` or Gazebo Harmonic not found**
```bash
# Install Gazebo Harmonic via ROS 2 Jazzy vendor packages
sudo apt install ros-jazzy-ros-gz
source /opt/ros/jazzy/setup.bash
```

**Drone doesn't spawn / Gazebo crashes**
```bash
# Kill lingering Gazebo Harmonic processes
pkill -f "gz sim" && pkill -f "gzserver"
# If you see GPU errors, force software rendering
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
```

**`QT_QPA_PLATFORM` / display errors on Gazebo Harmonic**
```bash
export QT_QPA_PLATFORM=xcb
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
```

**AeroPin node: "No module named drone_delivery_system"**
```bash
source ~/ros2_ws/install/setup.bash
# If still failing, rebuild the package:
cd ~/ros2_ws
colcon build --packages-select drone_delivery_system
source install/setup.bash
```

**Drone not responding to mission controller**
```bash
# Check flight state (should not be 0/landed while trying to navigate)
ros2 topic echo /drone/state
# Check control mode
ros2 topic echo /drone/cmd_mode
# Manually enable position control if needed
ros2 topic pub /drone/posctrl std_msgs/msg/Bool "data: true" --once
```

**`xterm` not found**
```bash
sudo apt install xterm
```

**`rosdep` fails for Jazzy packages**
```bash
rosdep update
rosdep install -r -y --from-paths src --ignore-src --rosdistro jazzy
```

---

## License

This project is licensed under the **GPL-3.0 License** — see [LICENSE](LICENSE) for details.

The `sjtu_drone` simulation core was originally developed by Shanghai Jiao Tong University, ported to ROS 2 by [NovoG93](https://github.com/NovoG93/sjtu_drone), and adapted here for **ROS 2 Jazzy + Gazebo Harmonic**.

---

*Built with ROS 2 Jazzy Jalisco · Gazebo Harmonic · Ubuntu 24.04 · Python 3.12 · AeroPin Mission Architecture*
