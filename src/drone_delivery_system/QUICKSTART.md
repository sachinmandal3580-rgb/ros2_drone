# Quick Start Guide - Drone Delivery System

## 1-Minute Setup

```bash
# Step 1: Copy package to your workspace
cd ~/ros2_ws/src  # or wherever your sjtu_drone is
cp -r /path/to/drone_delivery_system .

# Step 2: Install YOLOv8
pip install ultralytics opencv-python

# Step 3: Install ROS2 dependencies
sudo apt install -y ros-jazzy-vision-msgs ros-jazzy-cv-bridge ros-jazzy-gazebo-msgs

# Step 4: Build
cd ~/ros2_ws
colcon build --packages-select drone_delivery_system
source install/setup.bash
```

## Run Complete Mission (3 Terminals)

### Terminal 1: Drone Simulation
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
```

### Terminal 2: Delivery System
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch drone_delivery_system delivery_system.launch.py
```

### Terminal 3: Start Mission
```bash
cd ~/ros2_ws
source install/setup.bash

# Wait for drone to be ready (5 seconds)
sleep 5

# Start delivery mission
ros2 service call /delivery/start std_srvs/srv/Empty
```

## What Happens?

1. ✅ Drone takes off
2. ✅ Payload (red cube) attaches below drone
3. ✅ Drone searches for person (scanning pattern)
4. ✅ When person detected → approaches and centers
5. ✅ Hovers above person for 2 seconds
6. ✅ Drops payload (red cube falls)
7. ✅ Returns and lands

## Testing Without a Person Model

If you don't have a person in your Gazebo world:

### Option 1: Manual Control (Test Individual Components)

```bash
# Test 1: Attach payload
ros2 service call /payload/attach std_srvs/srv/Empty

# Test 2: Takeoff
ros2 topic pub /simple_drone/takeoff std_msgs/msg/Empty {} --once

# Test 3: Move drone
ros2 topic pub /simple_drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"

# Test 4: Drop payload
ros2 service call /payload/detach std_srvs/srv/Empty

# Test 5: Land
ros2 topic pub /simple_drone/land std_msgs/msg/Empty {} --once
```

### Option 2: Use Yourself as Target!

1. Print a photo of a person on paper
2. Hold it in front of your camera
3. Point the Gazebo camera view at it
4. The drone will detect it as a person!

### Option 3: Add Person Model to World

Edit `~/ros2_ws/src/sjtu_drone/sjtu_drone_description/worlds/home.sdf`:

```xml
<model name="person1">
  <pose>5 0 0 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.3</radius>
          <length>1.7</length>
        </cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.3</radius>
          <length>1.7</length>
        </cylinder>
      </geometry>
      <material>
        <ambient>0.8 0.6 0.4 1</ambient>
        <diffuse>0.8 0.6 0.4 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

## Monitor Everything (Optional Terminal 4)

```bash
# Watch detection status
ros2 topic echo /detections

# Watch delivery state
ros2 topic echo /delivery/state

# Watch payload status
ros2 topic echo /payload/attached

# View annotated camera feed with detections
ros2 run rqt_image_view rqt_image_view /detections/image
```

## Troubleshooting

### "No module named 'ultralytics'"
```bash
pip install ultralytics
```

### "Camera topic not found"
```bash
# Check camera topics
ros2 topic list | grep image

# Make sure drone simulation is running first!
```

### "Payload not appearing"
```bash
# Check Gazebo services
ros2 service list | grep gazebo

# Manually attach for testing
ros2 service call /payload/attach std_srvs/srv/Empty
```

### Drone not moving
```bash
# Check namespace
ros2 topic list | grep simple_drone

# Test manually
ros2 topic pub /simple_drone/takeoff std_msgs/msg/Empty {} --once
```

## Next Steps

Once working, customize:
1. Edit `config/delivery_params.yaml` for different speeds/altitudes
2. Modify search pattern in `delivery_controller.py`
3. Add GPS waypoint navigation
4. Implement multiple delivery points

## Complete Example Mission Log

```
[delivery_controller]: Starting delivery mission!
[payload_manager]: Payload attached successfully
[delivery_controller]: State: searching
[yolo_detector]: Detected 1 person(s)
[delivery_controller]: Person detected! Approaching...
[delivery_controller]: State: approaching
[delivery_controller]: Person centered! Starting hover...
[delivery_controller]: State: hovering
[delivery_controller]: Hover complete. Dropping payload...
[delivery_controller]: State: dropping
[payload_manager]: Payload dropped successfully
[delivery_controller]: Payload dropped! Returning to base...
[delivery_controller]: State: returning
[delivery_controller]: Mission complete! Landing...
[delivery_controller]: State: landed
```

---

**You're ready to fly! 🚁📦**
