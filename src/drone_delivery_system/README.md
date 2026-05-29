# Drone Delivery System with YOLOv8 Person Detection

Autonomous drone delivery system that detects persons using YOLOv8, approaches them, and drops a payload.

## Features

✅ **YOLOv8 Person Detection** - Real-time person detection from drone camera  
✅ **Autonomous Navigation** - Automatic approach and centering on detected person  
✅ **Payload Management** - Attach/detach payload with Gazebo integration  
✅ **State Machine** - Robust delivery mission state management  
✅ **ROS2 Integration** - Full ROS2 Jazzy support with topics and services  

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Delivery Mission Flow                    │
└─────────────────────────────────────────────────────────────┘

1. IDLE → 2. SEARCHING → 3. APPROACHING → 4. HOVERING → 5. DROPPING → 6. RETURNING → 7. LANDED

Components:
┌──────────────────┐      ┌──────────────────┐      ┌──────────────────┐
│  YOLOv8 Detector │──────│ Delivery         │──────│  Payload         │
│  (Camera feed)   │      │ Controller       │      │  Manager         │
└──────────────────┘      └──────────────────┘      └──────────────────┘
        │                          │                          │
        │ /detections              │ /cmd_vel                 │ Gazebo
        │                          │ /takeoff                 │ Services
        ▼                          ▼ /land                    ▼
   Person Found              Drone Control             Attach/Detach
```

## Prerequisites

### 1. System Requirements
- Ubuntu 24.04 Noble
- ROS2 Jazzy
- Gazebo Harmonic (gz-sim8)
- Python 3.12

### 2. Install Dependencies

```bash
# Install Python packages
pip install ultralytics opencv-python

# Install ROS2 packages
sudo apt install -y \
    ros-jazzy-vision-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-gazebo-msgs \
    ros-jazzy-gazebo-ros-pkgs
```

### 3. YOLOv8 Model

The package will automatically download YOLOv8n (nano) model on first run. You can also download manually:

```bash
# Download YOLOv8 nano model (fastest, ~6MB)
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"

# Or use larger models for better accuracy:
# yolov8s.pt (small, ~22MB)
# yolov8m.pt (medium, ~52MB)
# yolov8l.pt (large, ~87MB)
```

## Installation

```bash
# 1. Navigate to your ROS2 workspace
cd ~/ros2_ws/src
# or wherever you cloned sjtu_drone

# 2. Copy the package
cp -r /path/to/drone_delivery_system .

# 3. Install dependencies
cd ~/ros2_ws
rosdep install -r -y --from-paths src --ignore-src --rosdistro jazzy

# 4. Build
colcon build --packages-select drone_delivery_system

# 5. Source
source install/setup.bash
```

## Usage

### Quick Start - Complete Mission

```bash
# Terminal 1: Launch drone simulation
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py

# Terminal 2: Launch delivery system
source ~/ros2_ws/install/setup.bash
ros2 launch drone_delivery_system delivery_system.launch.py

# Terminal 3: Add a person to the scene (or use existing models in Gazebo)
# Then start the mission
ros2 service call /delivery/start std_srvs/srv/Empty
```

### Step-by-Step Testing

#### 1. Test YOLOv8 Detection

```bash
# Launch only the detector
ros2 run drone_delivery_system yolo_detector

# View detections
ros2 topic echo /detections

# View annotated images (if you have image viewer)
ros2 run rqt_image_view rqt_image_view /detections/image
```

#### 2. Test Payload Management

```bash
# Launch payload manager
ros2 run drone_delivery_system payload_manager

# Attach payload
ros2 service call /payload/attach std_srvs/srv/Empty

# Check status
ros2 topic echo /payload/status

# Detach payload
ros2 service call /payload/detach std_srvs/srv/Empty
```

#### 3. Full Delivery Mission

```bash
# Launch everything
ros2 launch drone_delivery_system delivery_system.launch.py

# Start mission
ros2 service call /delivery/start std_srvs/srv/Empty

# Monitor state
ros2 topic echo /delivery/state
```

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/simple_drone/bottom/image_raw` | sensor_msgs/Image | Camera feed for detection |
| `/detections` | vision_msgs/Detection2DArray | Person detections from YOLO |
| `/payload/attached` | std_msgs/Bool | Payload attachment status |
| `/simple_drone/navsat` | sensor_msgs/NavSatFix | Drone GPS position |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/simple_drone/cmd_vel` | geometry_msgs/Twist | Drone velocity commands |
| `/simple_drone/takeoff` | std_msgs/Empty | Takeoff command |
| `/simple_drone/land` | std_msgs/Empty | Land command |
| `/detections` | vision_msgs/Detection2DArray | Person detections |
| `/detections/image` | sensor_msgs/Image | Annotated image with boxes |
| `/delivery/state` | std_msgs/String | Current delivery state |
| `/payload/status` | std_msgs/String | Payload status (attached/detached) |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/delivery/start` | std_srvs/Empty | Start delivery mission |
| `/payload/attach` | std_srvs/Empty | Attach payload to drone |
| `/payload/detach` | std_srvs/Empty | Detach payload (drop) |

## Configuration

Edit `config/delivery_params.yaml`:

```yaml
delivery_controller:
  ros__parameters:
    drone_namespace: "simple_drone"     # Drone topic namespace
    search_altitude: 3.0                # Search altitude (meters)
    drop_altitude: 2.0                  # Drop altitude (meters)
    approach_speed: 0.5                 # Approach velocity (m/s)
    hover_time: 2.0                     # Hover duration before drop (sec)
    detection_confidence_min: 0.6       # Min YOLO confidence (0-1)

yolo_detector:
  ros__parameters:
    camera_topic: "/simple_drone/bottom/image_raw"
    model_path: "yolov8n.pt"           # YOLOv8 model (n/s/m/l/x)
    confidence_threshold: 0.5           # Detection threshold
    target_class: "person"              # Target object class
    publish_annotated: true             # Publish annotated images
```

## State Machine

```
┌──────┐  start_mission   ┌───────────┐  person_found  ┌─────────────┐
│ IDLE ├─────────────────▶│ SEARCHING ├───────────────▶│ APPROACHING │
└──────┘                  └───────────┘                └─────────────┘
                                ▲                              │
                                │ person_lost                  │ centered
                                └──────────────────────────────┘
                                                               │
                                                               ▼
┌────────┐                ┌──────────┐  hover_complete  ┌──────────┐
│ LANDED │◀───────────────│ RETURNING│◀─────────────────│ DROPPING │
└────────┘                └──────────┘                  └──────────┘
                                                               ▲
                                                               │
                                                         ┌──────────┐
                                                         │ HOVERING │
                                                         └──────────┘
```

## Troubleshooting

### Issue: YOLO not detecting persons

**Solution:**
```bash
# 1. Check if camera is publishing
ros2 topic hz /simple_drone/bottom/image_raw

# 2. View camera feed
ros2 run rqt_image_view rqt_image_view /simple_drone/bottom/image_raw

# 3. Lower confidence threshold
# Edit config/delivery_params.yaml:
#   confidence_threshold: 0.3

# 4. Test YOLO manually
python3 -c "from ultralytics import YOLO; model = YOLO('yolov8n.pt'); model.predict(source='0', show=True)"
```

### Issue: Payload not attaching

**Solution:**
```bash
# 1. Check Gazebo services are available
ros2 service list | grep gazebo

# 2. Manually spawn a test object
ros2 run gazebo_ros spawn_entity.py -entity test_box -database cube

# 3. Check payload manager logs
ros2 run drone_delivery_system payload_manager --ros-args --log-level debug
```

### Issue: Drone not responding to commands

**Solution:**
```bash
# 1. Check namespace matches
ros2 topic list | grep cmd_vel
# Should see: /simple_drone/cmd_vel

# 2. Manually test control
ros2 topic pub /simple_drone/takeoff std_msgs/msg/Empty {} --once
ros2 topic pub /simple_drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

# 3. Check delivery controller is running
ros2 node list | grep delivery
```

## Advanced Usage

### Custom Search Pattern

Modify `delivery_controller.py` → `search_for_person()`:

```python
def search_for_person(self):
    # Circular search pattern
    cmd = Twist()
    cmd.linear.x = 0.5
    cmd.angular.z = 0.3
    self.cmd_vel_pub.publish(cmd)
```

### Multiple Person Handling

Modify `approach_person()` to select closest/largest detection:

```python
def approach_person(self):
    # Find largest detection (closest person)
    largest_detection = max(
        self.current_detections,
        key=lambda d: d.bbox.size_x * d.bbox.size_y
    )
    # Use largest_detection for approach
```

### Add GPS Waypoint Navigation

```python
def return_to_base(self):
    # Navigate to GPS coordinates
    target_lat = 37.7749  # San Francisco
    target_lon = -122.4194
    self.navigate_to_gps(target_lat, target_lon)
```

## Testing with Person Models

### Option 1: Spawn Person in Gazebo

```bash
# While simulation is running
gz model --spawn-file=/usr/share/gazebo-11/models/person_standing/model.sdf \
         --model-name=person1 -x 5 -y 0 -z 0
```

### Option 2: Use Yourself!

Print out a person image and hold it in front of the drone's camera!

### Option 3: Use Video Feed

Point the drone camera at a video/screen showing people.

## ROS2 Command Reference

```bash
# Start mission
ros2 service call /delivery/start std_srvs/srv/Empty

# Monitor state
ros2 topic echo /delivery/state

# View detections
ros2 topic echo /detections

# Manual payload control
ros2 service call /payload/attach std_srvs/srv/Empty
ros2 service call /payload/detach std_srvs/srv/Empty

# Check payload status
ros2 topic echo /payload/attached

# Manual drone control
ros2 topic pub /simple_drone/takeoff std_msgs/msg/Empty {} --once
ros2 topic pub /simple_drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
ros2 topic pub /simple_drone/land std_msgs/msg/Empty {} --once
```

## Performance Tips

1. **Use GPU for YOLO**: Install CUDA and PyTorch with CUDA support
2. **Lower resolution**: Modify camera resolution in drone URDF
3. **Use lighter model**: Change to `yolov8n.pt` (fastest)
4. **Reduce detection rate**: Add throttling in detector callback

## Future Enhancements

- [ ] GPS waypoint navigation
- [ ] Multiple drone coordination
- [ ] Obstacle avoidance
- [ ] Return-to-home on low battery
- [ ] Mission waypoint file loading
- [ ] Web-based monitoring dashboard
- [ ] Delivery confirmation (QR code scanning)
- [ ] Multiple payload drops per mission

## Credits

- **SJTU Drone**: NovoG93/sjtu_drone
- **YOLOv8**: Ultralytics
- **ROS2**: Open Robotics
- **Gazebo**: Open Robotics

## License

GPL-3.0

## Support

For issues, create a GitHub issue or contact the maintainer.

---

**Happy Delivering! 🚁📦**
