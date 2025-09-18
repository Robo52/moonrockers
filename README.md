# moonrockers
Repo for all of the Moonrockers team programming work

Moonrockers Rover Control System
An autonomous rover with teleoperation capabilities, built for Jetson Orin Nano. Features a ROS2 hybrid architecture, real time computer vision, autonomous digging operations, and web-based remote control.

# Features

Autonomous Mining: Complete mining cycle automation with state machine coordination
Teleoperation: WiFi-based remote control with responsive web interface
Computer Vision: RealSense D435i with ArUco localization and obstacle detection
Navigation: Multi-algorithm path planning (A*, RRT, obstacle avoidance)
Digging Operations: Automated bucket control for digging, transport, and dumping
ROS2 Integration: Professional robotics architecture with node-based design
Real-time Monitoring: Live status updates and comprehensive logging

System Architecture
ROS2 Hybrid Architecture
┌─────────────────────────────────────────────────────────────┐
│                       WEB INTERFACE                         │
│  ┌─────────────┐ ┌─────────────────────────────────────────┐│
│  │   Gamepad   │ │       Touch Controls & Monitoring       ││
│  │   Support   │ │        (Joystick/Sliders/Status)        ││
│  └─────────────┘ └─────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
                        WebSocket/HTTP
┌─────────────────────────────────────────────────────────────┐
│                TELEOPERATION SERVER (FastAPI)               │
└─────────────────────────────────────────────────────────────┘
                         ROS2 Topics
┌─────────────────────────────────────────────────────────────┐
│                   ROS2 COORDINATION LAYER                   │
│ ┌─────────────┐ ┌──────────────┐ ┌─────────────────────────┐│
│ │   Vision    │ │  Navigation  │ │    Autonomous State     ││ 
│ │    Node     │ │     Node     │ │         Machine         ││
│ └─────────────┘ └──────────────┘ └─────────────────────────┘│
│ ┌─────────────┐ ┌──────────────┐ ┌─────────────────────────┐│
│ │ Localization│ │    Digging   │ │      Path Planner       ││
│ │     Node    │ │   Behaviors  │ │    (A*, RRT, Avoid)     ││
│ └─────────────┘ └──────────────┘ └─────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
                  Direct Hardware Interface
┌─────────────────────────────────────────────────────────────┐
│               HARDWARE BRIDGE & CONTROLLERS                 │
└─────────────────────────────────────────────────────────────┘
          │            │            │            │
┌─────────────┐ ┌──────────────┐ ┌──────────────┐ ┌─────────────┐
│    MOTORS   │ │    CAMERA    │ │    BUCKET    │ │    VISION   │
│(CAN Control)│ │  (Stepper +  │ │  (Actuators) │ │ (RealSense) │
│             │ │  RealSense)  │ │              │ │             │
└─────────────┘ └──────────────┘ └──────────────┘ └─────────────┘
Hardware 

Compute: Jetson Orin Nano
Motors: 4x REV NEO Brushless Motors (CAN communication)
Camera: Intel RealSense D435i (USB 3.0)
Actuators: 2x Firgelli Linear Actuators (PWM controlled)
Camera Mount: NEMA 17 Stepper Motor for pan
Communication: Soldered on CAN interface on Jetson
Voltage draw: 24V for motors, 12V for actuators, 5V for electronics

# Quick Start
1. System Setup
Run automated setup (sudo)
sudo bash setup_system.sh

Reboot to ensure all drivers are loaded
sudo reboot

2. Hardware Connections
CAN Bus Wiring:

Front Left Motor: CAN ID 1
Front Right Motor: CAN ID 2
Rear Left Motor: CAN ID 3
Rear Right Motor: CAN ID 4
Important: Use 120Ω termination resistors at both ends of CAN bus

GPIO Connections (Jetson Orin Nano):

Bucket Lift Actuator → Pin 32 (PWM)
Bucket Tilt Actuator → Pin 33 (PWM)
Stepper Motor → Pins 18 (Step), 16 (Direction), 22 (Enable)

USB Connections:

Intel RealSense D435i → USB 3.0 port (blue connector)

3. Launch the Rover
Option A: Complete ROS2 System
cd ~/moonrockers

Full autonomous system
ros2 launch moonrockers rover_system.launch.py enable_autonomous:=true

Teleoperation only
ros2 launch moonrockers teleoperation.launch.py port:=8000

Custom configuration
ros2 launch moonrockers rover_system.launch.py \
    enable_vision:=true \
    enable_navigation:=true \
    enable_autonomous:=false \
    log_level:=INFO
    
Option B: Interactive Launcher
Interactive menu system
./launch_rover.sh

Direct launch modes
./launch_rover.sh autonomous    # Full autonomous run
./launch_rover.sh teleoperation # Web control only
./launch_rover.sh test         # Hardware validation

# Project Structure
moonrockers/
├── src/
│   ├── hardware/                    # Hardware Controllers
│   │   ├── can_motor_controller.py  # REV NEO motor control via CAN
│   │   ├── stepper_controller.py    # NEMA 17 camera positioning  
│   │   ├── actuator_controller.py   # Firgelli linear actuators
│   │   ├── vision_system.py         # RealSense + ArUco detection
│   │   ├── hardware_bridge.py       # Unified hardware interface
│   │   └── config.py                # Hardware configuration
│   ├── ros2_env/                    # ROS2 Components
|   |   ├── ros2_env/
|   │   │   ├── vision_node.py       # ROS2 vision processing node
|   │   │   ├── navigation_node.py   # Path planning & execution
|   │   │   ├── localization_node.py   # ArUco-based pose estimation
|   │   │   └── main_controller.py   # ROS2 integration controller
│   ├── autonomous/                  # Autonomous Behaviors
│   │   ├── state_machine.py         # Main autonomous state machine
│   │   ├── digging_behaviors.py     # Digging operations
│   │   └── path_planner.py          # A*, RRT, obstacle avoidance
│   ├── teleoperation/               # Web-Based Control
│   │   ├── teleoperation_server.py  # FastAPI WebSocket server
│   │   ├── templates/index.html     # Web control interface
│   │   └── static/                  # CSS/JS assets
│   ├── main_controller.py           # Hybrid system coordinator
│   └── utils/                       # Utilities
│       ├── transforms.py            # Coordinate transformations
│       └── logging_setup.py         # Centralized logging system
├── launch/                          # ROS2 Launch Files
│   ├── rover_system.launch.py       # Complete system launch
│   └── teleoperation.launch.py      # Teleoperation-only launch
├── config/                          # Configuration Files
│   ├── navigation_config.yaml       # Navigation parameters
│   └── aruco_config.yaml            # Vision & marker configuration
├── launch_rover.sh                  # Main launch script
├── setup_system.sh                  # System setup automation
├── requirements.txt                 # Python dependencies
├── hardware_validation.py           # Hardware testing suite
├── logs/                           # System logs directory
└── README.md
# Web Interface Features
Access the teleoperation interface at: http://[jetson_ip]:8000

Virtual Joystick: Touch/mouse control for movement
Camera Control: Pan camera and adjust angles with presets
Bucket Control: Lift/tilt controls with position presets
Gamepad Support: Dualshock 4 controller compatibility
Keyboard Control: WASD movement, spacebar to stop, E for e-stop
Live Status: Real time rover telemetry and sensor data
Vision Feedback: ArUco markers and obstacle detection display
E-Stop: Instant safety controls (red button + hotkeys)

# Autonomous Operations
The autonomous mode performs complete digging cycles:
Mission Phases:

Localization: Rotate camera to find ArUco markers for precise positioning
Navigation: Path planning with obstacle avoidance to excavation area
Digging Operations (repeated 3 times by default):

Position bucket for digging
Push forward to collect material
Scoop and raise bucket for transport
Navigate to berm area
Dump collected material with shaking motion

Return Home: Navigate back to starting position

# Advanced Capabilities:

Multi-Algorithm Planning: A*, RRT, straight-line, and dynamic obstacle avoidance
Sensor Fusion: Combines ArUco tag with wheel odometry for precise localization
Adaptive Behavior: Adjusts digging depth and bucket angle based on terrain
Safety Monitoring: Continuous obstacle detection and e-stop
Mission Recovery: Automatic retry and error recovery behaviors

# Configs
All system parameters are centralized in configuration files:
Hardware Configuration (src/hardware/config.py)
pythonMOTOR_CONFIG = {
    'front_left': 1,        # CAN ID
    'front_right': 2,       # CAN ID  
    'wheel_diameter': 0.2,  # meters
    'max_velocity': 1.0     # m/s
}

CAMERA_CONFIG = {
    'stepper_steps_per_rev': 200,
    'camera_rotation_speed': 30,  # degrees/second
    'camera_max_angle': 180       # degrees from center
}
Navigation Configuration (config/navigation_config.yaml)
yamlmoonrockers_navigation:
  ros__parameters:
    max_linear_velocity: 0.5   # m/s
    max_angular_velocity: 1.0  # rad/s
    goal_tolerance: 0.15       # meters
    planner_type: "astar"      # astar, rrt, straight_line
    
ArUco Tag Setup (config/aruco_config.yaml)

# Testing & Development
Hardware Validation (test suite for hardware)
python3 hardware_validation.py

# Test specific components
python3 src/hardware/can_motor_controller.py    # Motors
python3 src/hardware/vision_system.py           # Camera
python3 src/hardware/hardware_bridge.py         # Integrated system

# System Monitoring
View system status
./launch_rover.sh status

# Monitor ROS2 topics
ros2 topic list
ros2 topic echo /moonrockers_vision/aruco_markers
ros2 topic echo /navigation/planned_path

# Monitor CAN traffic
candump can0

# View logs
tail -f logs/moonrockers_system.log
tail -f logs/autonomous.log

# Development Tools
ROS2 development
ros2 run moonrockers vision_node           # Run individual nodes
ros2 launch moonrockers teleoperation.launch.py  # Teleoperation only

# Performance analysis  
ros2 topic hz /vision/aruco_markers        # Check detection frequency
ros2 topic bw /cmd_vel                     # Monitor command bandwidth

# Troubleshooting CAN Interface Issues
Check CAN interface status  
ip link show can0

# Manually configure CAN interface
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000  
sudo ip link set can0 up

# Test CAN communication
candump can0 &
cansend can0 123#DEADBEEF
Motor Issues

Verify CAN wiring and termination resistors (120Ω at both ends)
Check motor CAN IDs match configuration (1-4)
Monitor for fault codes: ros2 topic echo /hardware/motor_status
Ensure adequate power supply
Check motor temperature limits (<85°C)

Vision Issues
Check RealSense camera detection
lsusb | grep Intel

# Test camera functionality
realsense-viewer

# Monitor vision topics
ros2 topic hz /vision/aruco_markers     # Detection frequency
ros2 topic echo /vision/obstacles       # Obstacle detection

# ROS2 Issues
Check ROS2 node status
ros2 node list
ros2 node info /moonrockers_vision

# Debug topic connections
ros2 topic info /cmd_vel --verbose

# Check parameter configuration
ros2 param list /moonrockers_navigation
ros2 param get /moonrockers_navigation max_linear_velocity

# Network Issues
bash# Find rover IP address
hostname -I

# Test teleoperation connection
curl http://[rover-ip]:8000/api/status

# Check firewall settings
sudo ufw status
sudo ufw allow 8000

Development Guide
Adding New Hardware

Create controller class in src/hardware/
Add to HardwareBridge initialization
Update configuration in config.py
Add teleoperation controls to web interface
Create ROS2 topics/nodes if needed

Extending Autonomous Behaviors

Modify mission states in state_machine.py
Add new behaviors to digging_behaviors.py
Configure parameters in config/ YAML files
Test with hardware validation suite

Custom Path Planning

Add algorithm to path_planner.py
Configure in navigation_config.yaml
Integrate with navigation node

Web Interface Customization

Edit templates/index.html for UI changes
Modify teleoperation_server.py for new API endpoints
Add WebSocket message types for real-time data
Update CSS/JS assets in static/ directory

System Requirements
Minimum:

Jetson Orin Nano 
128GB+ NVMe SSD
WiFi connectivity for teleoperation

Software Dependencies:

Ubuntu 22.04 with JetPack 6
ROS2 Humble Hawksbill
Python 3.8+
OpenCV 4.5+ with contrib modules
PyRealSense2 SDK
FastAPI for web interface

Performance Specifications:

Vision processing: 10 FPS ArUco detection
Navigation planning: 10 Hz path updates
Motor control: 50 Hz CAN communication
Teleoperation: <100ms latency over WiFi

## Contributing

When adding new things:

1. Follow the existing code structure and naming conventions (PLEASE NAME THINGS THINGS THAT MAKE SENSE AND NOT the_brain WHEN IT IS JUST MOTOR CONTROL)
2. Add comprehensive error handling and logging (PLEASE FOR THE LOVE OF GOD ADD LOGS)
3. Include unit tests for new functionality (Not ass important, but still kinda nice)  
4. Update configuration files as needed
5. Document any new dependencies in requirements.txt (PLEASE PLEASE PLEASE PLEASE PLEASE)
