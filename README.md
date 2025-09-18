# moonrockers
Repo for all of the Moonrockers team programming work

## Features

- **Teleoperation**: WiFi-based remote control with web interface
- **Autonomous Navigation**: ArUco-based localization and obstacle avoidance  
- **Digging Operations**: Automated bucket control
- **Hybrid Architecture**: Combines ROS2 for high level coordination with direct hardware interfaces

## Hardware Requirements

- **Compute**: Jetson Orin Nano
- **Motors**: 4x REV NEO Brushless Motors with CAN communication
- **Camera**: Intel RealSense D435i
- **Actuators**: 2x Firgelli Linear Actuators (PWM controlled)
- **Pan/Tilt**: NEMA 17 Stepper Motor for camera rotation
- **Communication**: Soldered in CAN interface on Jetson

## Quick Start

### 1. System Setup

```bash
cd ~/moonrockers

# Run system setup (requires sudo)
sudo bash setup_system.sh

# Reboot to ensure all drivers are loaded
sudo reboot
```

### 2. Hardware Connections

**CAN Bus Wiring:**
- Connect REV NEO motors to CAN bus with the following IDs:
  - Front Left: CAN ID 1
  - Front Right: CAN ID 2  
  - Rear Left: CAN ID 3
  - Rear Right: CAN ID 4

**PWM Connections:**
- Bucket Lift Actuator → Jetson Pin 32
- Bucket Tilt Actuator → Jetson Pin 33

**Camera:**
- Intel RealSense D435i → USB 3.0 port
- NEMA 17 Stepper → GPIO pins (configured in hardware_config.py)

### 3. Test Motors

```bash
cd ~/moonrockers
./activate_env.sh
python test_motors.py
```

This will run a test suite to verify:
- CAN communication with all motors
- Individual motor control
- Differential drive kinematics
- E stop functionality
- Motor health monitoring

## Project Structure

```
moonrockers/
├── src/
│   ├── hardware/                    # Direct hardware interfaces
│   │   ├── can_motor_controller.py  # REV NEO motor control via CAN
│   │   ├── stepper_controller.py    # Camera pan/tilt control
│   │   ├── actuator_controller.py   # Linear actuator control
│   │   └── hardware_bridge.py       # Hardware abstraction layer
│   ├── ros_nodes/                   # ROS2 components
│   │   ├── main_controller.py       # Central state machine
│   │   ├── vision_node.py           # ArUco detection & obstacle avoidance
│   │   ├── navigation_node.py       # Path planning & execution
│   │   └── localization_node.py     # Pose estimation from ArUco
│   ├── teleoperation/              # Web-based remote control
│   │   ├── web_server.py           # FastAPI server
│   │   └── static/                 # Web interface files
│   ├── autonomous/                 # Autonomous behaviors
│   │   ├── state_machine.py        # Main autonomous logic
│   │   ├── path_planner.py         # Obstacle avoidance planning
│   │   └── mining_behaviors.py     # Mining-specific actions
│   └── utils/
│       ├── transforms.py           # Coordinate transformations
│       ├── config.py               # Hardware configuration
│       └── logging_setup.py        # Logging configuration
├── config/                         # Configuration files
├── launch/                         # ROS2 launch files
├── logs/                          # Log files
└── tests/                         # Test scripts
```

## Configuration

All hardware parameters are in `src/utils/config.py`:

- **Motor Configuration**: CAN IDs, wheel dimensions, control parameters
- **Camera Configuration**: Stepper motor settings, rotation limits
- **Actuator Configuration**: PWM pin assignments, servo limits  
- **Vision Configuration**: ArUco dictionary, camera calibration
- **Navigation Configuration**: Planning parameters, obstacle thresholds

## Development Workflow

### Motor Development
1. Test individual motors: `python test_motors.py`
2. Verify CAN communication: `candump can0`
3. Check motor health: Monitor temperature, current, faults

### Next Components to Implement
1. **Stepper Motor Controller** - Camera turn for ArUco detection
2. **Vision System** - ArUco detection and obstacle avoidance
3. **PWM Actuator Controller** - Linear actuator control for bucket
4. **Basic Teleoperation** - Web interface for manual control
5. **ROS2 Integration** - State machine and navigation nodes

## Troubleshooting

### CAN Interface Issues
```bash
# Check CAN interface status  
ip link show can0

# Bring up CAN interface manually
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000  
sudo ip link set can0 up

# Monitor CAN traffic
candump can0
```

### Motor Issues
- Verify CAN wiring and termination resistors
- Check motor CAN IDs match configuration
- Monitor for fault codes in motor status
- Ensure proper power supply to motors

### GPIO Issues  
- Check user is in `gpio` group: `groups $USER`
- Verify GPIO permissions: `ls -l /sys/class/gpio`
- Test basic GPIO: `echo 32 > /sys/class/gpio/export`

## Contributing

When adding new components:

1. Follow the existing code structure and naming conventions (PLEASE NAME THINGS THINGS THAT MAKE SENSE AND NOT the_brain WHEN IT IS JUST MOTOR CONTROL)
2. Add comprehensive error handling and logging (PLEASE FOR THE LOVE OF GOD ADD LOGS)
3. Include unit tests for new functionality (Not ass important, but still kinda nice)  
4. Update configuration files as needed
5. Document any new dependencies in requirements.txt (PLEASE PLEASE PLEASE PLEASE PLEASE)
