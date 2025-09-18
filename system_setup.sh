#!/bin/bash
# System setup script for orin got tired of running everything every time it broke
# Sets up CAN interface, installs dependencies, and configures orin depens

set -e  # Exit on any error

echo "🚀 System Setup"
echo "=============================="

# Output colors for fun
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' 

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root
check_sudo() {
    if [[ $EUID -ne 0 ]]; then
        print_error "This script needs sudo privileges"
        echo "Please run with: sudo $0"
        exit 1
    fi
}

# Install system depens
install_system_deps() {
    print_status "Installing system depens..."
    
    apt-get update
    apt-get install -y \
        python3-pip \
        python3-venv \
        can-utils \
        git \
        build-essential \
        cmake \
        pkg-config \
        libjpeg-dev \
        libtiff5-dev \
        libpng-dev \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libv4l-dev \
        libxvidcore-dev \
        libx264-dev \
        libgtk-3-dev \
        libatlas-base-dev \
        gfortran \
        python3-dev
        
    print_status "System depends installed ✓"
}

# Setup CAN interface
setup_can_interface() {
    print_status "Setting up CAN interface..."
    
    # Check if CAN interface already exists
    if ip link show can0 &> /dev/null; then
        print_status "CAN interface can0 already exists"
    else
        print_warning "CAN interface can0 not found"
        print_warning "Please ensure CAN hardware is connected and drivers are loaded"
    fi
    
    # Configure CAN interface
    print_status "Configuring CAN interface..."
    ip link set can0 down 2>/dev/null || true
    # If you get the following two messages you broke the fucking orin. Resolder pins idot
    ip link set can0 type can bitrate 1000000 2>/dev/null || print_warning "Could not set CAN bitrate - check hardware"
    ip link set can0 up 2>/dev/null || print_warning "Could not bring up CAN interface - check hardware"
    
    # Create systemd service for CAN interface
    print_status "Creating CAN systemd service..."
    cat > /etc/systemd/system/can-interface.service << EOF
[Unit]
Description=CAN Interface Setup
After=network.target

[Service]
Type=oneshot
ExecStart=/sbin/ip link set can0 down
ExecStart=/sbin/ip link set can0 type can bitrate 1000000
ExecStart=/sbin/ip link set can0 up
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
    
    systemctl daemon-reload
    systemctl enable can-interface.service
    
    print_status "CAN interface configured ✓"
}

# Create Python virtual environment
setup_python_env() {
    print_status "Setting up Python environment..."
    
    # Switch to regular user for venv creation
    SUDO_USER_HOME=$(getent passwd $SUDO_USER | cut -d: -f6)
    cd "$SUDO_USER_HOME"
    
    # Create virtual environment as the sudo user
    sudo -u $SUDO_USER python3 -m venv mining_rover_env
    
    # Activate and install requirements
    print_status "Installing Python dependencies..."
    sudo -u $SUDO_USER bash -c "
        source mining_rover_env/bin/activate
        pip install --upgrade pip
        pip install -r requirements.txt
    "
    
    print_status "Python environment configured ✓"
}

# Setup RealSense camera
setup_realsense() {
    print_status "Setting up Intel RealSense camera..."
    
    # Add Intel RealSense repository
    apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || true
    add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u || true
    
    # Install RealSense libraries
    apt-get update
    apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev
    
    print_status "RealSense camera support installed ✓"
}

# Setup GPIO permissions
setup_gpio_permissions() {
    print_status "Setting up GPIO permissions..."
    
    # Add user to gpio group
    usermod -a -G gpio $SUDO_USER || print_warning "Could not add user to gpio group"
    
    # Create udev rules for GPIO access
    cat > /etc/udev/rules.d/99-gpio.rules << EOF
SUBSYSTEM=="gpio*", PROGRAM="/bin/sh -c 'chown -R root:gpio /sys/class/gpio && chmod -R 770 /sys/class/gpio; chown -R root:gpio /sys/devices/virtual/gpio && chmod -R 770 /sys/devices/virtual/gpio'"
EOF
    
    # Reload udev rules
    udevadm control --reload-rules
    
    print_status "GPIO permissions configured ✓"
}

# Create project structure
create_project_structure() {
    print_status "Creating project structure..."
    
    SUDO_USER_HOME=$(getent passwd $SUDO_USER | cut -d: -f6)
    PROJECT_DIR="$SUDO_USER_HOME/mining_rover"
    
    # Create directories
    sudo -u $SUDO_USER mkdir -p "$PROJECT_DIR"/{src/{hardware,ros_nodes,teleoperation,autonomous,utils},launch,config,logs,tests}
    
    # Create activation script
    sudo -u $SUDO_USER cat > "$PROJECT_DIR/activate_env.sh" << EOF
#!/bin/bash
# Activation script for mining rover environment

echo "🤖 Activating Mining Rover Environment"
source ~/mining_rover_env/bin/activate

# Add src to Python path
export PYTHONPATH=\$PYTHONPATH:\$(pwd)/src

# Set ROS domain ID (if using ROS2)
export ROS_DOMAIN_ID=42

echo "Environment activated! Ready to run rover code."
echo "Use 'python test_motors.py' to test motor controllers"
EOF
    
    chmod +x "$PROJECT_DIR/activate_env.sh"
    
    print_status "Project structure created ✓"
}

# Test CAN interface
test_can_interface() {
    print_status "Testing CAN interface..."
    
    if ip link show can0 up | grep -q "UP"; then
        print_status "CAN interface is UP ✓"
        
        # Test CAN communication (send a test frame)
        if command -v cansend &> /dev/null; then
            timeout 2s candump can0 &
            sleep 0.5
            # Broke it again idot
            cansend can0 123#DEADBEEF 2>/dev/null && print_status "CAN communication test passed " || print_warning "CAN communication test failed - check wiring"
            pkill candump 2>/dev/null || true
        fi
    else
        print_warning "CAN interface is not UP - check hardware connection"
    fi
}

# Main setup function
main() {
    print_status "Starting system setup..."
    
    check_sudo
    install_system_deps
    setup_can_interface
    setup_python_env
    setup_realsense
    setup_gpio_permissions
    create_project_structure
    test_can_interface
    
    echo ""
    print_status "Setup complete!"
    echo ""
    print_status "Next steps:"
    echo "  1. cd ~/mining_rover"
    echo "  2. ./activate_env.sh"
    echo "  3. python test_motors.py"
    echo ""
    print_status "Hardware checklist:"
    echo "  □ Connect REV NEO motors to CAN bus"
    echo "  □ Connect Intel RealSense D435i camera"
    echo "  □ Connect stepper motor to GPIO pins"
    echo "  □ Connect Firgelli actuators to PWM pins"
    echo "  □ Power up all systems"
    echo ""
    print_warning "Reboot recommended"
}

# Run main function
main "$@"
