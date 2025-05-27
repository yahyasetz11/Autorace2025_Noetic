#!/bin/bash

# TurtleBot3 Autonomous Racing System - Dependencies Installation Script
# Author: Yahya Setiawan
# Description: Automated installation of all required system and Python dependencies

set -e  # Exit on any error

echo "🚀 TurtleBot3 Autonomous Racing System - Dependencies Installation"
echo "=================================================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running on Ubuntu 20.04
check_ubuntu_version() {
    print_status "Checking Ubuntu version..."
    if ! grep -q "20.04" /etc/lsb-release 2>/dev/null; then
        print_warning "This script is designed for Ubuntu 20.04. Your system may have compatibility issues."
        read -p "Continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    else
        print_success "Ubuntu 20.04 detected"
    fi
}

# Update system packages
update_system() {
    print_status "Updating system packages..."
    sudo apt update
    sudo apt upgrade -y
    print_success "System packages updated"
}

# Install ROS Noetic
install_ros_noetic() {
    print_status "Installing ROS Noetic..."
    
    # Check if ROS is already installed
    if command -v roscore &> /dev/null; then
        print_success "ROS Noetic already installed"
        return
    fi
    
    # Setup ROS Noetic repositories
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    
    # Add ROS keys
    if ! curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -; then
        print_warning "Failed to add ROS key via curl, trying alternative method..."
        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    fi
    
    sudo apt update
    
    # Install ROS Noetic Desktop Full
    sudo apt install -y ros-noetic-desktop-full
    
    # Initialize rosdep
    if ! sudo rosdep init 2>/dev/null; then
        print_warning "rosdep already initialized or failed to initialize"
    fi
    rosdep update
    
    # Setup environment
    if ! grep -q "source /opt/ros/noetic/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    fi
    source /opt/ros/noetic/setup.bash
    
    print_success "ROS Noetic installed successfully"
}

# Install TurtleBot3 packages
install_turtlebot3() {
    print_status "Installing TurtleBot3 packages..."
    sudo apt install -y \
        ros-noetic-turtlebot3 \
        ros-noetic-turtlebot3-msgs \
        ros-noetic-turtlebot3-simulations \
        ros-noetic-turtlebot3-gazebo \
        ros-noetic-turtlebot3-navigation \
        ros-noetic-turtlebot3-slam \
        ros-noetic-turtlebot3-description \
        ros-noetic-turtlebot3-teleop \
        ros-noetic-turtlebot3-example \
        ros-noetic-actionlib \
        ros-noetic-actionlib-msgs \
        ros-noetic-move-base-msgs
    print_success "TurtleBot3 packages installed"
}

# Install TurtleBot3 AutoRace packages (if available)
install_turtlebot3_autorace() {
    print_status "Installing TurtleBot3 AutoRace packages..."
    
    # Try to install autorace packages, but don't fail if they're not available
    sudo apt install -y \
        ros-noetic-turtlebot3-autorace-camera \
        ros-noetic-turtlebot3-autorace-core \
        ros-noetic-turtlebot3-autorace-detect \
        ros-noetic-turtlebot3-autorace-driving \
        ros-noetic-turtlebot3-autorace-msgs || {
        print_warning "Some AutoRace packages not available in standard repositories"
        print_status "You may need to build them from source if required"
    }
    
    print_success "AutoRace packages installation attempted"
}

# Install ROS packages
install_ros_packages() {
    print_status "Installing additional ROS packages..."
    sudo apt install -y \
        ros-noetic-cv-bridge \
        ros-noetic-image-transport \
        ros-noetic-image-transport-plugins \
        ros-noetic-compressed-image-transport \
        ros-noetic-compressed-depth-image-transport \
        ros-noetic-dynamic-reconfigure \
        ros-noetic-joy \
        ros-noetic-teleop-twist-joy \
        ros-noetic-map-server \
        ros-noetic-amcl \
        ros-noetic-move-base \
        ros-noetic-gmapping \
        ros-noetic-slam-gmapping \
        ros-noetic-navigation \
        ros-noetic-dwa-local-planner \
        ros-noetic-robot-localization \
        ros-noetic-tf2-geometry-msgs \
        ros-noetic-tf2-sensor-msgs \
        ros-noetic-behaviortree-cpp-v3 \
        ros-noetic-rqt \
        ros-noetic-rqt-common-plugins \
        ros-noetic-rqt-reconfigure \
        ros-noetic-rqt-image-view \
        ros-noetic-rviz \
        ros-noetic-gazebo-ros \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-gazebo-ros-control \
        ros-noetic-controller-manager \
        ros-noetic-joint-state-controller \
        ros-noetic-effort-controllers \
        ros-noetic-position-controllers
    print_success "ROS packages installed"
}

# Install system dependencies
install_system_dependencies() {
    print_status "Installing system dependencies..."
    sudo apt install -y \
        build-essential \
        cmake \
        git \
        wget \
        curl \
        vim \
        nano \
        python3-pip \
        python3-dev \
        python3-catkin-tools \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        python3-setuptools \
        python3-wheel \
        libopencv-dev \
        libopencv-contrib-dev \
        libyaml-cpp-dev \
        libeigen3-dev \
        libboost-all-dev \
        libboost-system-dev \
        libboost-filesystem-dev \
        libboost-thread-dev \
        qtbase5-dev \
        libqt5svg5-dev \
        libjsoncpp-dev \
        libusb-1.0-0-dev \
        pkg-config \
        v4l-utils \
        cheese \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev
    print_success "System dependencies installed"
}

# Install Python dependencies
install_python_dependencies() {
    print_status "Installing Python dependencies..."
    
    # Upgrade pip first
    python3 -m pip install --upgrade pip
    
    # Install basic dependencies
    python3 -m pip install --user \
        numpy>=1.19.0 \
        opencv-python>=4.5.0 \
        opencv-contrib-python>=4.5.0 \
        PyYAML>=5.4.0 \
        matplotlib>=3.3.0 \
        scipy>=1.6.0 \
        Pillow>=8.0.0 \
        transforms3d>=0.3.1 \
        filterpy>=1.4.0 \
        rospkg>=1.3.0
    
    # Check if requirements.txt exists and install from it
    if [ -f "requirements.txt" ]; then
        print_status "Installing additional dependencies from requirements.txt..."
        python3 -m pip install --user -r requirements.txt
        print_success "Python dependencies installed from requirements.txt"
    else
        print_warning "requirements.txt not found, skipping additional dependencies"
    fi
    
    print_success "Python dependencies installed"
}

# Setup workspace
setup_workspace() {
    print_status "Setting up workspace..."
    
    # Source ROS environment
    source /opt/ros/noetic/setup.bash
    
    # Check if we're in a catkin workspace
    if [ ! -f "CMakeLists.txt" ] && [ ! -d "src" ]; then
        print_error "This doesn't appear to be a catkin workspace root directory"
        print_error "Please run this script from your catkin workspace root (where src/ directory is located)"
        exit 1
    fi
    
    # Build workspace if it hasn't been built yet
    if [ ! -d "devel" ]; then
        print_status "Building workspace for the first time..."
        catkin_make
        print_success "Workspace built successfully"
    else
        print_status "Rebuilding workspace..."
        catkin_make
        print_success "Workspace rebuilt successfully"
    fi
    
    # Source workspace
    if ! grep -q "source $(pwd)/devel/setup.bash" ~/.bashrc; then
        echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
    fi
    
    # Set TurtleBot3 model
    if ! grep -q "export TURTLEBOT3_MODEL=burger" ~/.bashrc; then
        echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    fi
    
    # Set ROS Master URI (for local development)
    if ! grep -q "export ROS_MASTER_URI=http://localhost:11311" ~/.bashrc; then
        echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
    fi
    
    # Set ROS Hostname
    if ! grep -q "export ROS_HOSTNAME=localhost" ~/.bashrc; then
        echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
    fi
    
    print_success "Workspace configured"
}

# Set up udev rules for devices
setup_udev_rules() {
    print_status "Setting up udev rules for camera and sensors..."
    
    # Add user to video group for camera access
    sudo usermod -a -G video $USER
    sudo usermod -a -G dialout $USER
    
    # Create udev rules for consistent device naming
    sudo tee /etc/udev/rules.d/99-turtlebot3-autorace.rules > /dev/null <<EOF
# TurtleBot3 AutoRace udev rules
# Camera devices
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="0c45", ATTRS{idProduct}=="636d", SYMLINK+="turtlebot3_camera"
SUBSYSTEM=="video4linux", KERNEL=="video[0-9]*", ATTRS{product}=="USB Camera", SYMLINK+="usb_camera"

# Serial devices for TurtleBot3
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="turtlebot3"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="turtlebot3_lidar"
EOF
    
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    print_success "Udev rules configured"
}

# Configure git (optional)
configure_git() {
    print_status "Configuring git (optional)..."
    
    if [ -z "$(git config --global user.name)" ]; then
        read -p "Enter your git username (or press Enter to skip): " git_username
        if [ ! -z "$git_username" ]; then
            git config --global user.name "$git_username"
        fi
    fi
    
    if [ -z "$(git config --global user.email)" ]; then
        read -p "Enter your git email (or press Enter to skip): " git_email
        if [ ! -z "$git_email" ]; then
            git config --global user.email "$git_email"
        fi
    fi
    
    print_success "Git configuration completed"
}

# Verify installation
verify_installation() {
    print_status "Verifying installation..."
    
    # Check ROS installation
    if ! command -v roscore &> /dev/null; then
        print_error "ROS installation failed - roscore command not found"
        exit 1
    fi
    
    # Source ROS environment for verification
    source /opt/ros/noetic/setup.bash
    
    # Check key packages
    local packages=("actionlib" "cv_bridge" "image_transport" "dynamic_reconfigure")
    for package in "${packages[@]}"; do
        if ! rospack find "$package" &> /dev/null; then
            print_error "Package $package not found"
            exit 1
        fi
    done
    
    # Check BehaviorTree.CPP
    if ! rospack find behaviortree_cpp_v3 &> /dev/null; then
        print_warning "behaviortree_cpp_v3 package not found, but installation may still work"
    fi
    
    # Check Python packages
    python3 -c "import cv2, numpy, yaml" 2>/dev/null || {
        print_error "Python dependencies verification failed"
        exit 1
    }
    
    # Check workspace
    if [ -f "devel/setup.bash" ]; then
        source devel/setup.bash
        print_success "Workspace verification passed"
    else
        print_warning "Workspace not properly built, but system dependencies are installed"
    fi
    
    print_success "Installation verification completed"
}

# Display post-installation instructions
show_post_install_instructions() {
    echo ""
    echo "=================================================================="
    print_success "Installation completed successfully! 🎉"
    echo "=================================================================="
    echo ""
    echo "📋 Next steps:"
    echo "1. Restart your terminal or run: source ~/.bashrc"
    echo "2. Verify ROS installation: roscore (in a new terminal)"
    echo "3. Connect your TurtleBot3 and camera"
    echo "4. Test camera: rosrun camera camera_laptop"
    echo "5. Launch the complete system: roslaunch launch_file run.launch"
    echo ""
    echo "🔧 Useful commands:"
    echo "• Check ROS environment: printenv | grep ROS"
    echo "• List ROS packages: rospack list | grep turtlebot3"
    echo "• Test camera devices: ls /dev/video*"
    echo "• Monitor ROS topics: rostopic list"
    echo ""
    echo "📁 Important directories:"
    echo "• Workspace: $(pwd)"
    echo "• Config files: $(pwd)/src/config/"
    echo "• Launch files: $(pwd)/src/launch_file/launch/"
    echo ""
    print_warning "⚠️  Please restart your terminal to apply all environment changes!"
    echo ""
    print_status "For troubleshooting, check the README.md file"
}

# Main installation process
main() {
    print_status "Starting TurtleBot3 Autonomous Racing System installation..."
    echo ""
    
    check_ubuntu_version
    update_system
    install_ros_noetic
    install_turtlebot3
    install_turtlebot3_autorace
    install_ros_packages
    install_system_dependencies
    install_python_dependencies
    setup_workspace
    setup_udev_rules
    configure_git
    verify_installation
    show_post_install_instructions
}

# Handle script interruption
trap 'print_error "Installation interrupted by user"; exit 1' INT

# Run main function
main "$@"