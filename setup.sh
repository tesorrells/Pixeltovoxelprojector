#!/bin/bash
#
# Setup script for RTSP Stream Processor
# This script installs all necessary dependencies on a Linux system
#

set -e  # Exit on error

echo "Setting up RTSP Stream Processor dependencies..."
echo "================================================"

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (use sudo)"
  exit 1
fi

# Detect package manager
if command -v apt-get &> /dev/null; then
  # Debian/Ubuntu
  PACKAGE_MANAGER="apt-get"
  
  # Required packages
  echo "Installing system dependencies..."
  apt-get update
  apt-get install -y build-essential cmake git \
    libopencv-dev libopencv-videoio-dev \
    libopengl-dev freeglut3-dev \
    python3-pip python3-dev

elif command -v dnf &> /dev/null; then
  # Fedora/RHEL/CentOS
  PACKAGE_MANAGER="dnf"
  
  # Required packages
  echo "Installing system dependencies..."
  dnf update -y
  dnf install -y gcc-c++ cmake git \
    opencv opencv-devel \
    mesa-libGL-devel freeglut-devel \
    python3-pip python3-devel

elif command -v pacman &> /dev/null; then
  # Arch Linux
  PACKAGE_MANAGER="pacman"
  
  # Required packages
  echo "Installing system dependencies..."
  pacman -Syu --noconfirm
  pacman -S --noconfirm base-devel cmake git \
    opencv freeglut \
    python python-pip

else
  echo "Unsupported package manager. Please install the following packages manually:"
  echo "- C++ compiler (gcc/g++)"
  echo "- CMake"
  echo "- Git"
  echo "- OpenCV with videoio support"
  echo "- OpenGL and GLUT development libraries"
  echo "- Python3 and pip"
  exit 1
fi

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install numpy PyOpenGL PyOpenGL_accelerate

# Create build directory and build the project
echo "Building the RTSP Stream Processor..."
mkdir -p build
cd build
cmake ..
make -j$(nproc)

echo ""
echo "Setup completed successfully!"
echo ""
echo "To use the RTSP Stream Processor:"
echo "1. Configure your cameras in camera_config.json"
echo "2. Run the processor: ./build/rtsp_processor camera_config.json output_dir"
echo "3. Visualize the voxel grid: python3 visualize_voxel_grid.py output_dir"
echo ""
echo "For more information, see the README.md file." 