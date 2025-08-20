#!/bin/bash

# Setup script for OAK-D Pro camera support
# This script installs DepthAI and sets up udev rules for OAK cameras

echo "Setting up OAK-D Pro camera support..."

# Install DepthAI Python library
echo "Installing DepthAI..."

# Check if we're in an externally managed environment
if pip3 install --help | grep -q "break-system-packages"; then
    echo "Detected externally managed Python environment."
    echo "Trying alternative installation methods..."
    
    # Method 1: Try pipx first (recommended for applications)
    if command -v pipx &> /dev/null; then
        echo "Using pipx to install DepthAI..."
        pipx install depthai
    # Method 2: Try apt package if available
    elif command -v apt-get &> /dev/null; then
        echo "Checking for DepthAI apt package..."
        if apt-cache search python3-depthai | grep -q depthai; then
            sudo apt-get install -y python3-depthai
        else
            echo "DepthAI not available via apt. Creating virtual environment..."
            python3 -m venv ~/.venv/depthai
            ~/.venv/depthai/bin/pip install depthai
            echo "DepthAI installed in virtual environment at ~/.venv/depthai/"
            echo "To use: source ~/.venv/depthai/bin/activate"
        fi
    # Method 3: Create virtual environment
    else
        echo "Creating virtual environment for DepthAI..."
        python3 -m venv ~/.venv/depthai
        ~/.venv/depthai/bin/pip install depthai
        echo "DepthAI installed in virtual environment at ~/.venv/depthai/"
        echo "To use: source ~/.venv/depthai/bin/activate"
    fi
else
    # Traditional pip install
    pip3 install depthai
fi

# Install system dependencies
echo "Installing system dependencies..."
if command -v apt-get &> /dev/null; then
    sudo apt-get update
    sudo apt-get install -y libusb-1.0-0-dev
elif command -v yum &> /dev/null; then
    sudo yum install -y libusb1-devel
elif command -v dnf &> /dev/null; then
    sudo dnf install -y libusb1-devel
else
    echo "Warning: Unable to detect package manager. Please install libusb-1.0-dev manually."
fi

# Set up udev rules for OAK cameras
echo "Setting up udev rules for OAK cameras..."
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules

# Reload udev rules
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "Setup complete!"
echo ""
echo "To test your OAK-D Pro camera:"
echo "1. Connect your OAK-D Pro camera via USB"
echo "2. Run: python3 -c \"import depthai as dai; print('Available devices:', len(dai.Device.getAllAvailableDevices()))\""
echo "3. Build and run the project with: ./rtsp_processor config/oak_d_pro_config.json"
echo ""
echo "If you have multiple OAK devices, you can list them with:"
echo "python3 -c \"import depthai as dai; [print(f'Device {i}: {d.getMxId()}') for i, d in enumerate(dai.Device.getAllAvailableDevices())]\""
