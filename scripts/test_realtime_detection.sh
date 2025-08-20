#!/bin/bash

# Real-time Drone Detection Test Script
# This script orchestrates the complete real-time detection system

set -e

echo "🚁 Real-time Drone Detection System Test"
echo "======================================="

# Configuration
ZMQ_PORT=${ZMQ_PORT:-5556}
CONFIG_FILE="config/oak_d_stereo_config.json"
BUILD_DIR="build"

# Check if build exists
if [ ! -f "$BUILD_DIR/rtsp_processor" ]; then
    echo "❌ Build not found. Building project..."
    cd build && make && cd ..
fi

# Function to cleanup background processes
cleanup() {
    echo ""
    echo "🧹 Cleaning up background processes..."
    
    # Kill OAK-D server
    pkill -f "oak_d_stereo_server.py" 2>/dev/null || true
    
    # Kill monitor
    pkill -f "console_drone_monitor.py" 2>/dev/null || true
    
    # Kill processor
    pkill -f "rtsp_processor" 2>/dev/null || true
    
    echo "✅ Cleanup complete"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

echo ""
echo "🔧 Starting system components..."

# Step 1: Start OAK-D stereo server
echo "📹 Starting OAK-D Pro stereo server..."
python3 scripts/oak_d_stereo_server.py &
OAK_SERVER_PID=$!

# Wait for server to initialize
echo "⏳ Waiting for camera server to initialize..."
sleep 3

# Check if server is running
if ! kill -0 $OAK_SERVER_PID 2>/dev/null; then
    echo "❌ Failed to start OAK-D server"
    exit 1
fi

echo "✅ OAK-D server running (PID: $OAK_SERVER_PID)"

# Step 2: Start console-based monitor  
echo "🖥️  Starting console drone monitor..."
source ~/.venv/depthai/bin/activate
python3 scripts/console_drone_monitor.py $ZMQ_PORT &
MONITOR_PID=$!

echo "✅ Console monitor started (PID: $MONITOR_PID)"

# Wait a moment for viewer to connect
sleep 2

# Step 3: Start the main processor with ZMQ enabled
echo "🔄 Starting main processor with live streaming..."
echo "   Config: $CONFIG_FILE"
echo "   ZMQ Port: $ZMQ_PORT"
echo ""

export ZMQ_PORT=$ZMQ_PORT

# Run the processor (this will block until stopped)
echo "🚀 Real-time drone detection system is now running!"
echo ""
echo "📊 You should see:"
echo "   - Live console monitoring with real-time stats"
echo "   - Detection alerts and coordinates"  
echo "   - Frame rate and processing statistics"
echo ""
echo "🎯 To test detection:"
echo "   - Wave your hand in front of the cameras"
echo "   - Move objects in the stereo field of view"
echo "   - Check console for detection alerts"
echo ""
echo "🛑 Press Ctrl+C to stop all components"
echo ""

cd $BUILD_DIR
./rtsp_processor ../$CONFIG_FILE

# This line will only be reached if rtsp_processor exits normally
cleanup
