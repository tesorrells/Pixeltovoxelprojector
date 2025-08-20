#!/bin/bash
# OAK-D Pro Test Runner
# Starts the stereo server and voxel reconstruction, then visualizes results

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

echo "🚀 OAK-D Pro Voxel Reconstruction Test"
echo "======================================"

# Check if build exists
if [ ! -f "build/rtsp_processor" ]; then
    echo "❌ rtsp_processor not found. Building..."
    cd build && make && cd ..
fi

# Check if virtual environment exists
if [ ! -d "$HOME/.venv/depthai" ]; then
    echo "❌ DepthAI virtual environment not found!"
    echo "   Run: ./scripts/setup_oak_d_pro.sh"
    exit 1
fi

# Function to cleanup background processes
cleanup() {
    echo ""
    echo "🛑 Cleaning up..."
    pkill -f oak_d_stereo_server.py 2>/dev/null || true
    pkill -f rtsp_processor 2>/dev/null || true
    echo "✅ Cleanup complete"
}

# Set trap to cleanup on exit
trap cleanup EXIT

echo "📸 Starting OAK-D Pro stereo server..."
source "$HOME/.venv/depthai/bin/activate"
python3 scripts/oak_d_stereo_server.py --resolution 720p --fps 30 &
SERVER_PID=$!

# Wait for server to start
echo "⏳ Waiting for cameras to initialize..."
sleep 5

# Test if servers are running
if ! curl -s http://localhost:8080/ > /dev/null; then
    echo "❌ Left camera server not responding"
    exit 1
fi

if ! curl -s http://localhost:8081/ > /dev/null; then
    echo "❌ Right camera server not responding"
    exit 1
fi

echo "✅ Both camera servers are running"
echo "📡 Left camera:  http://localhost:8080/stream"
echo "📡 Right camera: http://localhost:8081/stream"
echo ""
echo "🎯 Starting voxel reconstruction..."
echo "   Move objects in front of the cameras to see them reconstructed!"
echo "   Press 's' to save voxel grid, 'q' to quit"
echo ""

# Start the voxel reconstruction
./build/rtsp_processor config/oak_d_stereo_config.json

echo ""
echo "🔍 Looking for saved voxel grids..."
LATEST_VOXEL=$(ls -t voxel_grid_*.bin 2>/dev/null | head -1)

if [ -n "$LATEST_VOXEL" ]; then
    echo "📊 Found voxel grid: $LATEST_VOXEL"
    echo "🎨 Launching visualization..."
    
    # Export visualizations to image files
    echo "🎨 Exporting voxel visualizations to images..."
    python3 scripts/export_voxel_images.py "$LATEST_VOXEL" 95
    
    echo ""
    echo "📁 Visualization files created:"
    ls -la "${LATEST_VOXEL%.*}"_*.png "${LATEST_VOXEL%.*}"_*.txt 2>/dev/null || true
    echo ""
    echo "💡 Open the PNG files in your image viewer to see the 3D reconstruction!"
    echo "   - *_perspective.png: 3D overview"
    echo "   - *_top.png: Top-down view"
    echo "   - *_side.png: Side view"
    echo "   - *_slices.png: 2D cross-sections"
else
    echo "ℹ️  No voxel grid saved (you may have quit without pressing 's')"
fi

echo ""
echo "✅ Test complete!"
