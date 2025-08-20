# Drone Detection System Workflows

This document outlines the main workflows for the OAK-D Pro real-time drone detection system.

## 🚀 Real-time Detection Workflow

**Main production workflow for live drone detection:**

```bash
# Run the complete real-time detection system
./scripts/test_realtime_detection.sh
```

This script automatically:

1. Starts the OAK-D Pro stereo camera server
2. Launches the console-based drone monitoring system
3. Runs the main stereo processing engine with ZMQ streaming
4. Provides live alerts and statistics in the terminal

**Components:**

- `scripts/oak_d_stereo_server.py` - OAK-D Pro camera server
- `scripts/console_drone_monitor.py` - Real-time monitoring with alerts
- `build/rtsp_processor` - Main stereo processing engine

## 🔬 Offline Testing Workflow

**For validation and development using captured frames:**

### Step 1: Capture Test Frames

```bash
# Capture 10 seconds of frames at 2 FPS
python3 scripts/oak_d_capture_frames.py --output test_data --duration 10 --fps 2
```

### Step 2: Generate Metadata

```bash
# Create metadata for offline processing
python3 scripts/create_metadata_json.py test_data --output metadata.json
```

### Step 3: Run Offline Analysis

```bash
# Process with the original ray_voxel algorithm
./build/ray_voxel test_data metadata.json
```

### Step 4: Visualize Results

```bash
# View the results
python3 scripts/clean_voxel_view.py result.bin 95.0
```

## 📊 Detection Thresholds

The system uses graduated detection levels:

- **Trace**: Background motion (no alerts)
- **Minor**: 50+ points (🔍 minor alerts)
- **Moderate**: 120+ points OR 2500+ intensity (⚠️ moderate alerts)
- **Major**: 250+ points AND 3000+ intensity (🚨 major alerts)

## 🛠️ Setup and Configuration

### Initial Setup

```bash
# Install OAK-D Pro dependencies
./scripts/setup_oak_d_pro.sh

# Build the system
mkdir -p build && cd build
cmake .. && make
```

### Camera Configuration

The system automatically generates `config/oak_d_stereo_config.json` with:

- OAK-D Pro baseline: 75mm
- Resolution: 1280x720 at 30 FPS
- Stereo camera streams on localhost:8080 and 8081

## 🎯 Key Features

- **Real-time stereo reconstruction** at 30 FPS
- **ZMQ live streaming** for monitoring applications
- **Console-based alerts** that work in headless environments
- **Graduated detection system** to reduce false alarms
- **Offline testing capability** for validation and development
- **Cross-platform compatibility** (Linux tested)
