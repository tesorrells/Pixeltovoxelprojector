#!/usr/bin/env python3
"""
OAK-D Pro Frame Capture for Offline Testing

Captures synchronized frames from both OAK-D Pro mono cameras
and saves them for offline analysis with the ray_voxel.cpp program.
"""

import cv2
import depthai as dai
import time
import os
import json
from datetime import datetime

def capture_oak_d_frames(output_dir="test_captures", duration_seconds=10, fps_limit=2):
    """
    Capture synchronized frames from OAK-D Pro stereo cameras
    
    Args:
        output_dir: Directory to save captured frames
        duration_seconds: How long to capture (seconds)
        fps_limit: Maximum frames per second to capture
    """
    
    # Create output directories
    left_dir = os.path.join(output_dir, "left")
    right_dir = os.path.join(output_dir, "right")
    os.makedirs(left_dir, exist_ok=True)
    os.makedirs(right_dir, exist_ok=True)
    
    # Create pipeline
    pipeline = dai.Pipeline()
    
    # Define sources and outputs
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    
    left_out = pipeline.create(dai.node.XLinkOut)
    right_out = pipeline.create(dai.node.XLinkOut)
    
    left_out.setStreamName("left")
    right_out.setStreamName("right")
    
    # Properties
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)  # Left camera
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)  # Right camera
    
    # Linking
    mono_left.out.link(left_out.input)
    mono_right.out.link(right_out.input)
    
    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        print(f"🚀 Starting OAK-D frame capture...")
        print(f"📁 Output directory: {output_dir}")
        print(f"⏱️  Duration: {duration_seconds} seconds")
        print(f"📸 Target FPS: {fps_limit}")
        
        # Output queues
        q_left = device.getOutputQueue(name="left", maxSize=4, blocking=False)
        q_right = device.getOutputQueue(name="right", maxSize=4, blocking=False)
        
        frame_count = 0
        start_time = time.time()
        last_capture_time = 0
        frame_interval = 1.0 / fps_limit
        
        capture_metadata = {
            "capture_info": {
                "timestamp": datetime.now().isoformat(),
                "duration_seconds": duration_seconds,
                "fps_limit": fps_limit,
                "resolution": "720p",
                "baseline_mm": 75.0  # OAK-D Pro baseline distance
            },
            "frames": []
        }
        
        while True:
            current_time = time.time()
            elapsed = current_time - start_time
            
            # Check if capture duration exceeded
            if elapsed >= duration_seconds:
                break
            
            # Rate limiting
            if current_time - last_capture_time < frame_interval:
                time.sleep(0.01)
                continue
                
            # Get frames
            in_left = q_left.tryGet()
            in_right = q_right.tryGet()
            
            if in_left is not None and in_right is not None:
                # Convert to OpenCV format
                left_frame = in_left.getCvFrame()
                right_frame = in_right.getCvFrame()
                
                # Generate frame filenames
                left_filename = f"frame_{frame_count:06d}.png"
                right_filename = f"frame_{frame_count:06d}.png"
                
                left_path = os.path.join(left_dir, left_filename)
                right_path = os.path.join(right_dir, right_filename)
                
                # Save frames
                cv2.imwrite(left_path, left_frame)
                cv2.imwrite(right_path, right_frame)
                
                # Record metadata
                frame_metadata = {
                    "frame_index": frame_count,
                    "timestamp": current_time,
                    "elapsed_seconds": elapsed,
                    "left_file": left_filename,
                    "right_file": right_filename
                }
                capture_metadata["frames"].append(frame_metadata)
                
                print(f"📸 Captured frame {frame_count:06d} at {elapsed:.1f}s")
                
                frame_count += 1
                last_capture_time = current_time
        
        # Save metadata
        metadata_path = os.path.join(output_dir, "capture_metadata.json")
        with open(metadata_path, 'w') as f:
            json.dump(capture_metadata, f, indent=2)
        
        print(f"✅ Capture complete!")
        print(f"📊 Captured {frame_count} frame pairs")
        print(f"📄 Metadata saved to: {metadata_path}")
        print(f"🗂️  Frame files saved to: {left_dir} and {right_dir}")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Capture OAK-D Pro frames for offline testing")
    parser.add_argument("--output", "-o", default="test_captures", help="Output directory")
    parser.add_argument("--duration", "-d", type=float, default=10.0, help="Capture duration in seconds")
    parser.add_argument("--fps", "-f", type=float, default=2.0, help="Frames per second to capture")
    
    args = parser.parse_args()
    
    try:
        capture_oak_d_frames(args.output, args.duration, args.fps)
    except KeyboardInterrupt:
        print("\n🛑 Capture stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
