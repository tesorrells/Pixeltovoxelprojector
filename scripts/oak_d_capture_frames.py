#!/usr/bin/env python3
"""
OAK-D Pro Frame Capture for Offline Testing

Captures synchronized frames from both left and right cameras of OAK-D Pro
and saves them as numbered image files with timestamps for offline processing.
"""

import cv2
import depthai as dai
import time
import os
import json
import numpy as np
from datetime import datetime
import argparse

def create_oak_d_pipeline():
    """Create DepthAI pipeline for capturing both mono cameras"""
    pipeline = dai.Pipeline()
    
    # Create left and right mono camera nodes
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    
    # Create output nodes
    left_out = pipeline.create(dai.node.XLinkOut)
    right_out = pipeline.create(dai.node.XLinkOut)
    
    # Set stream names
    left_out.setStreamName("left")
    right_out.setStreamName("right")
    
    # Configure cameras
    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)  # Left camera
    mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)  # Right camera
    
    # Set resolution and FPS
    resolution = dai.MonoCameraProperties.SensorResolution.THE_720_P
    mono_left.setResolution(resolution)
    mono_right.setResolution(resolution)
    mono_left.setFps(30)
    mono_right.setFps(30)
    
    # Link cameras to outputs
    mono_left.out.link(left_out.input)
    mono_right.out.link(right_out.input)
    
    return pipeline

def capture_frames(output_dir, duration_seconds=10, fps_target=30):
    """Capture frames from both cameras for specified duration"""
    
    # Create output directories
    left_dir = os.path.join(output_dir, "left")
    right_dir = os.path.join(output_dir, "right")
    os.makedirs(left_dir, exist_ok=True)
    os.makedirs(right_dir, exist_ok=True)
    
    # Create pipeline and connect to device
    pipeline = create_oak_d_pipeline()
    
    with dai.Device(pipeline) as device:
        print(f"🎥 Connected to OAK-D Pro: {device.getDeviceInfo().name}")
        
        # Get output queues
        q_left = device.getOutputQueue(name="left", maxSize=4, blocking=False)
        q_right = device.getOutputQueue(name="right", maxSize=4, blocking=False)
        
        start_time = time.time()
        frame_count = 0
        target_frame_interval = 1.0 / fps_target
        last_capture_time = 0
        
        frames_metadata = []
        
        print(f"📸 Starting capture for {duration_seconds} seconds at ~{fps_target} FPS...")
        print("Move objects in front of the cameras for testing!")
        
        while time.time() - start_time < duration_seconds:
            current_time = time.time()
                
            # Get frames from both cameras
            in_left = q_left.tryGet()
            in_right = q_right.tryGet()
            
            if in_left is not None and in_right is not None:
                # Skip frame rate limiting for now to ensure we get frames
                # if current_time - last_capture_time < target_frame_interval:
                #     continue
                # Get timestamps
                timestamp_left = in_left.getTimestamp()
                timestamp_right = in_right.getTimestamp()
                
                # Convert to OpenCV format
                frame_left = in_left.getCvFrame()
                frame_right = in_right.getCvFrame()
                
                # Save frames with consistent numbering
                frame_filename = f"frame_{frame_count:06d}.png"
                left_path = os.path.join(left_dir, frame_filename)
                right_path = os.path.join(right_dir, frame_filename)
                
                cv2.imwrite(left_path, frame_left)
                cv2.imwrite(right_path, frame_right)
                
                # Store metadata
                frame_metadata = {
                    "frame_index": frame_count,
                    "timestamp_left": timestamp_left.total_seconds(),
                    "timestamp_right": timestamp_right.total_seconds(),
                    "capture_time": current_time,
                    "left_image": f"left/{frame_filename}",
                    "right_image": f"right/{frame_filename}"
                }
                frames_metadata.append(frame_metadata)
                
                frame_count += 1
                last_capture_time = current_time
                
                # Progress indicator
                elapsed = current_time - start_time
                if frame_count % 30 == 0:  # Every second at 30 FPS
                    print(f"📊 Captured {frame_count} frame pairs ({elapsed:.1f}s elapsed)")
    
    print(f"✅ Capture complete! Saved {frame_count} frame pairs")
    
    # Save metadata
    metadata_path = os.path.join(output_dir, "capture_metadata.json")
    with open(metadata_path, 'w') as f:
        json.dump({
            "capture_info": {
                "duration_seconds": duration_seconds,
                "total_frames": frame_count,
                "fps_target": fps_target,
                "start_time": start_time,
                "device_info": str(device.getDeviceInfo().name) if 'device' in locals() else "unknown"
            },
            "frames": frames_metadata
        }, f, indent=2)
    
    print(f"📄 Metadata saved to: {metadata_path}")
    return frame_count, metadata_path

def main():
    parser = argparse.ArgumentParser(description="Capture frames from OAK-D Pro for offline testing")
    parser.add_argument("--output-dir", default="oak_d_captures", 
                       help="Output directory for captured frames")
    parser.add_argument("--duration", type=int, default=10,
                       help="Capture duration in seconds")
    parser.add_argument("--fps", type=int, default=15,
                       help="Target frames per second")
    
    args = parser.parse_args()
    
    print("🚀 OAK-D Pro Frame Capture")
    print(f"📁 Output directory: {args.output_dir}")
    print(f"⏱️  Duration: {args.duration} seconds")
    print(f"🎞️  Target FPS: {args.fps}")
    
    try:
        frame_count, metadata_path = capture_frames(
            args.output_dir, 
            args.duration, 
            args.fps
        )
        
        print(f"\n✨ Success! Captured {frame_count} synchronized frame pairs")
        print(f"📂 Files saved in: {args.output_dir}/")
        print(f"   📸 Left frames: {args.output_dir}/left/")
        print(f"   📸 Right frames: {args.output_dir}/right/")
        print(f"   📄 Metadata: {metadata_path}")
        
    except Exception as e:
        print(f"❌ Error during capture: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())
