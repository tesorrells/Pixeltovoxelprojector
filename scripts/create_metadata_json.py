#!/usr/bin/env python3
"""Create metadata.json for captured OAK-D Pro frames"""

import json
import os
import glob

def create_metadata_json(frames_dir="simple_captures", output_file="metadata.json"):
    """Create metadata.json for the original ray_voxel.cpp implementation"""
    
    # Get list of left and right frames
    left_frames = sorted(glob.glob(os.path.join(frames_dir, "left", "frame_*.png")))
    right_frames = sorted(glob.glob(os.path.join(frames_dir, "right", "frame_*.png")))
    
    print(f"Found {len(left_frames)} left frames and {len(right_frames)} right frames")
    
    if len(left_frames) != len(right_frames):
        print("Warning: Unequal number of left and right frames")
    
    frames_metadata = []
    
    # OAK-D Pro stereo baseline (distance between cameras) in mm
    # The OAK-D Pro has a baseline of approximately 75mm
    baseline_mm = 75.0
    
    # Camera parameters based on OAK-D Pro specifications
    # FOV for 720p mono cameras is approximately 73 degrees
    fov_degrees = 73.0
    
    # Process all frames
    num_frames = min(len(left_frames), len(right_frames))
    
    for i in range(num_frames):
        left_frame = left_frames[i]
        right_frame = right_frames[i]
        
        # Extract frame index from filename
        left_basename = os.path.basename(left_frame)
        frame_idx = int(left_basename.split('_')[1].split('.')[0])
        
        # Convert to relative paths (ray_voxel.cpp will prefix with images_folder)
        left_rel_path = os.path.relpath(left_frame, frames_dir)
        right_rel_path = os.path.relpath(right_frame, frames_dir)
        
        # Left camera (camera_index = 0) - positioned to match expected coordinate system
        left_entry = {
            "camera_index": 0,
            "frame_index": frame_idx,
            "camera_position": [-baseline_mm, 0.0, 0.0],  # Left camera at negative X
            "yaw": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
            "fov_degrees": fov_degrees,
            "image_file": left_rel_path
        }
        frames_metadata.append(left_entry)
        
        # Right camera (camera_index = 1)
        right_entry = {
            "camera_index": 1,
            "frame_index": frame_idx,
            "camera_position": [0.0, 0.0, 0.0],  # Right camera at origin
            "yaw": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
            "fov_degrees": fov_degrees,
            "image_file": right_rel_path
        }
        frames_metadata.append(right_entry)
    
    # Save metadata
    with open(output_file, 'w') as f:
        json.dump(frames_metadata, f, indent=2)
    
    print(f"Created metadata.json with {len(frames_metadata)} entries")
    print(f"Covers {num_frames} frame pairs from both cameras")
    print(f"Saved to: {output_file}")
    
    return output_file

if __name__ == "__main__":
    create_metadata_json()
