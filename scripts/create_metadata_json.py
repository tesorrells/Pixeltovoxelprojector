#!/usr/bin/env python3
"""
Create Metadata JSON for Offline Ray Voxel Processing

Converts captured OAK-D Pro frame data into the metadata format
expected by ray_voxel.cpp for offline stereo reconstruction testing.
"""

import json
import os
import sys
from datetime import datetime

def create_metadata_json(images_folder, output_json="metadata.json", fov_degrees=85.0, baseline_mm=75.0):
    """
    Create metadata JSON for ray_voxel.cpp from captured OAK-D frames
    
    Args:
        images_folder: Base folder containing left/ and right/ subdirectories
        output_json: Output JSON file path
        fov_degrees: Camera field of view in degrees
        baseline_mm: Distance between cameras in millimeters
    """
    
    left_folder = os.path.join(images_folder, "left")
    right_folder = os.path.join(images_folder, "right")
    
    if not os.path.exists(left_folder) or not os.path.exists(right_folder):
        raise ValueError(f"Left or right camera folders not found in {images_folder}")
    
    # Get list of image files
    left_files = sorted([f for f in os.listdir(left_folder) if f.endswith('.png')])
    right_files = sorted([f for f in os.listdir(right_folder) if f.endswith('.png')])
    
    if len(left_files) != len(right_files):
        raise ValueError(f"Mismatch: {len(left_files)} left files vs {len(right_files)} right files")
    
    print(f"📁 Processing {len(left_files)} frame pairs from {images_folder}")
    
    # Create metadata structure
    frames_metadata = []
    
    for frame_idx, (left_file, right_file) in enumerate(zip(left_files, right_files)):
        # Use relative paths from the images_folder base
        left_rel_path = os.path.relpath(os.path.join(left_folder, left_file), images_folder)
        right_rel_path = os.path.relpath(os.path.join(right_folder, right_file), images_folder)
        
        # Left camera entry (camera_index = 0) - positioned to match corrected coordinate system
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
        
        # Right camera entry (camera_index = 1)
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
    
    # Create full metadata structure
    metadata = {
        "generation_info": {
            "created_by": "create_metadata_json.py",
            "timestamp": datetime.now().isoformat(),
            "source_folder": images_folder,
            "frame_count": len(left_files),
            "camera_count": 2
        },
        "camera_setup": {
            "baseline_mm": baseline_mm,
            "fov_degrees": fov_degrees,
            "coordinate_system": "right_handed",
            "left_camera_position": [-baseline_mm, 0.0, 0.0],
            "right_camera_position": [0.0, 0.0, 0.0]
        },
        "frames": frames_metadata
    }
    
    # Write metadata JSON
    output_path = os.path.join(images_folder, output_json)
    with open(output_path, 'w') as f:
        json.dump(metadata, f, indent=2)
    
    print(f"✅ Created metadata JSON: {output_path}")
    print(f"📊 Total entries: {len(frames_metadata)} ({len(left_files)} frame pairs × 2 cameras)")
    print(f"🎯 Baseline: {baseline_mm}mm, FOV: {fov_degrees}°")
    
    return output_path

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Create metadata JSON for offline ray voxel processing")
    parser.add_argument("images_folder", help="Folder containing left/ and right/ subdirectories with captured frames")
    parser.add_argument("--output", "-o", default="metadata.json", help="Output JSON filename (saved in images_folder)")
    parser.add_argument("--fov", type=float, default=85.0, help="Camera field of view in degrees")
    parser.add_argument("--baseline", type=float, default=75.0, help="Camera baseline distance in mm")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.images_folder):
        print(f"❌ Error: Folder '{args.images_folder}' does not exist")
        sys.exit(1)
    
    try:
        create_metadata_json(args.images_folder, args.output, args.fov, args.baseline)
        print(f"\n🎯 Usage: ./build/ray_voxel {args.images_folder} {args.output}")
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()