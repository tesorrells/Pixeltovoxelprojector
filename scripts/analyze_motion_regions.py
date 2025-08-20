#!/usr/bin/env python3
"""
Motion Region Analyzer

This script analyzes the voxel grid to find regions with the highest activity
(likely where your hand was moving) and creates focused visualizations.
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os
from mpl_toolkits.mplot3d import Axes3D

def load_voxel_grid(filename):
    """Load voxel grid from binary file"""
    with open(filename, "rb") as f:
        raw = f.read(4)
        N = np.frombuffer(raw, dtype=np.int32)[0]
        raw = f.read(4)
        voxel_size = np.frombuffer(raw, dtype=np.float32)[0]
        count = N*N*N
        raw = f.read(count*4)
        data = np.frombuffer(raw, dtype=np.float32)
        voxel_grid = data.reshape((N, N, N))
    return voxel_grid, voxel_size

def find_activity_hotspots(voxel_grid, voxel_size, grid_center, top_n=5):
    """Find the top N regions with highest activity"""
    
    # Use a very high threshold to find only the strongest signals
    non_zero_values = voxel_grid[voxel_grid > 0]
    hotspot_threshold = np.percentile(non_zero_values, 99.5)  # Top 0.5%
    
    print(f"Hotspot threshold (99.5th percentile): {hotspot_threshold:.2f}")
    
    # Find hotspot voxels
    hotspot_coords = np.argwhere(voxel_grid > hotspot_threshold)
    if len(hotspot_coords) == 0:
        print("No hotspots found! Try a lower threshold.")
        return []
    
    hotspot_intensities = voxel_grid[hotspot_coords[:, 0], hotspot_coords[:, 1], hotspot_coords[:, 2]]
    
    # Sort by intensity and take top N
    top_indices = np.argsort(hotspot_intensities)[-top_n:][::-1]
    top_coords = hotspot_coords[top_indices]
    top_intensities = hotspot_intensities[top_indices]
    
    # Convert to world coordinates
    N = voxel_grid.shape[0]
    half_side = (N * voxel_size) * 0.5
    grid_min = grid_center - half_side
    
    hotspots = []
    for i, (coord, intensity) in enumerate(zip(top_coords, top_intensities)):
        z_idx, y_idx, x_idx = coord
        x_world = grid_min[0] + (x_idx + 0.5) * voxel_size
        y_world = grid_min[1] + (y_idx + 0.5) * voxel_size  
        z_world = grid_min[2] + (z_idx + 0.5) * voxel_size
        
        hotspots.append({
            'rank': i + 1,
            'intensity': intensity,
            'position': (x_world, y_world, z_world),
            'indices': (z_idx, y_idx, x_idx)
        })
        
        print(f"Hotspot #{i+1}: Intensity {intensity:.2f} at ({x_world:.1f}, {y_world:.1f}, {z_world:.1f}) mm")
    
    return hotspots

def analyze_region_around_point(voxel_grid, center_indices, radius=10):
    """Analyze a cubic region around a point"""
    z_c, y_c, x_c = center_indices
    N = voxel_grid.shape[0]
    
    # Define region bounds
    z_min = max(0, z_c - radius)
    z_max = min(N, z_c + radius + 1)
    y_min = max(0, y_c - radius)
    y_max = min(N, y_c + radius + 1)
    x_min = max(0, x_c - radius)
    x_max = min(N, x_c + radius + 1)
    
    # Extract region
    region = voxel_grid[z_min:z_max, y_min:y_max, x_min:x_max]
    
    # Calculate statistics
    non_zero_mask = region > 0
    if np.any(non_zero_mask):
        stats = {
            'total_voxels': region.size,
            'active_voxels': np.sum(non_zero_mask),
            'total_intensity': np.sum(region),
            'max_intensity': np.max(region),
            'mean_intensity': np.mean(region[non_zero_mask]),
            'region_shape': region.shape
        }
    else:
        stats = {'total_voxels': region.size, 'active_voxels': 0}
    
    return region, stats

def create_focused_visualizations(voxel_grid, voxel_size, hotspots, output_prefix):
    """Create focused visualizations around hotspots"""
    
    grid_center = np.array([0.0, 0.0, 500.0], dtype=np.float32)
    N = voxel_grid.shape[0]
    half_side = (N * voxel_size) * 0.5
    grid_min = grid_center - half_side
    
    for hotspot in hotspots:
        rank = hotspot['rank']
        center_indices = hotspot['indices']
        
        print(f"\nAnalyzing region around hotspot #{rank}...")
        
        # Analyze region around this hotspot
        region, stats = analyze_region_around_point(voxel_grid, center_indices, radius=15)
        
        print(f"  Region stats: {stats}")
        
        if stats['active_voxels'] == 0:
            continue
            
        # Create visualization of this region
        fig = plt.figure(figsize=(15, 5))
        
        # 3D scatter plot of the region
        ax1 = fig.add_subplot(131, projection='3d')
        
        # Find non-zero voxels in region
        coords = np.argwhere(region > 0)
        if len(coords) > 0:
            intensities = region[coords[:, 0], coords[:, 1], coords[:, 2]]
            
            # Convert to world coordinates relative to region
            z_offset, y_offset, x_offset = center_indices[0] - 15, center_indices[1] - 15, center_indices[2] - 15
            z_offset = max(0, z_offset)
            y_offset = max(0, y_offset) 
            x_offset = max(0, x_offset)
            
            x_world = grid_min[0] + (coords[:, 2] + x_offset + 0.5) * voxel_size
            y_world = grid_min[1] + (coords[:, 1] + y_offset + 0.5) * voxel_size
            z_world = grid_min[2] + (coords[:, 0] + z_offset + 0.5) * voxel_size
            
            scatter = ax1.scatter(x_world, y_world, z_world, c=intensities, cmap='hot', 
                                s=20, alpha=0.7)
            
            # Mark the hotspot center
            hx, hy, hz = hotspot['position']
            ax1.scatter([hx], [hy], [hz], c='blue', s=100, marker='*', 
                       label=f'Hotspot #{rank}')
            
            ax1.set_xlabel('X (mm)')
            ax1.set_ylabel('Y (mm)')
            ax1.set_zlabel('Z (mm)')
            ax1.set_title(f'Hotspot #{rank} Region\n{stats["active_voxels"]} active voxels')
            ax1.legend()
            plt.colorbar(scatter, ax=ax1, shrink=0.5)
        
        # XY projection (top view)
        ax2 = fig.add_subplot(132)
        xy_projection = np.sum(region, axis=0)  # Sum along Z axis
        im2 = ax2.imshow(xy_projection, cmap='hot', origin='lower')
        ax2.set_title(f'Top View (XY) - Hotspot #{rank}')
        ax2.set_xlabel('X direction')
        ax2.set_ylabel('Y direction')
        plt.colorbar(im2, ax=ax2)
        
        # XZ projection (side view) 
        ax3 = fig.add_subplot(133)
        xz_projection = np.sum(region, axis=1)  # Sum along Y axis
        im3 = ax3.imshow(xz_projection, cmap='hot', origin='lower')
        ax3.set_title(f'Side View (XZ) - Hotspot #{rank}')
        ax3.set_xlabel('X direction')
        ax3.set_ylabel('Z direction')
        plt.colorbar(im3, ax=ax3)
        
        plt.tight_layout()
        
        # Save the plot
        output_file = f"{output_prefix}_hotspot_{rank}.png"
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"  Saved visualization: {output_file}")
        plt.close()

def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze_motion_regions.py <voxel_grid.bin>")
        print("Example: python analyze_motion_regions.py oak_d_original_result.bin")
        return 1
    
    filename = sys.argv[1]
    if not os.path.isfile(filename):
        print(f"File not found: {filename}")
        return 1
    
    print(f"Loading and analyzing voxel grid from: {filename}")
    voxel_grid, voxel_size = load_voxel_grid(filename)
    print(f"Loaded: {voxel_grid.shape} grid, voxel size: {voxel_size:.4f}")
    
    # Grid center from rtsp_processor.cpp
    grid_center = np.array([0.0, 0.0, 500.0], dtype=np.float32)
    
    # Find hotspots
    print("\nFinding motion hotspots (likely hand movements)...")
    hotspots = find_activity_hotspots(voxel_grid, voxel_size, grid_center, top_n=3)
    
    if not hotspots:
        print("No significant hotspots found!")
        return 1
    
    # Create focused visualizations
    base_name = os.path.splitext(os.path.basename(filename))[0]
    output_prefix = f"{base_name}_analysis"
    
    print(f"\nCreating focused visualizations...")
    create_focused_visualizations(voxel_grid, voxel_size, hotspots, output_prefix)
    
    print(f"\n✅ Analysis complete! Check the generated images:")
    print(f"   {output_prefix}_hotspot_*.png")
    print(f"\nThe hotspots show the areas with strongest motion - likely your hand movements!")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
