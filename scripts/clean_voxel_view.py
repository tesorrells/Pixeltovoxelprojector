#!/usr/bin/env python3
"""
Clean Voxel Viewer - Shows only significant motion, filters noise
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

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

def create_clean_visualization(voxel_grid, voxel_size, percentile_threshold=99):
    """Create a clean visualization showing only significant motion"""
    
    # Calculate threshold to filter noise
    non_zero_values = voxel_grid[voxel_grid > 0]
    threshold = np.percentile(non_zero_values, percentile_threshold)
    
    print(f"Using {percentile_threshold}th percentile threshold: {threshold:.2f}")
    
    # Find significant voxels
    coords = np.argwhere(voxel_grid > threshold)
    if len(coords) == 0:
        print("No voxels above threshold!")
        return
    
    intensities = voxel_grid[coords[:, 0], coords[:, 1], coords[:, 2]]
    print(f"Showing {len(coords):,} significant voxels (filtered from {len(non_zero_values):,})")
    
    # Convert to world coordinates
    grid_center = np.array([0.0, 0.0, 500.0], dtype=np.float32)
    N = voxel_grid.shape[0]
    half_side = (N * voxel_size) * 0.5
    grid_min = grid_center - half_side
    
    # Array is organized as [Z, Y, X]
    z_idx = coords[:, 0] + 0.5
    y_idx = coords[:, 1] + 0.5
    x_idx = coords[:, 2] + 0.5
    
    x_world = grid_min[0] + x_idx * voxel_size
    y_world = grid_min[1] + y_idx * voxel_size
    z_world = grid_min[2] + z_idx * voxel_size
    
    # Create the visualization
    fig = plt.figure(figsize=(16, 12))
    
    # Main 3D view
    ax1 = fig.add_subplot(221, projection='3d')
    scatter = ax1.scatter(x_world, y_world, z_world, c=intensities, cmap='hot', 
                         s=10, alpha=0.8)
    ax1.set_xlabel('X (mm)')
    ax1.set_ylabel('Y (mm)')
    ax1.set_zlabel('Z (mm)')
    ax1.set_title(f'Clean 3D View - Significant Motion Only\n({len(coords):,} voxels above {percentile_threshold}th percentile)')
    plt.colorbar(scatter, ax=ax1, shrink=0.7)
    
    # Add camera positions for reference
    ax1.scatter([0, 75], [0, 0], [0, 0], c=['blue', 'blue'], s=100, marker='s', 
               label='Cameras (Left=0, Right=75mm)')
    ax1.legend()
    
    # XY view (top down)
    ax2 = fig.add_subplot(222)
    ax2.scatter(x_world, y_world, c=intensities, cmap='hot', s=5, alpha=0.7)
    ax2.set_xlabel('X (mm)')
    ax2.set_ylabel('Y (mm)')
    ax2.set_title('Top View (XY)')
    ax2.grid(True, alpha=0.3)
    # Mark camera positions
    ax2.scatter([0, 75], [0, 0], c='blue', s=50, marker='s')
    
    # XZ view (side view)
    ax3 = fig.add_subplot(223)
    ax3.scatter(x_world, z_world, c=intensities, cmap='hot', s=5, alpha=0.7)
    ax3.set_xlabel('X (mm)')
    ax3.set_ylabel('Z (mm) - Distance from cameras')
    ax3.set_title('Side View (XZ)')
    ax3.grid(True, alpha=0.3)
    
    # YZ view (front view)
    ax4 = fig.add_subplot(224)
    scatter4 = ax4.scatter(y_world, z_world, c=intensities, cmap='hot', s=5, alpha=0.7)
    ax4.set_xlabel('Y (mm)')
    ax4.set_ylabel('Z (mm) - Distance from cameras')
    ax4.set_title('Front View (YZ)')
    ax4.grid(True, alpha=0.3)
    plt.colorbar(scatter4, ax=ax4)
    
    plt.tight_layout()
    
    # Print some statistics about the motion
    print(f"\nMotion Statistics:")
    print(f"  X range: {x_world.min():.1f} to {x_world.max():.1f} mm")
    print(f"  Y range: {y_world.min():.1f} to {y_world.max():.1f} mm") 
    print(f"  Z range: {z_world.min():.1f} to {z_world.max():.1f} mm")
    print(f"  Distance from camera: {z_world.min():.1f} to {z_world.max():.1f} mm")
    print(f"  Intensity range: {intensities.min():.0f} to {intensities.max():.0f}")
    
    return fig

def main():
    if len(sys.argv) < 2:
        print("Usage: python clean_voxel_view.py <voxel_grid.bin> [percentile_threshold]")
        print("Example: python clean_voxel_view.py oak_d_original_result.bin 99")
        print("  percentile_threshold: 90-99.9 (higher = less noise, default=99)")
        return 1
    
    filename = sys.argv[1]
    percentile = float(sys.argv[2]) if len(sys.argv) > 2 else 99.0
    
    if not os.path.isfile(filename):
        print(f"File not found: {filename}")
        return 1
    
    print(f"Loading voxel grid from: {filename}")
    voxel_grid, voxel_size = load_voxel_grid(filename)
    print(f"Loaded: {voxel_grid.shape} grid, voxel size: {voxel_size:.4f}")
    
    # Create clean visualization
    fig = create_clean_visualization(voxel_grid, voxel_size, percentile)
    
    if fig is not None:
        # Save the plot
        base_name = os.path.splitext(os.path.basename(filename))[0]
        output_file = f"{base_name}_clean_view.png"
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"\n✅ Clean visualization saved: {output_file}")
        
        # Show the plot
        plt.show()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
