#!/usr/bin/env python3
"""
Export Voxel Visualizations to Images
Saves voxel reconstruction visualizations as PNG files you can view.
"""

import sys
import os
import struct
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def load_voxel_grid(filename):
    """Load voxel grid from binary file."""
    print(f"Loading voxel grid from: {filename}")
    
    with open(filename, 'rb') as f:
        # Read metadata (N, voxel_size)
        N_bytes = f.read(4)
        if len(N_bytes) < 4:
            raise ValueError("Incomplete file: cannot read grid size")
        N = struct.unpack('i', N_bytes)[0]
        
        voxel_size_bytes = f.read(4)
        if len(voxel_size_bytes) < 4:
            raise ValueError("Incomplete file: cannot read voxel size")
        voxel_size = struct.unpack('f', voxel_size_bytes)[0]
        
        # Read grid data
        expected_size = N * N * N * 4  # float32 = 4 bytes
        grid_bytes = f.read(expected_size)
        if len(grid_bytes) < expected_size:
            raise ValueError(f"Incomplete grid data: expected {expected_size}, got {len(grid_bytes)}")
        
        # Convert to numpy array
        grid = np.frombuffer(grid_bytes, dtype=np.float32)
        grid = grid.reshape((N, N, N))
        
        print(f"Loaded: {N}x{N}x{N} grid, voxel size: {voxel_size:.4f}")
        print(f"Grid stats: min={grid.min():.4f}, max={grid.max():.4f}, mean={grid.mean():.4f}")
        
        return grid, N, voxel_size


def create_3d_visualization(grid, N, voxel_size, threshold_percentile=90, output_prefix="voxel_3d"):
    """Create 3D visualization and save to file."""
    
    # Calculate threshold from percentile of non-zero values
    non_zero_values = grid[grid > 0]
    if len(non_zero_values) == 0:
        print("No non-zero voxels found!")
        return None
    
    threshold = np.percentile(non_zero_values, threshold_percentile)
    print(f"Using threshold: {threshold:.4f} ({threshold_percentile}th percentile)")
    
    # Find voxels above threshold
    occupied_voxels = grid > threshold
    x_indices, y_indices, z_indices = np.nonzero(occupied_voxels)
    
    if len(x_indices) == 0:
        print("No voxels above threshold!")
        return None
    
    print(f"Visualizing {len(x_indices)} voxels above threshold")
    
    # Convert indices to world coordinates
    half_size = N * voxel_size / 2.0
    x_coords = x_indices * voxel_size - half_size
    y_coords = y_indices * voxel_size - half_size
    z_coords = z_indices * voxel_size - half_size
    
    # Get intensities for coloring
    intensities = grid[occupied_voxels]
    
    # Create multiple views
    views = [
        (45, 45, "perspective"),
        (0, 0, "front"),
        (90, 0, "side"),
        (0, 90, "top")
    ]
    
    for elev, azim, view_name in views:
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Create scatter plot with color mapping
        scatter = ax.scatter(x_coords, y_coords, z_coords, 
                            c=intensities, cmap='hot', 
                            s=2, alpha=0.6, marker='.')
        
        # Add colorbar
        plt.colorbar(scatter, ax=ax, label='Voxel Intensity', shrink=0.5)
        
        # Set view angle
        ax.view_init(elev=elev, azim=azim)
        
        # Set labels and title
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_title(f'OAK-D Pro Voxel Reconstruction - {view_name.title()} View\n'
                    f'Grid: {N}³, Threshold: {threshold:.1f}, Voxels: {len(x_indices):,}')
        
        # Set equal aspect ratio
        max_range = np.array([x_coords.max()-x_coords.min(), 
                             y_coords.max()-y_coords.min(),
                             z_coords.max()-z_coords.min()]).max() / 2.0
        
        mid_x = (x_coords.max()+x_coords.min()) * 0.5
        mid_y = (y_coords.max()+y_coords.min()) * 0.5
        mid_z = (z_coords.max()+z_coords.min()) * 0.5
        
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        # Save the plot
        output_file = f"{output_prefix}_{view_name}.png"
        plt.tight_layout()
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"Saved 3D {view_name} view: {output_file}")


def create_slice_visualization(grid, N, voxel_size, output_prefix="voxel_slices"):
    """Create 2D slice visualizations and save to file."""
    
    # Calculate slice indices
    z_slice = N // 2
    y_slice = N // 2
    x_slice = N // 2
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    
    # XY slice (top view)
    im1 = axes[0,0].imshow(grid[:, :, z_slice].T, origin='lower', cmap='hot', aspect='equal')
    axes[0,0].set_title(f'XY Slice (Top View) - Z = {z_slice}')
    axes[0,0].set_xlabel('X Index')
    axes[0,0].set_ylabel('Y Index')
    plt.colorbar(im1, ax=axes[0,0])
    
    # XZ slice (side view)
    im2 = axes[0,1].imshow(grid[:, y_slice, :].T, origin='lower', cmap='hot', aspect='equal')
    axes[0,1].set_title(f'XZ Slice (Side View) - Y = {y_slice}')
    axes[0,1].set_xlabel('X Index')
    axes[0,1].set_ylabel('Z Index')
    plt.colorbar(im2, ax=axes[0,1])
    
    # YZ slice (front view)
    im3 = axes[1,0].imshow(grid[x_slice, :, :].T, origin='lower', cmap='hot', aspect='equal')
    axes[1,0].set_title(f'YZ Slice (Front View) - X = {x_slice}')
    axes[1,0].set_xlabel('Y Index')
    axes[1,0].set_ylabel('Z Index')
    plt.colorbar(im3, ax=axes[1,0])
    
    # Histogram of voxel values
    non_zero_values = grid[grid > 0]
    if len(non_zero_values) > 0:
        axes[1,1].hist(non_zero_values, bins=50, alpha=0.7, color='orange')
        axes[1,1].set_xlabel('Voxel Intensity')
        axes[1,1].set_ylabel('Count')
        axes[1,1].set_title(f'Voxel Intensity Distribution\n{len(non_zero_values):,} non-zero voxels')
        axes[1,1].set_yscale('log')
    else:
        axes[1,1].text(0.5, 0.5, 'No non-zero voxels', ha='center', va='center')
        axes[1,1].set_title('No Data')
    
    plt.tight_layout()
    output_file = f"{output_prefix}.png"
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved slice views: {output_file}")


def create_summary_stats(grid, N, voxel_size, output_file="voxel_stats.txt"):
    """Create summary statistics file."""
    
    non_zero_values = grid[grid > 0]
    
    with open(output_file, 'w') as f:
        f.write("OAK-D Pro Voxel Reconstruction Summary\n")
        f.write("="*50 + "\n\n")
        f.write(f"Grid size: {N}x{N}x{N} = {N**3:,} total voxels\n")
        f.write(f"Voxel size: {voxel_size:.4f} mm\n")
        f.write(f"Total grid volume: {(N * voxel_size)**3 / 1000**3:.2f} cubic meters\n\n")
        
        f.write("Intensity Statistics:\n")
        f.write(f"  Minimum: {grid.min():.4f}\n")
        f.write(f"  Maximum: {grid.max():.4f}\n")
        f.write(f"  Mean: {grid.mean():.4f}\n")
        f.write(f"  Standard deviation: {grid.std():.4f}\n\n")
        
        f.write("Occupancy Statistics:\n")
        f.write(f"  Non-zero voxels: {len(non_zero_values):,}\n")
        f.write(f"  Occupancy rate: {len(non_zero_values) / N**3 * 100:.3f}%\n\n")
        
        if len(non_zero_values) > 0:
            f.write("Non-zero Voxel Statistics:\n")
            f.write(f"  Mean intensity: {non_zero_values.mean():.4f}\n")
            f.write(f"  Median intensity: {np.median(non_zero_values):.4f}\n")
            f.write(f"  90th percentile: {np.percentile(non_zero_values, 90):.4f}\n")
            f.write(f"  95th percentile: {np.percentile(non_zero_values, 95):.4f}\n")
            f.write(f"  99th percentile: {np.percentile(non_zero_values, 99):.4f}\n")
    
    print(f"Saved statistics: {output_file}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 export_voxel_images.py <voxel_grid.bin> [threshold_percentile]")
        print("  threshold_percentile: 0-100, default 95 (shows top 5% brightest voxels)")
        return 1
    
    filename = sys.argv[1]
    threshold_percentile = float(sys.argv[2]) if len(sys.argv) > 2 else 95
    
    if not os.path.exists(filename):
        print(f"Error: File not found: {filename}")
        return 1
    
    try:
        # Load the voxel grid
        grid, N, voxel_size = load_voxel_grid(filename)
        
        # Get base name for output files
        base_name = os.path.splitext(os.path.basename(filename))[0]
        
        print(f"\nGenerating visualizations...")
        
        # Create 3D visualizations
        create_3d_visualization(grid, N, voxel_size, threshold_percentile, f"{base_name}_3d")
        
        # Create slice visualizations
        create_slice_visualization(grid, N, voxel_size, f"{base_name}_slices")
        
        # Create summary statistics
        create_summary_stats(grid, N, voxel_size, f"{base_name}_stats.txt")
        
        print(f"\n✅ All visualizations saved! Files:")
        print(f"   📊 3D views: {base_name}_3d_*.png")
        print(f"   🔍 Slice views: {base_name}_slices.png")
        print(f"   📄 Statistics: {base_name}_stats.txt")
        
    except Exception as e:
        print(f"Error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
