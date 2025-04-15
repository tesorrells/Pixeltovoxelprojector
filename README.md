# Pixel To Voxel Projector

A C++/Python toolkit for reconstructing 3D objects from multiple camera views by mapping motion detection from 2D image space into 3D voxel space.

## Features

- Process multiple camera feeds in real-time using RTSP streams
- Detect motion between consecutive frames
- Cast rays from changed pixels into a 3D voxel grid
- Accumulate brightness in overlapping regions to reconstruct moving objects
- Visualize the 3D reconstruction in real-time

## Getting Started

### Installation

Run the setup script to install all dependencies:

```bash
sudo ./setup.sh
```

### Configuration

Configure your cameras in the `camera_config.json` file:

```json
[
  {
    "camera_index": 0,
    "rtsp_url": "rtsp://admin:password@192.168.1.100:554/live",
    "camera_position": [0.0, 0.0, 0.0],
    "yaw": 0.0,
    "pitch": 0.0,
    "roll": 0.0,
    "fov_degrees": 60.0
  },
  ...
]
```

### Running the Real-time Processor

1. Start the RTSP stream processor:

```bash
./build/rtsp_processor camera_config.json output_dir
```

2. Visualize the voxel grid in real-time:

```bash
python3 visualize_voxel_grid.py output_dir
```

## Controls for Visualization

- Mouse: Look around
- W/A/S/D: Move camera
- Space/C: Move up/down
- +/-: Adjust threshold
- Q: Quit

## Batch Processing

The project also supports batch processing of pre-recorded images using the original `ray_voxel.cpp` implementation:

```bash
./build/ray_voxel <metadata.json> <image_folder> <output_voxel_bin>
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.
