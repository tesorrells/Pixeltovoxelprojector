#!/usr/bin/env python3
"""
Console-based Real-time Drone Monitor

Works in headless environments - shows detection alerts and statistics 
in the terminal without requiring GUI windows.
"""

import zmq
import numpy as np
import time
import sys
import zlib
from collections import deque
import threading
import queue
import json

class ConsoleDroneMonitor:
    def __init__(self, zmq_port=5556):
        self.zmq_port = zmq_port
        self.running = True
        
        # ZMQ setup
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://localhost:{zmq_port}")
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")  # Subscribe to all messages
        self.socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
        
        # Detection parameters - balanced for real drone detection
        self.detection_threshold = 200   # Threshold for meaningful detections
        self.minor_detection_threshold = 50   # Small objects or distant motion
        self.moderate_detection_threshold = 120  # Medium-sized objects or hand waving
        self.major_detection_threshold = 250  # Large objects, close hand waving, or drones
        
        # Statistics tracking
        self.frame_count = 0
        self.detection_count = 0
        self.major_detection_count = 0
        self.fps_deque = deque(maxlen=30)
        self.last_frame_time = time.time()
        self.start_time = time.time()
        
        # Detection history
        self.recent_detections = deque(maxlen=10)
        
        print("🚁 Console Drone Detection Monitor")
        print("=" * 50)
        print(f"🔗 Connected to ZMQ stream on port {zmq_port}")
        print(f"🎯 Detection threshold: {self.detection_threshold}")
        print(f"🔍 Minor alerts: {self.minor_detection_threshold}+ points")
        print(f"⚠️  Moderate alerts: {self.moderate_detection_threshold}+ points") 
        print(f"🚨 Major alerts: {self.major_detection_threshold}+ points")
        print("📊 Monitoring for drone activity...")
        print("-" * 50)
    
    def parse_voxel_data(self, data):
        """Parse compressed voxel grid data from ZMQ"""
        try:
            meta_data = data[0]
            compressed_data = data[1]
            
            # Unpack metadata
            if len(meta_data) >= 8:
                N = int.from_bytes(meta_data[0:4], byteorder='little')
                voxel_size = np.frombuffer(meta_data[4:8], dtype=np.float32)[0]
            else:
                return None, None, None
            
            # Decompress voxel data
            decompressed = zlib.decompress(bytes(compressed_data))
            voxel_grid = np.frombuffer(decompressed, dtype=np.float32).reshape((N, N, N))
            
            return voxel_grid, N, voxel_size
            
        except Exception as e:
            print(f"❌ Error parsing data: {e}")
            return None, None, None
    
    def voxel_to_world(self, voxel_coords, grid_size, voxel_size):
        """Convert voxel indices to world coordinates"""
        grid_center = np.array([0.0, 0.0, 500.0])
        half_size = (grid_size * voxel_size) * 0.5
        grid_min = grid_center - half_size
        
        world_coords = []
        for coord in voxel_coords:
            z_idx, y_idx, x_idx = coord
            x_world = grid_min[0] + (x_idx + 0.5) * voxel_size
            y_world = grid_min[1] + (y_idx + 0.5) * voxel_size
            z_world = grid_min[2] + (z_idx + 0.5) * voxel_size
            world_coords.append([x_world, y_world, z_world])
        
        return np.array(world_coords)
    
    def detect_objects(self, voxel_grid, grid_size, voxel_size):
        """Extract and analyze objects from voxel grid"""
        if voxel_grid is None:
            return []
        
        # Dynamic threshold - conservative approach
        max_intensity = voxel_grid.max()
        if max_intensity > 0:
            # Use a lower threshold only for very high-intensity data (very close objects)
            if max_intensity > 5000:
                threshold = max(100, np.percentile(voxel_grid[voxel_grid > 0], 88))
            else:
                threshold = max(self.detection_threshold, np.percentile(voxel_grid[voxel_grid > 0], 92))
        else:
            threshold = self.detection_threshold
        
        # Find significant voxels
        significant_mask = voxel_grid > threshold
        coords = np.argwhere(significant_mask)
        
        if len(coords) == 0:
            return []
        
        # Convert to world coordinates
        world_coords = self.voxel_to_world(coords, grid_size, voxel_size)
        intensities = voxel_grid[coords[:, 0], coords[:, 1], coords[:, 2]]
        
        # Calculate object properties
        centroid = np.mean(world_coords, axis=0)
        distance = np.linalg.norm(centroid)
        max_intensity = intensities.max()
        
        detection = {
            'timestamp': time.time(),
            'point_count': len(coords),
            'centroid': centroid,
            'distance': distance,
            'max_intensity': max_intensity,
            'spread': np.std(world_coords, axis=0),
            'intensities': intensities
        }
        
        return [detection] if len(coords) >= 5 else []  # Filter noise
    
    def print_status_line(self, detections):
        """Print a real-time status line"""
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        # Calculate FPS
        fps = 1.0 / (current_time - self.last_frame_time) if current_time > self.last_frame_time else 0
        self.fps_deque.append(fps)
        avg_fps = np.mean(self.fps_deque) if self.fps_deque else 0
        self.last_frame_time = current_time
        
        self.frame_count += 1
        
        # Status line
        status = f"⏱️  {elapsed:6.1f}s | 📊 {self.frame_count:4d} frames | ⚡ {avg_fps:4.1f} FPS | "
        
        if detections:
            detection = detections[0]  # Primary detection
            points = detection['point_count']
            dist = detection['distance']
            x, y, z = detection['centroid']
            
            max_int = detection['max_intensity']
            status += f"🎯 {points:3d} pts @ ({x:4.0f},{y:4.0f},{z:4.0f})mm, {dist:4.0f}mm away, max:{max_int:.0f}"
            
            # Alert levels based on point count AND intensity
            
            # Major alert: many points AND high intensity (genuine close object)
            if points >= self.major_detection_threshold and max_int > 3000:
                status = "🚨 MAJOR ALERT! " + status
                self.major_detection_count += 1
            # Moderate alert: medium points OR high intensity
            elif points >= self.moderate_detection_threshold or max_int > 2500:
                status = "⚠️  MODERATE    " + status
            elif points >= self.minor_detection_threshold:
                status = "🔍 minor       " + status
            else:
                status = "   trace      " + status
            
            self.detection_count += 1
            self.recent_detections.append(detection)
            
        else:
            status += "🔍 monitoring..."
        
        # Clear line and print
        print(f"\r{status:<100}", end="", flush=True)
    
    def print_detailed_alert(self, detection):
        """Print detailed information for significant detections"""
        if detection['point_count'] >= self.major_detection_threshold:
            print()  # New line after status
            print("🚨" * 20)
            print(f"  MAJOR DRONE DETECTION ALERT!")
            print(f"  Time: {time.strftime('%H:%M:%S')}")
            print(f"  Points: {detection['point_count']}")
            print(f"  Location: ({detection['centroid'][0]:.0f}, {detection['centroid'][1]:.0f}, {detection['centroid'][2]:.0f}) mm")
            print(f"  Distance: {detection['distance']:.0f} mm")
            print(f"  Max Intensity: {detection['max_intensity']:.0f}")
            print(f"  Spread: X={detection['spread'][0]:.0f}, Y={detection['spread'][1]:.0f}, Z={detection['spread'][2]:.0f} mm")
            print("🚨" * 20)
    
    def print_summary(self):
        """Print session summary"""
        elapsed = time.time() - self.start_time
        detection_rate = (self.detection_count / self.frame_count * 100) if self.frame_count > 0 else 0
        major_rate = (self.major_detection_count / self.frame_count * 100) if self.frame_count > 0 else 0
        
        print("\n" + "=" * 60)
        print("📊 SESSION SUMMARY")
        print("=" * 60)
        print(f"⏱️  Duration: {elapsed:.1f} seconds")
        print(f"📊 Total frames: {self.frame_count}")
        print(f"🎯 Detections: {self.detection_count} ({detection_rate:.1f}%)")
        print(f"🚨 Major alerts: {self.major_detection_count} ({major_rate:.1f}%)")
        print(f"⚡ Average FPS: {np.mean(self.fps_deque) if self.fps_deque else 0:.1f}")
        
        if self.recent_detections:
            print("\n🔍 RECENT ACTIVITY:")
            for i, det in enumerate(list(self.recent_detections)[-3:]):
                age = time.time() - det['timestamp']
                print(f"  {i+1}. {age:.1f}s ago: {det['point_count']} points at {det['distance']:.0f}mm")
        
        print("=" * 60)
    
    def data_receiver_thread(self):
        """Background thread to receive ZMQ data"""
        data_queue = queue.Queue(maxsize=5)
        
        while self.running:
            try:
                parts = self.socket.recv_multipart(zmq.NOBLOCK)
                if len(parts) >= 2:
                    try:
                        data_queue.put_nowait(parts)
                    except queue.Full:
                        # Drop old data
                        try:
                            data_queue.get_nowait()
                            data_queue.put_nowait(parts)
                        except queue.Empty:
                            pass
                            
            except zmq.Again:
                time.sleep(0.01)
            except Exception as e:
                if self.running:
                    print(f"\n❌ ZMQ error: {e}")
                time.sleep(0.1)
        
        return data_queue
    
    def run(self):
        """Main monitoring loop"""
        print("🚀 Starting console monitoring...")
        
        try:
            while self.running:
                # Get data
                try:
                    parts = self.socket.recv_multipart(zmq.NOBLOCK)
                    if len(parts) >= 2:
                        # Parse and analyze
                        voxel_grid, grid_size, voxel_size = self.parse_voxel_data(parts)
                        if voxel_grid is not None:
                            detections = self.detect_objects(voxel_grid, grid_size, voxel_size)
                            
                            # Update display
                            self.print_status_line(detections)
                            
                            # Detailed alerts for major detections
                            for detection in detections:
                                if detection['point_count'] >= self.major_detection_threshold:
                                    self.print_detailed_alert(detection)
                                    
                except zmq.Again:
                    # No data available
                    time.sleep(0.05)
                    
        except KeyboardInterrupt:
            print("\n🛑 Monitoring stopped by user")
        finally:
            self.running = False
            self.print_summary()
            self.socket.close()
            self.context.term()

def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 5556
    
    monitor = ConsoleDroneMonitor(port)
    monitor.run()

if __name__ == "__main__":
    main()
