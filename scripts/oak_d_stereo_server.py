#!/usr/bin/env python3
"""
OAK-D Pro Stereo Camera Server
Captures both left and right camera feeds from OAK-D Pro and serves them
as separate HTTP MJPEG streams that your C++ voxel system can consume.

This allows your existing multi-camera stereo reconstruction to work with OAK-D Pro.
"""

import cv2
import depthai as dai
import threading
import time
import argparse
import signal
import sys
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
import json


class OAKDStereoServer:
    def __init__(self, resolution="720p", fps=30, base_port=8080, device_id=""):
        self.device_id = device_id
        self.fps = fps
        self.base_port = base_port
        self.left_port = base_port
        self.right_port = base_port + 1
        self.running = False
        self.frame_count = 0
        
        # Current frames for each camera
        self.left_frame = None
        self.right_frame = None
        self.left_lock = threading.Lock()
        self.right_lock = threading.Lock()
        
        # Parse resolution
        if resolution == "720p":
            self.width, self.height = 1280, 720
            self.sensor_res = dai.MonoCameraProperties.SensorResolution.THE_720_P
        elif resolution == "480p":
            self.width, self.height = 640, 480
            self.sensor_res = dai.MonoCameraProperties.SensorResolution.THE_480_P
        elif resolution == "400p":
            self.width, self.height = 640, 400
            self.sensor_res = dai.MonoCameraProperties.SensorResolution.THE_400_P
        else:
            # Default to 720p
            self.width, self.height = 1280, 720
            self.sensor_res = dai.MonoCameraProperties.SensorResolution.THE_720_P
        
        print(f"OAK-D Pro Stereo Server initialized:")
        print(f"  Resolution: {self.width}x{self.height}")
        print(f"  FPS: {self.fps}")
        print(f"  Left camera port: {self.left_port}")
        print(f"  Right camera port: {self.right_port}")
        print(f"  Device ID: {self.device_id or 'auto-detect'}")
    
    def create_pipeline(self):
        """Create DepthAI pipeline for stereo cameras."""
        pipeline = dai.Pipeline()
        
        # Create MonoCamera nodes for left and right
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        
        # Create output nodes
        xout_left = pipeline.create(dai.node.XLinkOut)
        xout_right = pipeline.create(dai.node.XLinkOut)
        
        xout_left.setStreamName("left")
        xout_right.setStreamName("right")
        
        # Properties for left camera
        mono_left.setResolution(self.sensor_res)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_left.setFps(self.fps)
        
        # Properties for right camera  
        mono_right.setResolution(self.sensor_res)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        mono_right.setFps(self.fps)
        
        # Link cameras to outputs
        mono_left.out.link(xout_left.input)
        mono_right.out.link(xout_right.input)
        
        return pipeline
    
    def camera_thread(self):
        """Camera capture thread for both stereo cameras."""
        pipeline = self.create_pipeline()
        
        try:
            # Connect to device
            if self.device_id:
                device = dai.Device(pipeline, self.device_id)
            else:
                device = dai.Device(pipeline)
            
            # Get output queues
            q_left = device.getOutputQueue(name="left", maxSize=4, blocking=False)
            q_right = device.getOutputQueue(name="right", maxSize=4, blocking=False)
            
            print(f"📸 OAK-D Pro stereo cameras started successfully")
            print(f"🔴 Capturing stereo at {self.fps} FPS...")
            
            while self.running:
                # Get frames from both cameras
                in_left = q_left.get()
                in_right = q_right.get()
                
                if in_left is not None:
                    left_frame = in_left.getCvFrame()
                    if left_frame is not None and left_frame.size > 0:
                        # Convert grayscale to BGR for consistency
                        left_frame_bgr = cv2.cvtColor(left_frame, cv2.COLOR_GRAY2BGR)
                        
                        # Add overlay
                        cv2.putText(left_frame_bgr, f"OAK-D LEFT: {self.frame_count}", 
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        
                        with self.left_lock:
                            self.left_frame = left_frame_bgr.copy()
                
                if in_right is not None:
                    right_frame = in_right.getCvFrame()
                    if right_frame is not None and right_frame.size > 0:
                        # Convert grayscale to BGR for consistency
                        right_frame_bgr = cv2.cvtColor(right_frame, cv2.COLOR_GRAY2BGR)
                        
                        # Add overlay
                        cv2.putText(right_frame_bgr, f"OAK-D RIGHT: {self.frame_count}", 
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        
                        with self.right_lock:
                            self.right_frame = right_frame_bgr.copy()
                
                self.frame_count += 1
                
                if self.frame_count % 60 == 0:
                    print(f"📊 Captured {self.frame_count} stereo frame pairs")
                
                time.sleep(0.001)
                    
        except Exception as e:
            print(f"❌ Camera error: {e}")
        finally:
            print("📸 Camera thread stopped")
    
    def get_frame_jpeg(self, is_left=True):
        """Get current frame as JPEG bytes."""
        lock = self.left_lock if is_left else self.right_lock
        frame = None
        
        with lock:
            current_frame = self.left_frame if is_left else self.right_frame
            if current_frame is not None:
                frame = current_frame.copy()
        
        if frame is not None:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
            result, encoded_img = cv2.imencode('.jpg', frame, encode_param)
            if result:
                return encoded_img.tobytes()
        return None
    
    def create_mjpeg_handler(self, camera_name, is_left):
        """Create MJPEG handler for a specific camera."""
        class MJPEGHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/stream':
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control', 'no-cache')
                    self.end_headers()
                    
                    try:
                        while self.server.stereo_server.running:
                            frame_data = self.server.stereo_server.get_frame_jpeg(is_left)
                            if frame_data:
                                self.wfile.write(b'--frame\r\n')
                                self.send_header('Content-Type', 'image/jpeg')
                                self.send_header('Content-Length', str(len(frame_data)))
                                self.end_headers()
                                self.wfile.write(frame_data)
                                self.wfile.write(b'\r\n')
                            time.sleep(1.0 / self.server.stereo_server.fps)
                    except Exception as e:
                        print(f"{camera_name} client disconnected: {e}")
                
                elif self.path == '/':
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html')
                    self.end_headers()
                    html = f"""
                    <html>
                    <head><title>OAK-D Pro {camera_name} Camera</title></head>
                    <body>
                        <h1>OAK-D Pro {camera_name} Camera</h1>
                        <p>Resolution: {self.server.stereo_server.width}x{self.server.stereo_server.height} @ {self.server.stereo_server.fps} FPS</p>
                        <p>Stream URL: <code>http://localhost:{self.server.server_port}/stream</code></p>
                        <img src="/stream" style="max-width: 100%;" />
                    </body>
                    </html>
                    """
                    self.wfile.write(html.encode())
                else:
                    self.send_error(404)
            
            def log_message(self, format, *args):
                pass  # Suppress HTTP logs
        
        return MJPEGHandler
    
    def start_server(self, port, camera_name, is_left):
        """Start HTTP server for one camera."""
        class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
            allow_reuse_address = True
        
        handler_class = self.create_mjpeg_handler(camera_name, is_left)
        server = ThreadedHTTPServer(('localhost', port), handler_class)
        server.stereo_server = self  # Reference for handler
        
        print(f"📡 {camera_name} camera server started on port {port}")
        
        try:
            server.serve_forever()
        except Exception as e:
            print(f"❌ {camera_name} server error: {e}")
        finally:
            print(f"📡 {camera_name} server stopped")
    
    def create_config_file(self):
        """Create configuration file for C++ system."""
        baseline_mm = 75  # OAK-D Pro baseline distance in mm
        
        config = [
            {
                "camera_index": 0,
                "camera_type": "rtsp",
                "rtsp_url": f"http://localhost:{self.left_port}/stream",
                "camera_position": [0.0, 0.0, 0.0],
                "yaw": 0.0,
                "pitch": 0.0,
                "roll": 0.0,
                "fov_degrees": 73.0  # OAK-D Pro mono camera FOV
            },
            {
                "camera_index": 1,
                "camera_type": "rtsp", 
                "rtsp_url": f"http://localhost:{self.right_port}/stream",
                "camera_position": [baseline_mm, 0.0, 0.0],  # Right camera offset by baseline
                "yaw": 0.0,
                "pitch": 0.0,
                "roll": 0.0,
                "fov_degrees": 73.0
            }
        ]
        
        config_path = "config/oak_d_stereo_config.json"
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=2)
        
        print(f"📄 Created stereo config: {config_path}")
        return config_path
    
    def start(self):
        """Start the stereo server."""
        self.running = True
        
        # Start camera thread
        camera_thread = threading.Thread(target=self.camera_thread, daemon=True)
        camera_thread.start()
        
        # Wait for cameras to initialize
        time.sleep(2)
        
        # Start HTTP servers in separate threads
        left_thread = threading.Thread(
            target=self.start_server, 
            args=(self.left_port, "LEFT", True), 
            daemon=True
        )
        right_thread = threading.Thread(
            target=self.start_server, 
            args=(self.right_port, "RIGHT", False), 
            daemon=True
        )
        
        left_thread.start()
        right_thread.start()
        
        # Create config file for C++ system
        config_path = self.create_config_file()
        
        print(f"\n🚀 OAK-D Pro Stereo Server running!")
        print(f"📡 Left camera:  http://localhost:{self.left_port}/stream")
        print(f"📡 Right camera: http://localhost:{self.right_port}/stream")
        print(f"🎯 To test: ./build/rtsp_processor {config_path}")
        print(f"⏹️  Press Ctrl+C to stop")
        
        try:
            # Keep main thread alive
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            print(f"\n🛑 Stopping stereo server...")
            self.stop()
    
    def stop(self):
        """Stop the stereo server."""
        self.running = False
        print("✅ Stereo server stopped")


def main():
    parser = argparse.ArgumentParser(description="OAK-D Pro Stereo Camera Server")
    parser.add_argument("--resolution", "-r", choices=["400p", "480p", "720p"], 
                       default="720p", help="Camera resolution")
    parser.add_argument("--fps", type=int, default=30, help="Target FPS")
    parser.add_argument("--base-port", type=int, default=8080, 
                       help="Base port (left=base, right=base+1)")
    parser.add_argument("--device-id", help="Specific OAK device ID")
    
    args = parser.parse_args()
    
    # Check if DepthAI is available
    try:
        devices = dai.Device.getAllAvailableDevices()
        if not devices:
            print("❌ No OAK-D Pro devices found!")
            print("   Make sure your camera is connected and udev rules are set up.")
            return 1
        
        print(f"✅ Found {len(devices)} OAK device(s)")
        for i, device in enumerate(devices):
            print(f"   Device {i}: {device.getMxId()}")
            
    except Exception as e:
        print(f"❌ DepthAI error: {e}")
        return 1
    
    # Create and start server
    server = OAKDStereoServer(
        resolution=args.resolution,
        fps=args.fps,
        base_port=args.base_port,
        device_id=args.device_id or ""
    )
    
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        server.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Start server
    server.start()
    return 0


if __name__ == "__main__":
    sys.exit(main())
