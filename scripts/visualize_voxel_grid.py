#!/usr/bin/env python3
"""
Real-time 3D Voxel Grid Visualizer

This script monitors a directory for new voxel grid files and 
visualizes them in real-time using OpenGL.

Usage:
    python visualize_voxel_grid.py <voxel_grid_directory> [threshold]
    
    - voxel_grid_directory: Directory where voxel grid files are saved
    - threshold: Optional brightness threshold for visualization (default: 0.5)
"""

import sys
import os
import time
import numpy as np
import struct
import glob
from datetime import datetime
from threading import Thread, Lock

# OpenGL visualization
try:
    from OpenGL.GL import *
    from OpenGL.GLU import *
    from OpenGL.GLUT import *
except ImportError:
    print("ERROR: PyOpenGL not installed. Install with:")
    print("pip install PyOpenGL PyOpenGL_accelerate")
    sys.exit(1)

# Camera control
class Camera:
    def __init__(self):
        self.pos = np.array([0.0, 0.0, 800.0])
        self.yaw = 0.0
        self.pitch = -30.0
        self.speed = 5.0
        self.mouse_sensitivity = 0.2
        self.last_x = 0
        self.last_y = 0
        self.first_mouse = True
        
    def update_view(self):
        # Convert to radians
        yaw_rad = np.radians(self.yaw)
        pitch_rad = np.radians(self.pitch)
        
        # Calculate the camera's direction vector
        direction = np.array([
            np.cos(yaw_rad) * np.cos(pitch_rad),
            np.sin(pitch_rad),
            np.sin(yaw_rad) * np.cos(pitch_rad)
        ])
        
        # Normalize
        direction = direction / np.linalg.norm(direction)
        
        # Set the view matrix using gluLookAt
        target = self.pos + direction
        gluLookAt(
            self.pos[0], self.pos[1], self.pos[2],
            target[0], target[1], target[2],
            0.0, 1.0, 0.0
        )

# Voxel Grid data
class VoxelGrid:
    def __init__(self):
        self.grid = None
        self.N = 0
        self.voxel_size = 0.0
        self.threshold = 0.5
        self.last_modified_time = 0
        self.mutex = Lock()
        self.display_list = None
        self.display_list_valid = False
        
    def load_file(self, filename):
        if not os.path.exists(filename):
            return False
            
        # Check if file was modified since last load
        mod_time = os.path.getmtime(filename)
        if mod_time <= self.last_modified_time:
            return False
            
        with open(filename, 'rb') as f:
            with self.mutex:
                try:
                    # Read grid size (N) as int32
                    N_bytes = f.read(4)
                    if not N_bytes or len(N_bytes) < 4:
                        return False
                    self.N = struct.unpack('i', N_bytes)[0]
                    
                    # Read voxel size as float32
                    voxel_size_bytes = f.read(4)
                    if not voxel_size_bytes or len(voxel_size_bytes) < 4:
                        return False
                    self.voxel_size = struct.unpack('f', voxel_size_bytes)[0]
                    
                    # Read grid data
                    expected_size = self.N * self.N * self.N * 4  # float32 = 4 bytes
                    grid_bytes = f.read(expected_size)
                    if len(grid_bytes) < expected_size:
                        print(f"Error: Incomplete grid data in {filename}")
                        return False
                        
                    # Convert to numpy array
                    self.grid = np.frombuffer(grid_bytes, dtype=np.float32)
                    self.grid = self.grid.reshape((self.N, self.N, self.N))
                    
                    # Mark display list as invalid to regenerate it
                    self.display_list_valid = False
                    self.last_modified_time = mod_time
                    
                    print(f"Loaded voxel grid: {self.N}x{self.N}x{self.N}, voxel size: {self.voxel_size}")
                    return True
                except Exception as e:
                    print(f"Error loading voxel grid: {e}")
                    return False
    
    def create_display_list(self):
        if self.grid is None:
            return
        
        with self.mutex:
            # Delete previous display list if it exists
            if self.display_list is not None:
                glDeleteLists(self.display_list, 1)
            
            # Create a new display list
            self.display_list = glGenLists(1)
            glNewList(self.display_list, GL_COMPILE)
            
            # Draw voxels
            half_size = self.N * self.voxel_size / 2.0
            for i in range(self.N):
                for j in range(self.N):
                    for k in range(self.N):
                        val = self.grid[i, j, k]
                        if val > self.threshold:
                            # Normalize value for color intensity
                            normalized_val = min(1.0, val / (self.threshold * 5.0))
                            
                            # Position in world space
                            x = i * self.voxel_size - half_size
                            y = j * self.voxel_size - half_size
                            z = k * self.voxel_size - half_size
                            
                            # Draw cube
                            glColor4f(1.0, 0.2, 0.2, normalized_val)
                            glPushMatrix()
                            glTranslatef(x, y, z)
                            
                            # Use cubes for high values, points for lower values
                            if normalized_val > 0.5:
                                glutSolidCube(self.voxel_size * 0.9)
                            else:
                                glPointSize(2.0)
                                glBegin(GL_POINTS)
                                glVertex3f(0, 0, 0)
                                glEnd()
                                
                            glPopMatrix()
            
            glEndList()
            self.display_list_valid = True

# Global variables
voxel_grid = VoxelGrid()
camera = Camera()
monitor_dir = "."
last_file_check = 0
file_check_interval = 1.0  # seconds
width, height = 1200, 800
keys_pressed = set()
running = True

# Keyboard and mouse handlers
def keyboard(key, x, y):
    global keys_pressed
    global running
    
    if key == b'q' or key == b'Q':
        running = False
        glutLeaveMainLoop()
    elif key == b'+' or key == b'=':
        with voxel_grid.mutex:
            voxel_grid.threshold *= 0.8
            voxel_grid.display_list_valid = False
            print(f"Threshold: {voxel_grid.threshold}")
    elif key == b'-' or key == b'_':
        with voxel_grid.mutex:
            voxel_grid.threshold *= 1.2
            voxel_grid.display_list_valid = False
            print(f"Threshold: {voxel_grid.threshold}")
    else:
        keys_pressed.add(key)

def keyboard_up(key, x, y):
    global keys_pressed
    if key in keys_pressed:
        keys_pressed.remove(key)

def mouse_motion(x, y):
    global camera
    
    if camera.first_mouse:
        camera.last_x = x
        camera.last_y = y
        camera.first_mouse = False
        return
    
    # Calculate offset
    x_offset = x - camera.last_x
    y_offset = camera.last_y - y  # Reversed since y-coordinates go from bottom to top
    
    camera.last_x = x
    camera.last_y = y
    
    # Apply sensitivity
    x_offset *= camera.mouse_sensitivity
    y_offset *= camera.mouse_sensitivity
    
    # Update camera angles
    camera.yaw += x_offset
    camera.pitch += y_offset
    
    # Clamp pitch to avoid flipping
    if camera.pitch > 89.0:
        camera.pitch = 89.0
    if camera.pitch < -89.0:
        camera.pitch = -89.0

def reshape(w, h):
    global width, height
    width, height = w, h
    
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, width / height, 1.0, 5000.0)
    glMatrixMode(GL_MODELVIEW)

# File monitoring thread
def monitor_files():
    global running
    
    while running:
        try:
            # Find latest voxel grid file
            voxel_files = glob.glob(os.path.join(monitor_dir, "voxel_grid_*.bin"))
            if voxel_files:
                latest_file = max(voxel_files, key=os.path.getmtime)
                voxel_grid.load_file(latest_file)
                
            time.sleep(0.5)
        except Exception as e:
            print(f"Error in monitor thread: {e}")
            time.sleep(1)

# Process keyboard input and move camera
def process_input():
    # Define movement speed
    speed = camera.speed
    
    # Get forward direction
    yaw_rad = np.radians(camera.yaw)
    pitch_rad = np.radians(camera.pitch)
    
    forward = np.array([
        np.cos(yaw_rad) * np.cos(pitch_rad),
        np.sin(pitch_rad),
        np.sin(yaw_rad) * np.cos(pitch_rad)
    ])
    forward = forward / np.linalg.norm(forward)
    
    # Get right direction
    right = np.cross(forward, np.array([0.0, 1.0, 0.0]))
    right = right / np.linalg.norm(right)
    
    # Get up direction
    up = np.cross(right, forward)
    
    # Process keys
    if b'w' in keys_pressed:
        camera.pos += forward * speed
    if b's' in keys_pressed:
        camera.pos -= forward * speed
    if b'a' in keys_pressed:
        camera.pos -= right * speed
    if b'd' in keys_pressed:
        camera.pos += right * speed
    if b' ' in keys_pressed:  # Space for up
        camera.pos += up * speed
    if b'c' in keys_pressed:  # 'c' for down
        camera.pos -= up * speed

# Draw axes and grid
def draw_grid():
    # Draw axes
    glLineWidth(3.0)
    
    # X axis (red)
    glColor3f(1.0, 0.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(100.0, 0.0, 0.0)
    glEnd()
    
    # Y axis (green)
    glColor3f(0.0, 1.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 100.0, 0.0)
    glEnd()
    
    # Z axis (blue)
    glColor3f(0.0, 0.0, 1.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 0.0, 100.0)
    glEnd()
    
    # Draw grid on XZ plane
    glLineWidth(1.0)
    glColor3f(0.5, 0.5, 0.5)
    
    grid_size = 1000
    step = 100
    
    glBegin(GL_LINES)
    for i in range(-grid_size, grid_size + 1, step):
        glVertex3f(i, 0.0, -grid_size)
        glVertex3f(i, 0.0, grid_size)
        glVertex3f(-grid_size, 0.0, i)
        glVertex3f(grid_size, 0.0, i)
    glEnd()

# OpenGL display function
def display():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    
    # Process keyboard input
    process_input()
    
    # Update view based on camera
    camera.update_view()
    
    # Draw reference grid
    draw_grid()
    
    # Draw voxel grid
    with voxel_grid.mutex:
        if voxel_grid.grid is not None:
            if not voxel_grid.display_list_valid:
                voxel_grid.create_display_list()
            
            if voxel_grid.display_list_valid:
                # Enable blending for transparency
                glEnable(GL_BLEND)
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
                
                # Draw the display list
                glCallList(voxel_grid.display_list)
                
                # Disable blending
                glDisable(GL_BLEND)
    
    glutSwapBuffers()

def idle():
    glutPostRedisplay()

# Main function
def main():
    global monitor_dir
    
    # Parse command line arguments
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <voxel_grid_directory> [threshold]")
        return 1
    
    monitor_dir = sys.argv[1]
    if not os.path.isdir(monitor_dir):
        print(f"Error: {monitor_dir} is not a valid directory")
        return 1
    
    if len(sys.argv) >= 3:
        voxel_grid.threshold = float(sys.argv[2])
    
    # Initialize GLUT
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(width, height)
    glutCreateWindow("3D Voxel Grid Visualizer")
    
    # Set callbacks
    glutDisplayFunc(display)
    glutReshapeFunc(reshape)
    glutKeyboardFunc(keyboard)
    glutKeyboardUpFunc(keyboard_up)
    glutPassiveMotionFunc(mouse_motion)
    glutIdleFunc(idle)
    
    # Set OpenGL state
    glClearColor(0.0, 0.0, 0.0, 1.0)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_POINT_SMOOTH)
    
    # Start file monitoring thread
    monitor_thread = Thread(target=monitor_files)
    monitor_thread.daemon = True
    monitor_thread.start()
    
    print("3D Voxel Grid Visualizer")
    print("------------------------")
    print("Controls:")
    print("  Mouse: Look around")
    print("  W/A/S/D: Move camera")
    print("  Space/C: Move up/down")
    print("  +/-: Adjust threshold")
    print("  Q: Quit")
    
    # Start main loop
    glutMainLoop()
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 