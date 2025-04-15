// main.cpp
//
// Main application for the 3D Voxel-based Drone Detection System
// This application:
//   1) Initializes the voxel grid and detector
//   2) Sets up CoT messaging for ATAK integration
//   3) Processes voxel data to detect and track drones
//   4) Sends drone tracking data to ATAK

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <csignal>
#include <cmath>
#include <fstream>
#include <sstream>

// Include our components (Headers)
#include "processing/voxel_drone_detector.h"
#include "cot/cot_message_generator.h"
#include "cot/cot_network_handler.h" // Needed for sending CoT

// Flag for controlling program execution
volatile sig_atomic_t running = 1;

// Signal handler for graceful shutdown
void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received. Shutting down..." << std::endl;
    running = 0;
}

// Configuration structure
struct Config {
    // Voxel grid parameters
    int grid_size_x = 100;
    int grid_size_y = 100;
    int grid_size_z = 50;
    float voxel_size = 0.5f;  // Size of each voxel in meters
    
    // Detection parameters
    float min_cluster_size = 5.0f;  // Minimum size for a cluster to be considered
    float max_cluster_size = 100.0f; // Maximum size for a cluster
    float cluster_tolerance = 1.0f;  // Distance tolerance for clustering
    
    // Position origin (for converting to lat/lon)
    double origin_lat = 38.8895;
    double origin_lon = -77.0352;
    
    // ATAK CoT server parameters
    std::string atak_server_ip = "127.0.0.1";
    int atak_server_port = 8087;
    int update_interval = 1;  // In seconds
    
    // Test mode flag (for simulating drones)
    bool test_mode = false;
    
    // Load configuration from file
    bool loadFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Could not open config file: " << filename << std::endl;
            return false;
        }
        
        std::string line;
        while (std::getline(file, line)) {
            // Skip comments and empty lines
            if (line.empty() || line[0] == '#') {
                continue;
            }
            
            std::istringstream iss(line);
            std::string key;
            if (std::getline(iss, key, '=')) {
                std::string value;
                if (std::getline(iss, value)) {
                    // Remove whitespace
                    key.erase(0, key.find_first_not_of(" \t"));
                    key.erase(key.find_last_not_of(" \t") + 1);
                    value.erase(0, value.find_first_not_of(" \t"));
                    value.erase(value.find_last_not_of(" \t") + 1);
                    
                    // Process key/value pairs
                    if (key == "grid_size_x") grid_size_x = std::stoi(value);
                    else if (key == "grid_size_y") grid_size_y = std::stoi(value);
                    else if (key == "grid_size_z") grid_size_z = std::stoi(value);
                    else if (key == "voxel_size") voxel_size = std::stof(value);
                    else if (key == "min_cluster_size") min_cluster_size = std::stof(value);
                    else if (key == "max_cluster_size") max_cluster_size = std::stof(value);
                    else if (key == "cluster_tolerance") cluster_tolerance = std::stof(value);
                    else if (key == "origin_lat") origin_lat = std::stod(value);
                    else if (key == "origin_lon") origin_lon = std::stod(value);
                    else if (key == "atak_server_ip") atak_server_ip = value;
                    else if (key == "atak_server_port") atak_server_port = std::stoi(value);
                    else if (key == "update_interval") update_interval = std::stoi(value);
                    else if (key == "test_mode") test_mode = (value == "true" || value == "1");
                }
            }
        }
        
        file.close();
        return true;
    }
    
    // Print the current configuration
    void printConfig() const {
        std::cout << "=== Configuration ===" << std::endl;
        std::cout << "Voxel Grid: " << grid_size_x << "x" << grid_size_y << "x" << grid_size_z 
                  << " (voxel size: " << voxel_size << "m)" << std::endl;
        std::cout << "Cluster Parameters: min=" << min_cluster_size << ", max=" << max_cluster_size 
                  << ", tolerance=" << cluster_tolerance << std::endl;
        std::cout << "Position Origin: " << origin_lat << ", " << origin_lon << std::endl;
        std::cout << "ATAK Server: " << atak_server_ip << ":" << atak_server_port 
                  << " (update interval: " << update_interval << "s)" << std::endl;
        std::cout << "Test Mode: " << (test_mode ? "On" : "Off") << std::endl;
    }
};

// Simulate a drone moving through the voxel space (for testing)
void simulateDrone(VoxelDroneDetector& detector, int grid_size_x, int grid_size_y, int grid_size_z, float voxel_size) {
    static float t = 0.0f;
    t += 0.1f;
    
    // Clear previous voxel grid
    detector.clearVoxelGrid();
    
    // Center of the grid
    float center_x = grid_size_x / 2.0f;
    float center_y = grid_size_y / 2.0f;
    float center_z = grid_size_z / 2.0f;
    
    // Drone position (circular path)
    float radius = std::min(grid_size_x, grid_size_y) / 4.0f;
    float drone_x = center_x + radius * cos(t);
    float drone_y = center_y + radius * sin(t);
    float drone_z = center_z + 5.0f * sin(t / 2.0f);
    
    // Create a drone-shaped cluster of voxels
    for (int dx = -2; dx <= 2; dx++) {
        for (int dy = -2; dy <= 2; dy++) {
            for (int dz = -1; dz <= 1; dz++) {
                // Calculate distance from drone center
                float distance = sqrt(dx*dx + dy*dy + dz*dz);
                
                // Skip if too far (to create a roughly drone-shaped object)
                if (distance > 2.5f) continue;
                
                // Calculate voxel position
                int vx = static_cast<int>(drone_x + dx);
                int vy = static_cast<int>(drone_y + dy);
                int vz = static_cast<int>(drone_z + dz);
                
                // Ensure we're within grid bounds
                if (vx >= 0 && vx < grid_size_x && 
                    vy >= 0 && vy < grid_size_y && 
                    vz >= 0 && vz < grid_size_z) {
                    
                    // Intensity falls off with distance from center
                    float intensity = 1.0f - (distance / 3.0f);
                    if (intensity < 0.1f) intensity = 0.1f;
                    
                    // Set voxel value
                    detector.setVoxelValue(vx, vy, vz, intensity);
                }
            }
        }
    }
    
    // Process the grid to detect the drone
    detector.processVoxelGrid();
}

// Main application entry point
int main(int argc, char** argv) {
    // Register signal handler for clean shutdown
    signal(SIGINT, signalHandler);
    
    // Load configuration
    Config config;
    std::string config_file = "drone_detector.conf";
    
    // Check for config file path from command line
    if (argc > 1) {
        config_file = argv[1];
    }
    
    // Try to load config, use defaults if not available
    if (!config.loadFromFile(config_file)) {
        std::cout << "Using default configuration" << std::endl;
    }
    
    // Print current configuration
    config.printConfig();
    
    // Create the voxel drone detector
    VoxelDroneDetector detector(config.grid_size_x, config.grid_size_y, config.grid_size_z);
    detector.setVoxelSize(config.voxel_size);
    detector.setClusterParameters(config.min_cluster_size, config.max_cluster_size, config.cluster_tolerance);
    
    // Create the CoT message generator
    CoTMessageGenerator cot_generator("drone-detector-01");
    cot_generator.setSensorLocation(config.origin_lat, config.origin_lon, 0.0);
    
    // Create the CoT Network Handler
    CoTNetworkHandler cot_sender(config.atak_server_ip, config.atak_server_port);
    
    // Start the CoT sender thread
    if (!cot_sender.start()) {
        std::cerr << "Failed to start CoT network handler. Check connection to ATAK server: "
                  << config.atak_server_ip << ":" << config.atak_server_port << std::endl;
        // Decide if we should exit or continue without sending
        // return 1; // Example: Exit if sender fails to start
    }
    
    std::cout << "Drone detection system started." << std::endl;
    std::cout << "Press Ctrl+C to exit." << std::endl;
    
    // Main processing loop
    auto last_cot_send_time = std::chrono::steady_clock::now();
    while (running) {
        if (config.test_mode) {
            // In test mode, simulate a moving drone
            simulateDrone(detector, config.grid_size_x, config.grid_size_y, config.grid_size_z, config.voxel_size);
        } else {
            // In normal mode, we'd receive voxel data from an external source
            // This could be from a sensor system, network connection, etc.
            // For now, just process whatever is in the voxel grid
            detector.processVoxelGrid();
        }
        
        // Report detected drones
        auto drones = detector.getDetectedDrones();
        std::cout << "\rDetected drones: " << drones.size() << "   " << std::flush;
        
        // Generate and send CoT messages periodically
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_cot_send_time).count();
        
        if (elapsed_ms >= config.update_interval * 1000 && !drones.empty()) {
            std::vector<Detection> detections_for_cot; // Convert VoxelDroneDetector::DroneDetection to CoTMessageGenerator::Detection
            detections_for_cot.reserve(drones.size());
            for(const auto& drone : drones) {
                Detection det;
                det.x = drone.x; // Assuming drone.x/y/z are world coordinates
                det.y = drone.y;
                det.z = drone.z;
                det.confidence = drone.confidence;
                det.classification = 1; // Assuming classification 1 = drone
                det.uid = drone.uuid; // Pass the UUID
                det.timestamp = drone.detection_time;
                detections_for_cot.push_back(det);
            }

            if (!detections_for_cot.empty()) {
                std::vector<std::string> cot_messages = cot_generator.generateCoTMessages(detections_for_cot);
                cot_sender.queueMessages(cot_messages);
                std::cout << " [Sent " << cot_messages.size() << " CoT]" << std::flush;
            }
            last_cot_send_time = current_time;
        }
        
        // Short sleep to prevent CPU overuse
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << std::endl << "Shutting down..." << std::endl;
    
    // Stop the CoT message generator
    cot_sender.stop();
    
    std::cout << "Drone detection system stopped." << std::endl;
    
    return 0;
} 