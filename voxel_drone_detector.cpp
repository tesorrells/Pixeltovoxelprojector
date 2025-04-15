// voxel_drone_detector.cpp
//
// Drone detection from 3D voxel grid
// This module:
//   1) Identifies potential drone locations from voxel grid
//   2) Tracks drone instances across time (assigns IDs)
//   3) Calculates position and velocity

#include <vector>
#include <map>
#include <memory>
#include <string>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <optional>
#include <random>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <atomic>

// Structure to represent a 3D point with voxel indices
struct VoxelPoint {
    int x, y, z;       // Voxel indices
    float value;       // Voxel value (intensity)
    
    // Comparison operator for sets/maps
    bool operator<(const VoxelPoint& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }
    
    // Equality operator
    bool operator==(const VoxelPoint& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// Structure to represent a detected drone in voxel space
struct VoxelDrone {
    int id;                                // Unique identifier
    std::string uuid;                      // UUID for CoT
    std::vector<VoxelPoint> voxels;        // Voxels belonging to this drone
    float total_intensity;                 // Sum of all voxel intensities
    float max_intensity;                   // Maximum voxel intensity
    
    // Center point (centroid) in voxel space
    float center_x, center_y, center_z;
    
    // Real-world position (meters)
    float position_x, position_y, position_z;
    float uncertainty;                     // Position uncertainty (meters)
    
    // Velocity estimate (meters/second)
    float velocity_x, velocity_y, velocity_z;
    bool velocity_valid;                   // Whether velocity estimate is valid
    
    // Timestamp when last detected
    std::chrono::system_clock::time_point last_seen;
    
    // Constructor
    VoxelDrone(int _id) : id(_id), 
                         total_intensity(0.0f),
                         max_intensity(0.0f),
                         center_x(0.0f), center_y(0.0f), center_z(0.0f),
                         position_x(0.0f), position_y(0.0f), position_z(0.0f),
                         uncertainty(10.0f),
                         velocity_x(0.0f), velocity_y(0.0f), velocity_z(0.0f),
                         velocity_valid(false),
                         last_seen(std::chrono::system_clock::now())
    {
        // Generate UUID for CoT
        auto now = std::chrono::system_clock::now();
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
            
        // Generate random string component for more uniqueness
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 15);
        const char* hex_chars = "0123456789abcdef";
        std::string random_hex;
        for (int i = 0; i < 8; ++i) {
            random_hex += hex_chars[dis(gen)];
        }
        
        uuid = "drone-" + std::to_string(_id) + "-" + std::to_string(millis) + "-" + random_hex;
    }
    
    // Update the drone's position based on its voxels
    void updatePosition(float voxel_size, const float grid_min[3]) {
        if (voxels.empty()) return;
        
        // Calculate centroid in voxel space
        float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
        float sum_weights = 0.0f;
        
        for (const auto& voxel : voxels) {
            float weight = voxel.value;
            sum_x += voxel.x * weight;
            sum_y += voxel.y * weight;
            sum_z += voxel.z * weight;
            sum_weights += weight;
        }
        
        if (sum_weights > 0) {
            center_x = sum_x / sum_weights;
            center_y = sum_y / sum_weights;
            center_z = sum_z / sum_weights;
            
            // Convert to real-world coordinates
            position_x = grid_min[0] + center_x * voxel_size;
            position_y = grid_min[1] + center_y * voxel_size;
            position_z = grid_min[2] + center_z * voxel_size;
            
            // Calculate uncertainty based on spread of points
            float var_x = 0.0f, var_y = 0.0f, var_z = 0.0f;
            for (const auto& voxel : voxels) {
                float weight = voxel.value / sum_weights;
                var_x += weight * std::pow(voxel.x - center_x, 2);
                var_y += weight * std::pow(voxel.y - center_y, 2);
                var_z += weight * std::pow(voxel.z - center_z, 2);
            }
            
            // Average standard deviation in real-world units
            uncertainty = voxel_size * std::sqrt((var_x + var_y + var_z) / 3.0f);
        }
    }
    
    // Calculate velocity based on previous position and time difference
    void updateVelocity(float prev_x, float prev_y, float prev_z, 
                       const std::chrono::system_clock::time_point& prev_time) {
        auto now = std::chrono::system_clock::now();
        double dt = std::chrono::duration<double>(now - prev_time).count();
        
        if (dt > 0.001) {  // Avoid division by very small numbers
            velocity_x = (position_x - prev_x) / dt;
            velocity_y = (position_y - prev_y) / dt;
            velocity_z = (position_z - prev_z) / dt;
            velocity_valid = true;
        }
    }
    
    // Calculate speed in meters per second
    float getSpeed() const {
        if (!velocity_valid) return 0.0f;
        return std::sqrt(velocity_x*velocity_x + velocity_y*velocity_y + velocity_z*velocity_z);
    }
    
    // Calculate altitude in meters
    float getAltitude() const {
        return position_z;
    }
};

// Struct to represent a detected drone with its properties
struct DroneDetection {
    int id;                     // Unique identifier
    float x, y, z;              // Position in voxel grid coordinates
    float vx, vy, vz;           // Velocity in voxel grid coordinates (units/sec)
    float size;                 // Size of the drone (in voxel units)
    float confidence;           // Detection confidence (0.0-1.0)
    std::chrono::time_point<std::chrono::system_clock> detection_time; // Time of detection
    
    // Last known positions for velocity calculation
    float last_x, last_y, last_z;
    std::chrono::time_point<std::chrono::system_clock> last_detection_time;
    
    // Default constructor
    DroneDetection()
        : id(-1), x(0), y(0), z(0), vx(0), vy(0), vz(0), size(0), confidence(0),
          detection_time(std::chrono::system_clock::now()),
          last_x(0), last_y(0), last_z(0), last_detection_time(detection_time) {}
    
    // Constructor with position
    DroneDetection(int id, float x, float y, float z, float size, float confidence)
        : id(id), x(x), y(y), z(z), vx(0), vy(0), vz(0), size(size), confidence(confidence),
          detection_time(std::chrono::system_clock::now()),
          last_x(x), last_y(y), last_z(z), last_detection_time(detection_time) {}
};

// Struct to represent a voxel position in 3D space
struct VoxelPosition {
    int x, y, z;
    
    // Constructor
    VoxelPosition(int x, int y, int z) : x(x), y(y), z(z) {}
    
    // Equality operator for using in unordered sets/maps
    bool operator==(const VoxelPosition& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// Hash function for VoxelPosition
namespace std {
    template <>
    struct hash<VoxelPosition> {
        size_t operator()(const VoxelPosition& pos) const {
            // Combine the hash of x, y, z using a simple hash combiner
            size_t h1 = std::hash<int>{}(pos.x);
            size_t h2 = std::hash<int>{}(pos.y);
            size_t h3 = std::hash<int>{}(pos.z);
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };
}

class VoxelDroneDetector {
private:
    std::vector<VoxelDrone> drones;
    int next_id;
    std::mutex drones_mutex;
    
    // Configuration
    float detection_threshold;       // Minimum voxel value to consider for detection
    float clustering_distance;       // Maximum distance (in voxels) to consider as part of the same drone
    int min_voxels_per_drone;        // Minimum number of voxels to form a drone
    int max_voxels_per_drone;        // Maximum number of voxels to consider per drone (limit for performance)
    std::chrono::seconds max_inactive_time;  // Maximum time a drone can be inactive before removing
    
    // Current voxel grid parameters
    int grid_size[3];               // Dimensions of the voxel grid (N x N x N)
    float voxel_size;               // Size of each voxel in meters
    float grid_min[3];              // Minimum coordinates of the grid in real-world space
    
    // Detected drones
    std::vector<DroneDetection> detected_drones;
    
    // ID counter for new detections
    int next_drone_id;
    
    // Mutex for thread safety
    std::mutex detector_mutex;
    
    // Find connected components in the voxel grid
    std::vector<std::vector<VoxelPosition>> findConnectedComponents() {
        std::vector<std::vector<VoxelPosition>> clusters;
        
        // Track visited voxels
        std::vector<std::vector<std::vector<bool>>> visited(
            grid_size[0],
            std::vector<std::vector<bool>>(
                grid_size[1],
                std::vector<bool>(grid_size[2], false)
            )
        );
        
        // Threshold for considering a voxel part of an object
        const float intensity_threshold = 0.2f;
        
        // Neighbor offsets (6-connected: front, back, left, right, up, down)
        const int neighbor_offsets[6][3] = {
            {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, 
            {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
        };
        
        // Scan through all voxels
        for (int x = 0; x < grid_size[0]; ++x) {
            for (int y = 0; y < grid_size[1]; ++y) {
                for (int z = 0; z < grid_size[2]; ++z) {
                    // Skip if already visited or below threshold
                    if (visited[x][y][z] || voxel_grid[x][y][z] < intensity_threshold) {
                        continue;
                    }
                    
                    // Start a new cluster
                    std::vector<VoxelPosition> cluster;
                    std::queue<VoxelPosition> queue;
                    
                    // Add the first voxel to the queue
                    VoxelPosition pos(x, y, z);
                    queue.push(pos);
                    visited[x][y][z] = true;
                    
                    // Process the connected component using breadth-first search
                    while (!queue.empty()) {
                        VoxelPosition current = queue.front();
                        queue.pop();
                        
                        // Add to the current cluster
                        cluster.push_back(current);
                        
                        // Check neighbors
                        for (int i = 0; i < 6; ++i) {
                            int nx = current.x + neighbor_offsets[i][0];
                            int ny = current.y + neighbor_offsets[i][1];
                            int nz = current.z + neighbor_offsets[i][2];
                            
                            // Skip if out of bounds
                            if (nx < 0 || nx >= grid_size[0] || 
                                ny < 0 || ny >= grid_size[1] || 
                                nz < 0 || nz >= grid_size[2]) {
                                continue;
                            }
                            
                            // Skip if already visited or below threshold
                            if (visited[nx][ny][nz] || voxel_grid[nx][ny][nz] < intensity_threshold) {
                                continue;
                            }
                            
                            // Add to queue and mark as visited
                            queue.push(VoxelPosition(nx, ny, nz));
                            visited[nx][ny][nz] = true;
                        }
                    }
                    
                    // If the cluster is large enough, add it to our list
                    if (cluster.size() >= min_voxels_per_drone && cluster.size() <= max_voxels_per_drone) {
                        clusters.push_back(cluster);
                    }
                }
            }
        }
        
        return clusters;
    }
    
    // Calculate the properties of each cluster and create DroneDetection objects
    std::vector<DroneDetection> analyzeClusters(const std::vector<std::vector<VoxelPosition>>& clusters) {
        std::vector<DroneDetection> new_detections;
        
        for (const auto& cluster : clusters) {
            if (cluster.empty()) {
                continue;
            }
            
            // Calculate centroid
            float sum_x = 0, sum_y = 0, sum_z = 0;
            float total_intensity = 0;
            
            for (const auto& voxel : cluster) {
                float intensity = voxel_grid[voxel.x][voxel.y][voxel.z];
                sum_x += voxel.x * intensity;
                sum_y += voxel.y * intensity;
                sum_z += voxel.z * intensity;
                total_intensity += intensity;
            }
            
            // Compute weighted centroid
            float centroid_x = sum_x / total_intensity;
            float centroid_y = sum_y / total_intensity;
            float centroid_z = sum_z / total_intensity;
            
            // Calculate size (approximate diameter)
            float max_dist_sq = 0;
            for (const auto& voxel : cluster) {
                float dx = voxel.x - centroid_x;
                float dy = voxel.y - centroid_y;
                float dz = voxel.z - centroid_z;
                float dist_sq = dx*dx + dy*dy + dz*dz;
                max_dist_sq = std::max(max_dist_sq, dist_sq);
            }
            float size = 2 * std::sqrt(max_dist_sq);  // Diameter
            
            // Calculate confidence based on cluster size and intensity
            float avg_intensity = total_intensity / cluster.size();
            float confidence = avg_intensity * std::min(1.0f, cluster.size() / (min_voxels_per_drone * 3.0f));
            
            // Create a new detection
            new_detections.emplace_back(
                next_drone_id++,  // Assign a unique ID
                centroid_x,
                centroid_y,
                centroid_z,
                size,
                confidence
            );
        }
        
        return new_detections;
    }
    
    // Associate new detections with existing ones based on proximity
    void updateDetections(const std::vector<DroneDetection>& new_detections) {
        std::vector<DroneDetection> updated_detections;
        std::vector<bool> new_detection_matched(new_detections.size(), false);
        
        auto now = std::chrono::system_clock::now();
        
        // First, try to match existing detections with new ones
        for (auto& existing : detected_drones) {
            bool matched = false;
            int best_match_idx = -1;
            float best_match_dist = std::numeric_limits<float>::max();
            
            // Find the closest new detection
            for (size_t i = 0; i < new_detections.size(); ++i) {
                if (new_detection_matched[i]) {
                    continue;  // Skip if already matched
                }
                
                const auto& new_det = new_detections[i];
                
                // Calculate distance
                float dx = existing.x - new_det.x;
                float dy = existing.y - new_det.y;
                float dz = existing.z - new_det.z;
                float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                
                // If within tolerance and better than current best match
                if (dist < clustering_distance && dist < best_match_dist) {
                    best_match_dist = dist;
                    best_match_idx = i;
                }
            }
            
            // If we found a match
            if (best_match_idx >= 0) {
                const auto& match = new_detections[best_match_idx];
                
                // Save previous position for velocity calculation
                existing.last_x = existing.x;
                existing.last_y = existing.y;
                existing.last_z = existing.z;
                existing.last_detection_time = existing.detection_time;
                
                // Update position and properties
                existing.x = match.x;
                existing.y = match.y;
                existing.z = match.z;
                existing.size = match.size;
                existing.confidence = match.confidence;
                existing.detection_time = match.detection_time;
                
                // Calculate velocity
                auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                    existing.detection_time - existing.last_detection_time).count() / 1000.0f;
                
                if (time_diff > 0.01f) { // Avoid division by very small numbers
                    existing.vx = (existing.x - existing.last_x) / time_diff;
                    existing.vy = (existing.y - existing.last_y) / time_diff;
                    existing.vz = (existing.z - existing.last_z) / time_diff;
                }
                
                // Mark as matched
                new_detection_matched[best_match_idx] = true;
                matched = true;
                
                // Add to updated detections
                updated_detections.push_back(existing);
            }
            else {
                // Detection lost - we might want to keep it for a few frames
                // For now, we just discard it
            }
        }
        
        // Add unmatched new detections
        for (size_t i = 0; i < new_detections.size(); ++i) {
            if (!new_detection_matched[i]) {
                updated_detections.push_back(new_detections[i]);
            }
        }
        
        // Update the detected drones list
        detected_drones = updated_detections;
    }
    
    // Remove inactive drones
    void removeInactiveDrones() {
        auto now = std::chrono::system_clock::now();
        
        drones.erase(
            std::remove_if(drones.begin(), drones.end(),
                [this, now](const VoxelDrone& drone) {
                    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                        now - drone.last_seen);
                    return elapsed > max_inactive_time;
                }),
            drones.end());
    }

public:
    VoxelDroneDetector(
        float threshold = 0.5f,
        float cluster_dist = 1.5f,
        int min_voxels = 5,
        int max_voxels = 1000,
        int inactive_timeout_seconds = 30)
        : next_id(1),
          detection_threshold(threshold),
          clustering_distance(cluster_dist),
          min_voxels_per_drone(min_voxels),
          max_voxels_per_drone(max_voxels),
          max_inactive_time(inactive_timeout_seconds),
          next_drone_id(1)
    {
        // Initialize grid parameters with defaults
        grid_size[0] = grid_size[1] = grid_size[2] = 0;
        voxel_size = 0.0f;
        grid_min[0] = grid_min[1] = grid_min[2] = 0.0f;
    }
    
    // Update voxel grid metadata
    void setGridParameters(int N, float voxel_size_meters, const float min_coords[3]) {
        grid_size[0] = grid_size[1] = grid_size[2] = N;
        voxel_size = voxel_size_meters;
        grid_min[0] = min_coords[0];
        grid_min[1] = min_coords[1];
        grid_min[2] = min_coords[2];
    }
    
    // Process voxel grid to detect and track drones
    void processVoxelGrid(const std::vector<float>& voxel_grid, int N) {
        std::lock_guard<std::mutex> lock(drones_mutex);
        
        // Set grid size if changed
        if (grid_size[0] != N) {
            grid_size[0] = grid_size[1] = grid_size[2] = N;
        }
        
        // Find connected components in the voxel grid
        auto clusters = findConnectedComponents();
        
        // Analyze clusters to create detections
        auto new_detections = analyzeClusters(clusters);
        
        // Update existing detections with new ones
        updateDetections(new_detections);
        
        // Clean up inactive drones
        removeInactiveDrones();
    }
    
    // Get the current list of detected drones
    std::vector<DroneDetection> getDetectedDrones() const {
        std::lock_guard<std::mutex> lock(detector_mutex);
        return detected_drones;
    }
    
    // Get a specific drone by ID
    std::optional<DroneDetection> getDroneById(int id) const {
        std::lock_guard<std::mutex> lock(detector_mutex);
        
        for (const auto& drone : detected_drones) {
            if (drone.id == id) {
                return drone;
            }
        }
        
        return std::nullopt;
    }
    
    // Get the number of currently tracked drones
    size_t getDroneCount() const {
        std::lock_guard<std::mutex> lock(detector_mutex);
        return detected_drones.size();
    }
    
    // Convert from voxel coordinates to world coordinates (meters)
    void voxelToWorld(float voxel_x, float voxel_y, float voxel_z,
                      float& world_x, float& world_y, float& world_z) const {
        // Simple scaling based on voxel size
        world_x = voxel_x * voxel_size;
        world_y = voxel_y * voxel_size;
        world_z = voxel_z * voxel_size;
    }
    
    // Convert from world coordinates (meters) to voxel coordinates
    void worldToVoxel(float world_x, float world_y, float world_z,
                      int& voxel_x, int& voxel_y, int& voxel_z) const {
        // Simple scaling based on voxel size
        voxel_x = static_cast<int>(world_x / voxel_size);
        voxel_y = static_cast<int>(world_y / voxel_size);
        voxel_z = static_cast<int>(world_z / voxel_size);
    }
}; 