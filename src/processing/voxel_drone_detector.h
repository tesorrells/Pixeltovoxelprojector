#pragma once

#include <vector>
#include <string>
#include <chrono>
#include <mutex>
#include <optional>
#include <vector>
#include <map>
#include <memory>
#include <cmath>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <atomic>


// Structure to represent a 3D point with voxel indices
struct VoxelPoint {
    int x, y, z;       // Voxel indices
    float value;       // Voxel value (intensity)

    // Comparison operator for sets/maps
    bool operator<(const VoxelPoint& other) const;

    // Equality operator
    bool operator==(const VoxelPoint& other) const;
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
    VoxelDrone(int _id);

    // Update the drone's position based on its voxels
    void updatePosition(float voxel_size, const float grid_min[3]);

    // Calculate velocity based on previous position and time difference
    void updateVelocity(float prev_x, float prev_y, float prev_z,
                       const std::chrono::system_clock::time_point& prev_time);

    // Calculate speed in meters per second
    float getSpeed() const;

    // Calculate altitude in meters
    float getAltitude() const;
};

// Struct to represent a detected drone with its properties (Simplified version for external reporting)
// Note: This struct might be different from the internal VoxelDrone struct
struct DroneDetection {
    int id;                     // Unique identifier
    std::string uuid;           // UUID matching VoxelDrone
    float x, y, z;              // Position in real-world coordinates (meters)
    float vx, vy, vz;           // Velocity in m/s
    bool velocity_valid;        // Is velocity estimate valid?
    float uncertainty;          // Position uncertainty (meters)
    float confidence;           // Detection confidence (e.g., based on intensity/size)
    std::chrono::system_clock::time_point detection_time; // Time of detection

    // Default constructor
    DroneDetection();

    // Constructor from VoxelDrone
    explicit DroneDetection(const VoxelDrone& drone);
};


// Struct to represent a voxel position in 3D space
struct VoxelPosition {
    int x, y, z;

    // Constructor
    VoxelPosition(int x, int y, int z);

    // Equality operator for using in unordered sets/maps
    bool operator==(const VoxelPosition& other) const;
};

// Hash function for VoxelPosition
namespace std {
    template <>
    struct hash<VoxelPosition> {
        size_t operator()(const VoxelPosition& pos) const;
    };
}

class VoxelDroneDetector {
public:
    // Constructor
    VoxelDroneDetector(
        int initial_grid_size_x = 100,
        int initial_grid_size_y = 100,
        int initial_grid_size_z = 50,
        float threshold = 0.5f,
        float cluster_dist = 1.5f,
        int min_voxels = 5,
        int max_voxels = 1000,
        int inactive_timeout_seconds = 30);

    // Set grid parameters dynamically
    void setGridParameters(int size_x, int size_y, int size_z, float voxel_size_meters, const float min_coords[3]);
    void setVoxelSize(float size_m); // Convenience setter
    void setClusterParameters(float min_cluster_size, float max_cluster_size, float tolerance_voxels); // Convenience setter for voxel-based params

    // Process a new voxel grid - This is the main entry point for detection
    // Assumes voxel_grid is a flat array of size grid_size[0]*grid_size[1]*grid_size[2]
    void processVoxelGrid(const std::vector<float>& voxel_grid);
    // Overload that uses internally maintained grid (set via setVoxelValue etc.)
    void processVoxelGrid();

    // Simulate setting a voxel value (useful for testing)
    void setVoxelValue(int vx, int vy, int vz, float value);
    void clearVoxelGrid(); // Clears internal representation if needed

    // Get the list of currently tracked drones
    std::vector<DroneDetection> getDetectedDrones() const;
    std::optional<DroneDetection> getDroneById(int id) const;
    size_t getDroneCount() const;

    // Coordinate conversions (if needed externally)
    void voxelToWorld(int voxel_x, int voxel_y, int voxel_z, float& world_x, float& world_y, float& world_z) const;
    void worldToVoxel(float world_x, float world_y, float world_z, int& voxel_x, int& voxel_y, int& voxel_z) const;


private:
    // Internal processing methods
    void updateDetections(const std::vector<VoxelPoint>& potential_voxels);
    std::vector<std::vector<VoxelPoint>> clusterVoxels(const std::vector<VoxelPoint>& points) const;
    void assignVoxelsToDrones(const std::vector<std::vector<VoxelPoint>>& clusters);
    void updateDroneProperties();
    void removeInactiveDrones();

    // Member variables
    std::vector<VoxelDrone> drones_;
    int next_id_;
    mutable std::mutex drones_mutex_; // Mutex for accessing drones_

    // Configuration
    float detection_threshold_;       // Minimum voxel value to consider for detection
    float clustering_distance_sq_;    // Maximum squared distance (in voxels) for clustering
    int min_voxels_per_drone_;        // Minimum number of voxels to form a drone
    int max_voxels_per_drone_;        // Maximum number of voxels per drone (performance limit)
    std::chrono::seconds max_inactive_time_; // Max time before removing inactive drone

    // Current voxel grid parameters
    int grid_size_[3];               // Dimensions (x,y,z)
    float voxel_size_;               // Size in meters
    float grid_min_[3];              // World coordinates of grid corner (0,0,0)

    // Temporary storage for processing
    std::vector<float> current_voxel_grid_; // Internal copy or reference? Decide based on processVoxelGrid

    // ID counter for new detections
    int next_drone_id_;

    // Mutex for thread safety on internal state if needed (e.g., grid parameters)
    mutable std::mutex detector_mutex_; // Mutex for grid parameters etc.

}; 