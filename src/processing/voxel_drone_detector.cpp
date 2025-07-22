// voxel_drone_detector.cpp
//
// Drone detection from 3D voxel grid
// Implementation file

#include "processing/voxel_drone_detector.h"
#include <iostream> // For std::cerr, std::cout
#include <random>   // For UUID generation
#include <queue>    // For BFS in clustering
#include <set>      // For visited set in clustering
#include <algorithm> // For std::min, std::max, std::sqrt
#include <cmath>    // For std::pow, std::fabs

// VoxelPoint Implementation
bool VoxelPoint::operator<(const VoxelPoint& other) const {
    if (x != other.x) return x < other.x;
    if (y != other.y) return y < other.y;
    return z < other.z;
}

bool VoxelPoint::operator==(const VoxelPoint& other) const {
    return x == other.x && y == other.y && z == other.z;
}

// VoxelDrone Implementation
VoxelDrone::VoxelDrone(int _id) : id(_id),
                               total_intensity(0.0f),
                               max_intensity(0.0f),
                               center_x(0.0f), center_y(0.0f), center_z(0.0f),
                               position_x(0.0f), position_y(0.0f), position_z(0.0f),
                               uncertainty(10.0f), // Default high uncertainty
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

void VoxelDrone::updatePosition(float voxel_size, const float grid_min[3]) {
    if (voxels.empty()) return;

    // Calculate intensity-weighted centroid in voxel space
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    float sum_weights = 0.0f;
    max_intensity = 0.0f;
    total_intensity = 0.0f;

    for (const auto& voxel : voxels) {
        float weight = voxel.value; // Use intensity as weight
        sum_x += voxel.x * weight;
        sum_y += voxel.y * weight;
        sum_z += voxel.z * weight;
        sum_weights += weight;
        if(voxel.value > max_intensity) max_intensity = voxel.value;
        total_intensity += voxel.value;
    }

    if (sum_weights > 1e-6f) { // Avoid division by zero
        center_x = sum_x / sum_weights;
        center_y = sum_y / sum_weights;
        center_z = sum_z / sum_weights;

        // Convert centroid to real-world coordinates (origin is grid_min)
        position_x = grid_min[0] + (center_x + 0.5f) * voxel_size; // Center of voxel
        position_y = grid_min[1] + (center_y + 0.5f) * voxel_size;
        position_z = grid_min[2] + (center_z + 0.5f) * voxel_size;

        // Calculate uncertainty based on weighted variance of voxel positions
        float var_x = 0.0f, var_y = 0.0f, var_z = 0.0f;
        for (const auto& voxel : voxels) {
            float weight = voxel.value / sum_weights;
            var_x += weight * std::pow(voxel.x - center_x, 2);
            var_y += weight * std::pow(voxel.y - center_y, 2);
            var_z += weight * std::pow(voxel.z - center_z, 2);
        }

        // Uncertainty as average standard deviation in world units
        uncertainty = voxel_size * std::sqrt((var_x + var_y + var_z) / 3.0f);
    } else {
        // Handle case with no significant weight (e.g., single voxel?)
        center_x = voxels[0].x;
        center_y = voxels[0].y;
        center_z = voxels[0].z;
        position_x = grid_min[0] + (center_x + 0.5f) * voxel_size;
        position_y = grid_min[1] + (center_y + 0.5f) * voxel_size;
        position_z = grid_min[2] + (center_z + 0.5f) * voxel_size;
        uncertainty = voxel_size; // Uncertainty is roughly the size of a voxel
    }
}

void VoxelDrone::updateVelocity(float prev_x, float prev_y, float prev_z,
                           const std::chrono::system_clock::time_point& prev_time) {
    auto now = std::chrono::system_clock::now();
    double dt = std::chrono::duration<double>(now - prev_time).count();

    if (dt > 0.01) { // Avoid division by very small dt, ensure minimum time has passed
        velocity_x = (position_x - prev_x) / dt;
        velocity_y = (position_y - prev_y) / dt;
        velocity_z = (position_z - prev_z) / dt;
        velocity_valid = true;
    } else {
        velocity_valid = false; // Not enough time passed for a reliable estimate
    }
}

float VoxelDrone::getSpeed() const {
    if (!velocity_valid) return 0.0f;
    return std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y + velocity_z * velocity_z);
}

float VoxelDrone::getAltitude() const {
    return position_z; // Assuming Z is altitude
}


// DroneDetection Implementation
DroneDetection::DroneDetection() :
    id(-1),
    uuid(""),
    x(0.0f), y(0.0f), z(0.0f),
    vx(0.0f), vy(0.0f), vz(0.0f),
    velocity_valid(false),
    uncertainty(0.0f),
    confidence(0.0f),
    detection_time(std::chrono::system_clock::now()) {}

DroneDetection::DroneDetection(const VoxelDrone& drone) :
    id(drone.id),
    uuid(drone.uuid),
    x(drone.position_x), y(drone.position_y), z(drone.position_z),
    vx(drone.velocity_x), vy(drone.velocity_y), vz(drone.velocity_z),
    velocity_valid(drone.velocity_valid),
    uncertainty(drone.uncertainty),
    confidence(drone.max_intensity), // Use max intensity as a simple confidence measure
    detection_time(drone.last_seen)
    {}

// VoxelPosition Implementation
VoxelPosition::VoxelPosition(int x, int y, int z) : x(x), y(y), z(z) {}

bool VoxelPosition::operator==(const VoxelPosition& other) const {
    return x == other.x && y == other.y && z == other.z;
}

// Hash function for VoxelPosition (needs to be outside the class)
namespace std {
    size_t hash<VoxelPosition>::operator()(const VoxelPosition& pos) const {
        // Combine the hash of x, y, z using a simple hash combiner
        // (FNV-1a variation or similar might be better, but this is simple)
        size_t h1 = std::hash<int>{}(pos.x);
        size_t h2 = std::hash<int>{}(pos.y);
        size_t h3 = std::hash<int>{}(pos.z);
        // Combine hashes (boost::hash_combine style)
        size_t seed = 0;
        seed ^= h1 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= h2 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= h3 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
}


// VoxelDroneDetector Implementation
VoxelDroneDetector::VoxelDroneDetector(
    int initial_grid_size_x,
    int initial_grid_size_y,
    int initial_grid_size_z,
    float threshold,
    float cluster_dist,
    int min_voxels,
    int max_voxels,
    int inactive_timeout_seconds)
    : next_id_(1),
      detection_threshold_(threshold),
      clustering_distance_sq_(cluster_dist * cluster_dist), // Store squared distance
      min_voxels_per_drone_(min_voxels),
      max_voxels_per_drone_(max_voxels),
      max_inactive_time_(std::chrono::seconds(inactive_timeout_seconds)),
      voxel_size_(0.0f), // Initialize properly in setGridParameters
      next_drone_id_(1) // This seems redundant with next_id_?
{
    grid_size_[0] = initial_grid_size_x;
    grid_size_[1] = initial_grid_size_y;
    grid_size_[2] = initial_grid_size_z;
    grid_min_[0] = 0.0f;
    grid_min_[1] = 0.0f;
    grid_min_[2] = 0.0f;
    // Ensure current_voxel_grid_ is sized appropriately if used
    current_voxel_grid_.resize(grid_size_[0] * grid_size_[1] * grid_size_[2], 0.0f);
}

void VoxelDroneDetector::setGridParameters(int size_x, int size_y, int size_z, float voxel_size_meters, const float min_coords[3]) {
    std::lock_guard<std::mutex> lock(detector_mutex_);
    grid_size_[0] = size_x;
    grid_size_[1] = size_y;
    grid_size_[2] = size_z;
    voxel_size_ = voxel_size_meters;
    grid_min_[0] = min_coords[0];
    grid_min_[1] = min_coords[1];
    grid_min_[2] = min_coords[2];
    // Resize internal grid if necessary
    current_voxel_grid_.resize(grid_size_[0] * grid_size_[1] * grid_size_[2], 0.0f);
    current_voxel_grid_.assign(current_voxel_grid_.size(), 0.0f); // Clear it
}

void VoxelDroneDetector::setVoxelSize(float size_m) {
    std::lock_guard<std::mutex> lock(detector_mutex_);
    voxel_size_ = size_m;
}

// This function name is a bit confusing. It likely means clustering parameters in voxel units.
void VoxelDroneDetector::setClusterParameters(float min_cluster_size_voxels, float max_cluster_size_voxels, float tolerance_voxels) {
     std::lock_guard<std::mutex> lock(detector_mutex_);
     // Assuming min/max are voxel counts, tolerance is distance
     min_voxels_per_drone_ = static_cast<int>(min_cluster_size_voxels);
     max_voxels_per_drone_ = static_cast<int>(max_cluster_size_voxels);
     clustering_distance_sq_ = tolerance_voxels * tolerance_voxels;
}


void VoxelDroneDetector::processVoxelGrid(const std::vector<float>& voxel_grid) {
    // Lock for configuration access, then lock for drone list access
    std::unique_lock<std::mutex> detector_lock(detector_mutex_);
    if (voxel_grid.size() != static_cast<size_t>(grid_size_[0] * grid_size_[1] * grid_size_[2])) {
        std::cerr << "Error: Input voxel_grid size mismatch! Expected "
                  << grid_size_[0] * grid_size_[1] * grid_size_[2] << ", got " << voxel_grid.size() << std::endl;
        return;
    }
    // Copy input grid to internal storage (or use reference if performance critical and safe)
    current_voxel_grid_ = voxel_grid;
    detector_lock.unlock(); // Unlock config mutex

    // Find potential drone voxels (above threshold)
    std::vector<VoxelPoint> potential_voxels;
    potential_voxels.reserve(current_voxel_grid_.size() / 100); // Pre-allocate some space
    for (int iz = 0; iz < grid_size_[2]; ++iz) {
        for (int iy = 0; iy < grid_size_[1]; ++iy) {
            for (int ix = 0; ix < grid_size_[0]; ++ix) {
                size_t index = static_cast<size_t>(ix) + 
                               static_cast<size_t>(iy) * grid_size_[0] + 
                               static_cast<size_t>(iz) * grid_size_[0] * grid_size_[1];
                if (current_voxel_grid_[index] >= detection_threshold_) {
                    potential_voxels.push_back({ix, iy, iz, current_voxel_grid_[index]});
                }
            }
        }
    }

    // Perform clustering, drone association, updates within drone list lock
    updateDetections(potential_voxels);
}

// Overload that processes the internally maintained voxel grid (populated via setVoxelValue)
void VoxelDroneDetector::processVoxelGrid() {
    // Reuse existing processing function using the internally accumulated grid.
    processVoxelGrid(current_voxel_grid_);
}

// Simulate setting a single voxel value
void VoxelDroneDetector::setVoxelValue(int vx, int vy, int vz, float value) {
    std::lock_guard<std::mutex> lock(detector_mutex_);
    if (vx >= 0 && vx < grid_size_[0] &&
        vy >= 0 && vy < grid_size_[1] &&
        vz >= 0 && vz < grid_size_[2])
    {
        size_t index = static_cast<size_t>(vx) +
                       static_cast<size_t>(vy) * grid_size_[0] +
                       static_cast<size_t>(vz) * grid_size_[0] * grid_size_[1];
        current_voxel_grid_[index] = value;
    }
}

// Clears the internal voxel grid representation
void VoxelDroneDetector::clearVoxelGrid() {
     std::lock_guard<std::mutex> lock(detector_mutex_);
     current_voxel_grid_.assign(current_voxel_grid_.size(), 0.0f);
}


// --- Private Methods --- //

void VoxelDroneDetector::updateDetections(const std::vector<VoxelPoint>& potential_voxels) {
    // Lock the drone list for the entire update process
    std::lock_guard<std::mutex> lock(drones_mutex_);

    // 1. Cluster the potential voxels
    std::vector<std::vector<VoxelPoint>> clusters = clusterVoxels(potential_voxels);

    // 2. Assign clusters to existing drones or create new drones
    assignVoxelsToDrones(clusters);

    // 3. Update properties (position, velocity) of all active drones
    updateDroneProperties();

    // 4. Remove drones that haven't been seen for a while
    removeInactiveDrones();
}


// Basic Density-Based Clustering (similar to DBSCAN but simpler)
std::vector<std::vector<VoxelPoint>> VoxelDroneDetector::clusterVoxels(const std::vector<VoxelPoint>& points) const {
    std::vector<std::vector<VoxelPoint>> clusters;
    std::unordered_set<VoxelPosition> visited;

    for (const auto& point : points) {
        VoxelPosition pos(point.x, point.y, point.z);
        if (visited.count(pos)) {
            continue; // Already part of a cluster
        }

        std::vector<VoxelPoint> current_cluster;
        std::queue<VoxelPoint> q;

        q.push(point);
        visited.insert(pos);
        current_cluster.push_back(point);

        while (!q.empty()) {
            VoxelPoint current_point = q.front();
            q.pop();

            // Find neighbors within clustering distance
            // This is inefficient O(N^2), could use spatial partitioning (k-d tree, octree)
            // For simplicity now, we iterate through all points
            for (const auto& neighbor_candidate : points) {
                VoxelPosition neighbor_pos(neighbor_candidate.x, neighbor_candidate.y, neighbor_candidate.z);
                if (visited.count(neighbor_pos)) {
                    continue;
                }

                float dx = static_cast<float>(current_point.x - neighbor_candidate.x);
                float dy = static_cast<float>(current_point.y - neighbor_candidate.y);
                float dz = static_cast<float>(current_point.z - neighbor_candidate.z);
                float dist_sq = dx * dx + dy * dy + dz * dz;

                if (dist_sq <= clustering_distance_sq_) {
                    visited.insert(neighbor_pos);
                    q.push(neighbor_candidate);
                    current_cluster.push_back(neighbor_candidate);
                }
            }
        }

        // Check if the cluster meets the minimum size requirement
        if (current_cluster.size() >= static_cast<size_t>(min_voxels_per_drone_) &&
            current_cluster.size() <= static_cast<size_t>(max_voxels_per_drone_)) {
            clusters.push_back(current_cluster);
        }
    }

    return clusters;
}

// Assigns clusters to the closest existing drone or creates a new drone
void VoxelDroneDetector::assignVoxelsToDrones(const std::vector<std::vector<VoxelPoint>>& clusters) {
    auto now = std::chrono::system_clock::now();
    std::vector<bool> cluster_assigned(clusters.size(), false);

    // Try to match clusters to existing drones based on proximity
    for (auto& drone : drones_) {
        float min_dist_sq = std::numeric_limits<float>::max();
        int best_cluster_idx = -1;

        // Calculate drone's center in voxel coordinates for comparison
        float drone_center_vx = (drone.position_x - grid_min_[0]) / voxel_size_ - 0.5f;
        float drone_center_vy = (drone.position_y - grid_min_[1]) / voxel_size_ - 0.5f;
        float drone_center_vz = (drone.position_z - grid_min_[2]) / voxel_size_ - 0.5f;

        for (size_t i = 0; i < clusters.size(); ++i) {
            if (cluster_assigned[i]) continue;

            // Calculate cluster centroid (simple average for matching)
            float cluster_cx = 0.f, cluster_cy = 0.f, cluster_cz = 0.f;
            for(const auto& p : clusters[i]) {
                cluster_cx += p.x;
                cluster_cy += p.y;
                cluster_cz += p.z;
            }
            cluster_cx /= clusters[i].size();
            cluster_cy /= clusters[i].size();
            cluster_cz /= clusters[i].size();

            float dx = drone_center_vx - cluster_cx;
            float dy = drone_center_vy - cluster_cy;
            float dz = drone_center_vz - cluster_cz;
            float dist_sq = dx * dx + dy * dy + dz * dz;

            // Check if this cluster is closer than previous best match
            // Use a threshold based on clustering distance? Maybe 2x clustering distance?
            float match_threshold_sq = 4.0f * clustering_distance_sq_;
            if (dist_sq < min_dist_sq && dist_sq < match_threshold_sq) {
                min_dist_sq = dist_sq;
                best_cluster_idx = i;
            }
        }

        // If a suitable cluster was found, assign it to this drone
        if (best_cluster_idx != -1) {
            drone.voxels = clusters[best_cluster_idx];
            drone.last_seen = now;
            cluster_assigned[best_cluster_idx] = true;
        } else {
            // No matching cluster found, mark drone voxels as empty for this frame
            drone.voxels.clear();
        }
    }

    // Create new drones for unassigned clusters
    for (size_t i = 0; i < clusters.size(); ++i) {
        if (!cluster_assigned[i]) {
            drones_.emplace_back(next_id_++);
            drones_.back().voxels = clusters[i];
            drones_.back().last_seen = now;
            // Initial position/velocity will be calculated in updateDroneProperties
        }
    }
}

void VoxelDroneDetector::updateDroneProperties() {
    for (auto& drone : drones_) {
        if (!drone.voxels.empty()) {
            float prev_x = drone.position_x;
            float prev_y = drone.position_y;
            float prev_z = drone.position_z;
            auto prev_time = drone.last_seen; // Use last_seen before updating position

            // Update position based on new voxel data
            drone.updatePosition(voxel_size_, grid_min_);

            // Update velocity if previous position was valid
            // Need to track if the drone was seen *last* frame for velocity
            // This logic needs refinement - perhaps store previous state explicitly
            if (prev_time != drone.last_seen) { // Simplistic check: was updated this cycle
                 drone.updateVelocity(prev_x, prev_y, prev_z, prev_time);
            }
        }
        // Note: Drones that were *not* assigned clusters keep their old position/velocity
        // but their last_seen timestamp isn't updated, leading to removal later.
    }
}

void VoxelDroneDetector::removeInactiveDrones() {
    auto now = std::chrono::system_clock::now();
    drones_.erase(
        std::remove_if(drones_.begin(), drones_.end(),
            [&](const VoxelDrone& drone) {
                return (now - drone.last_seen) > max_inactive_time_;
            }),
        drones_.end());
}


// --- Public Methods --- //

std::vector<DroneDetection> VoxelDroneDetector::getDetectedDrones() const {
    std::lock_guard<std::mutex> lock(drones_mutex_);
    std::vector<DroneDetection> detections;
    detections.reserve(drones_.size());
    for (const auto& drone : drones_) {
        // Only return drones that were seen recently (or have valid state)
        if (!drone.voxels.empty() || (std::chrono::system_clock::now() - drone.last_seen) < max_inactive_time_) {
             detections.emplace_back(drone); // Use the conversion constructor
        }
    }
    return detections;
}

std::optional<DroneDetection> VoxelDroneDetector::getDroneById(int id) const {
    std::lock_guard<std::mutex> lock(drones_mutex_);
    for (const auto& drone : drones_) {
        if (drone.id == id) {
            return DroneDetection(drone);
        }
    }
    return std::nullopt;
}

size_t VoxelDroneDetector::getDroneCount() const {
    std::lock_guard<std::mutex> lock(drones_mutex_);
    // Count only active drones?
    size_t count = 0;
     for (const auto& drone : drones_) {
        if (!drone.voxels.empty() || (std::chrono::system_clock::now() - drone.last_seen) < max_inactive_time_) {
             count++;
        }
    }
    return count;
   // return drones_.size(); // Or just total tracked drones?
}

void VoxelDroneDetector::voxelToWorld(int voxel_x, int voxel_y, int voxel_z, float& world_x, float& world_y, float& world_z) const {
    std::lock_guard<std::mutex> lock(detector_mutex_);
    world_x = grid_min_[0] + (static_cast<float>(voxel_x) + 0.5f) * voxel_size_;
    world_y = grid_min_[1] + (static_cast<float>(voxel_y) + 0.5f) * voxel_size_;
    world_z = grid_min_[2] + (static_cast<float>(voxel_z) + 0.5f) * voxel_size_;
}

void VoxelDroneDetector::worldToVoxel(float world_x, float world_y, float world_z, int& voxel_x, int& voxel_y, int& voxel_z) const {
    std::lock_guard<std::mutex> lock(detector_mutex_);
    voxel_x = static_cast<int>((world_x - grid_min_[0]) / voxel_size_);
    voxel_y = static_cast<int>((world_y - grid_min_[1]) / voxel_size_);
    voxel_z = static_cast<int>((world_z - grid_min_[2]) / voxel_size_);

    // Clamp to grid boundaries
    voxel_x = std::max(0, std::min(grid_size_[0] - 1, voxel_x));
    voxel_y = std::max(0, std::min(grid_size_[1] - 1, voxel_y));
    voxel_z = std::max(0, std::min(grid_size_[2] - 1, voxel_z));
}
