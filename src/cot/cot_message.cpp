// cot_message.cpp
//
// Cursor on Target (CoT) message generator for ATAK integration
// This module:
//   1) Converts drone position data to CoT XML format
//   2) Establishes connection to ATAK server
//   3) Periodically sends updates on drone positions

#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <optional>
#include <cmath>

// Network headers
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// Include the drone detector
#include "voxel_drone_detector.cpp"

class CoTMessageGenerator {
private:
    // ATAK server connection parameters
    std::string server_address;
    int server_port;
    int socket_fd;
    bool connected;
    
    // Background thread control
    std::thread cot_thread;
    std::atomic<bool> running;
    
    // Reference to the drone detector
    VoxelDroneDetector& drone_detector;
    
    // Mutex for thread safety
    std::mutex mutex;
    
    // Last update time for each drone (to avoid sending too many updates)
    std::map<int, std::chrono::system_clock::time_point> last_update_time;
    
    // Minimum update interval for each drone (in seconds)
    std::chrono::seconds min_update_interval;
    
    // Generate timestamp in CoT format (YYYY-MM-DDThh:mm:ss.sssZ)
    std::string generateTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
            
        std::stringstream ss;
        ss << std::put_time(std::gmtime(&now_c), "%Y-%m-%dT%H:%M:%S");
        ss << '.' << std::setfill('0') << std::setw(3) << now_ms.count() << "Z";
        
        return ss.str();
    }
    
    // Generate stale time (when this data should be considered stale)
    std::string generateStaleTime(int seconds_from_now) {
        auto stale_time = std::chrono::system_clock::now() + std::chrono::seconds(seconds_from_now);
        auto stale_c = std::chrono::system_clock::to_time_t(stale_time);
        
        std::stringstream ss;
        ss << std::put_time(std::gmtime(&stale_c), "%Y-%m-%dT%H:%M:%S");
        ss << ".000Z";
        
        return ss.str();
    }
    
    // Convert decimal latitude/longitude to military grid reference system (MGRS)
    // This is a simplified placeholder - real implementation would use a proper coordinate conversion library
    std::string decimalToMGRS(double lat, double lon) {
        // This is a very simplified placeholder
        // In a real implementation, you would use a library like GeographicLib
        std::stringstream ss;
        ss << std::fixed << std::setprecision(5) << "MGRS:" << lat << ":" << lon;
        return ss.str();
    }
    
    // Generate a CoT XML message for a drone
    std::string generateDroneCoTMessage(const VoxelDrone& drone) {
        // In a real implementation, we would need to convert our XYZ coordinates
        // to latitude, longitude, and altitude. This is a simplified version.
        
        // For this example, we'll assume:
        // 1. Our grid's origin (0,0,0) is at a known lat/lon (e.g., 38.8895, -77.0352)
        // 2. We have a conversion factor from our meters to degrees
        
        // These would come from your system's configuration
        const double origin_lat = 38.8895; // Example: Washington DC
        const double origin_lon = -77.0352;
        const double meters_per_degree_lat = 111111.0; // Approximate
        const double meters_per_degree_lon = 85000.0;  // Approximate at latitude 38.9°N
        
        // Convert XYZ to lat/lon/alt
        double lat = origin_lat + (drone.position_y / meters_per_degree_lat);
        double lon = origin_lon + (drone.position_x / meters_per_degree_lon);
        double alt = drone.position_z; // Altitude in meters
        
        // Calculate heading and speed
        double heading = 0.0;
        double speed = 0.0;
        
        if (drone.velocity_valid) {
            // Calculate heading (compass direction) from velocity vector
            heading = std::atan2(drone.velocity_y, drone.velocity_x) * 180.0 / M_PI;
            if (heading < 0) heading += 360.0;
            
            // Calculate speed in m/s
            speed = drone.getSpeed();
        }
        
        // Create the CoT XML
        std::stringstream xml;
        
        xml << "<?xml version='1.0' encoding='UTF-8' standalone='yes'?>";
        xml << "<event version='2.0' uid='" << drone.uuid << "' ";
        xml << "type='a-f-A-M-F-Q-r' "; // Type code for Remotely Piloted Aircraft (RPA)
        xml << "time='" << generateTimestamp() << "' ";
        xml << "start='" << generateTimestamp() << "' ";
        xml << "stale='" << generateStaleTime(60) << "' "; // Stale after 60 seconds
        xml << "how='m-g'>";  // GPS measurement
        
        // Point (position)
        xml << "<point lat='" << std::fixed << std::setprecision(7) << lat << "' ";
        xml << "lon='" << std::fixed << std::setprecision(7) << lon << "' ";
        xml << "hae='" << std::fixed << std::setprecision(1) << alt << "' ";
        xml << "ce='" << std::fixed << std::setprecision(1) << drone.uncertainty << "' ";
        xml << "le='" << std::fixed << std::setprecision(1) << drone.uncertainty << "'/>";
        
        // Detail section
        xml << "<detail>";
        
        // Track information
        xml << "<track course='" << std::fixed << std::setprecision(1) << heading << "' ";
        xml << "speed='" << std::fixed << std::setprecision(1) << speed << "'/>";
        
        // Contact information
        xml << "<contact callsign='Drone" << drone.id << "'/>";
        
        // Drone-specific details
        xml << "<uid Droid='" << drone.uuid << "'/>";
        xml << "<precisionlocation altsrc='BAROMETRIC'/>";
        
        // Additional drone information
        xml << "<_flow-tags_ Speed='" << std::fixed << std::setprecision(1) << speed << "' ";
        xml << "Course='" << std::fixed << std::setprecision(1) << heading << "'/>";
        
        // Remarks
        xml << "<remarks>Detected drone with intensity " << std::fixed << std::setprecision(2) 
            << drone.total_intensity << "</remarks>";
            
        xml << "</detail>";
        xml << "</event>";
        
        return xml.str();
    }
    
    // Connect to ATAK server
    bool connectToServer() {
        socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd < 0) {
            return false;
        }
        
        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(server_port);
        
        if (inet_pton(AF_INET, server_address.c_str(), &server_addr.sin_addr) <= 0) {
            close(socket_fd);
            return false;
        }
        
        // For UDP, we don't actually connect, but we can store the server address
        connected = true;
        return true;
    }
    
    // Send CoT message to ATAK server
    bool sendCoTMessage(const std::string& message) {
        if (!connected) {
            if (!connectToServer()) {
                return false;
            }
        }
        
        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(server_port);
        
        if (inet_pton(AF_INET, server_address.c_str(), &server_addr.sin_addr) <= 0) {
            return false;
        }
        
        ssize_t sent = sendto(socket_fd, message.c_str(), message.length(), 0,
                             (struct sockaddr*)&server_addr, sizeof(server_addr));
                             
        return (sent == static_cast<ssize_t>(message.length()));
    }
    
    // CoT message sender thread function
    void cotSenderThread() {
        while (running) {
            // Get current detected drones
            auto drones = drone_detector.getDetectedDrones();
            auto now = std::chrono::system_clock::now();
            
            std::lock_guard<std::mutex> lock(mutex);
            
            // Send update for each drone
            for (const auto& drone : drones) {
                // Check if we should update this drone yet
                auto it = last_update_time.find(drone.id);
                bool should_update = (it == last_update_time.end()) ||
                                   ((now - it->second) > min_update_interval);
                
                if (should_update) {
                    // Generate and send CoT message
                    std::string cot_message = generateDroneCoTMessage(drone);
                    sendCoTMessage(cot_message);
                    
                    // Update the last update time
                    last_update_time[drone.id] = now;
                }
            }
            
            // Sleep for a short time before checking again
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
public:
    CoTMessageGenerator(VoxelDroneDetector& detector,
                      const std::string& server_ip = "127.0.0.1",
                      int server_port = 8087,
                      int update_interval_seconds = 1)
        : server_address(server_ip),
          server_port(server_port),
          connected(false),
          running(false),
          drone_detector(detector),
          min_update_interval(update_interval_seconds)
    {
    }
    
    ~CoTMessageGenerator() {
        stop();
        
        if (socket_fd >= 0) {
            close(socket_fd);
        }
    }
    
    // Start the CoT sender thread
    bool start() {
        std::lock_guard<std::mutex> lock(mutex);
        
        if (running) {
            return true; // Already running
        }
        
        // Try to connect to the server
        if (!connectToServer()) {
            return false;
        }
        
        // Start the sender thread
        running = true;
        cot_thread = std::thread(&CoTMessageGenerator::cotSenderThread, this);
        
        return true;
    }
    
    // Stop the CoT sender thread
    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex);
            
            if (!running) {
                return;
            }
            
            running = false;
        }
        
        if (cot_thread.joinable()) {
            cot_thread.join();
        }
        
        connected = false;
    }
    
    // Force send an update for a specific drone
    bool forceSendUpdate(int drone_id) {
        std::lock_guard<std::mutex> lock(mutex);
        
        auto drone_opt = drone_detector.getDroneById(drone_id);
        if (!drone_opt) {
            return false;
        }
        
        std::string cot_message = generateDroneCoTMessage(*drone_opt);
        bool success = sendCoTMessage(cot_message);
        
        if (success) {
            last_update_time[drone_id] = std::chrono::system_clock::now();
        }
        
        return success;
    }
    
    // Update server connection parameters
    void updateServerSettings(const std::string& server_ip, int port) {
        std::lock_guard<std::mutex> lock(mutex);
        
        bool was_running = running;
        
        // Stop if running
        if (was_running) {
            stop();
        }
        
        // Update settings
        server_address = server_ip;
        server_port = port;
        
        // Reconnect if needed
        if (was_running) {
            start();
        }
    }
    
    // Set the minimum update interval
    void setUpdateInterval(int seconds) {
        std::lock_guard<std::mutex> lock(mutex);
        min_update_interval = std::chrono::seconds(seconds);
    }
    
    // Check if connected to server
    bool isConnected() const {
        return connected;
    }
    
    // Generate a CoT message for testing (without sending)
    std::string generateTestMessage(int drone_id) {
        auto drone_opt = drone_detector.getDroneById(drone_id);
        if (!drone_opt) {
            return "";
        }
        
        return generateDroneCoTMessage(*drone_opt);
    }
}; 