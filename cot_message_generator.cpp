// cot_message_generator.cpp
//
// This file provides a class for generating Cursor on Target (CoT) messages
// for detected drones to be displayed on ATAK or other CoT-compatible systems.

#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <random>
#include <cmath>
#include <vector>
#include <mutex>
#include <map>

// Earth constants (WGS84)
constexpr double EARTH_RADIUS_METERS = 6378137.0;
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;

// DroneDetection struct
struct DroneDetection {
    int id;                     // Unique identifier for this drone
    float x, y, z;              // Position in local coordinates (meters)
    float vx, vy, vz;           // Velocity vector (m/s)
    float size;                 // Estimated size (meters)
    float confidence;           // Detection confidence [0.0-1.0]
    int classification;         // Classification type (0=unknown, 1=drone, 2=bird, 3=other)
    std::chrono::system_clock::time_point detection_time; // When the drone was detected
};

class CoTMessageGenerator {
private:
    // Origin coordinates for local to global coordinate conversion
    double origin_lat;
    double origin_lon;
    double origin_alt;
    
    // Sensor information
    std::string sensor_uid;
    std::string team_name;
    std::string team_role;
    
    // Server information
    std::string server_ip;
    int server_port;
    
    // Mutex for thread safety
    std::mutex message_mutex;
    
    // Classification type mapping
    std::map<int, std::string> classification_map = {
        {0, "UNKNOWN"},
        {1, "SMALL_UAS"},
        {2, "BIRD"},
        {3, "OTHER"}
    };
    
    // CoT type mapping
    std::map<int, std::string> cot_type_map = {
        {0, "a-u-G"},           // Unknown
        {1, "a-f-A-M-F-Q-r"},   // Small UAS
        {2, "a-n-A-C-F"},       // Bird
        {3, "a-u-G"}            // Other (generic)
    };
    
    // Generate a unique UID for CoT messages
    std::string generateUID() {
        // Format: "DroneDetect-[timestamp]-[random]"
        auto now = std::chrono::system_clock::now();
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
        
        // Add some randomness
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<> dis(1000, 9999);
        
        std::stringstream ss;
        ss << "DroneDetect-" << now_ms << "-" << dis(gen);
        return ss.str();
    }
    
    // Convert local coordinates to lat/lon
    void localToLatLon(float x, float y, float z, double& lat, double& lon, double& alt) const {
        // Convert origin to radians
        double lat_rad = origin_lat * DEG_TO_RAD;
        double lon_rad = origin_lon * DEG_TO_RAD;
        
        // Calculate meters per degree at the origin latitude
        double meters_per_lat = 111132.92 - 559.82 * cos(2 * lat_rad) + 1.175 * cos(4 * lat_rad);
        double meters_per_lon = 111412.84 * cos(lat_rad) - 93.5 * cos(3 * lat_rad);
        
        // Convert x,y to lat,lon
        lat = origin_lat + (y / meters_per_lat);
        lon = origin_lon + (x / meters_per_lon);
        alt = origin_alt + z; // Altitude in meters above origin
    }
    
    // Format time for CoT messages
    std::string formatTime(const std::chrono::system_clock::time_point& time) const {
        using namespace std::chrono;
        
        // Convert to milliseconds precision
        auto ms = duration_cast<milliseconds>(time.time_since_epoch()) % 1000;
        
        // Convert to time_t for formatting
        std::time_t t = system_clock::to_time_t(time);
        std::tm* tm_info = std::gmtime(&t);
        
        std::stringstream ss;
        ss << std::put_time(tm_info, "%Y-%m-%dT%H:%M:%S");
        ss << "." << std::setfill('0') << std::setw(3) << ms.count() << "Z";
        
        return ss.str();
    }
    
public:
    // Constructor
    CoTMessageGenerator(const std::string& sensorUID, 
                       const std::string& teamName, 
                       const std::string& teamRole)
        : sensor_uid(sensorUID),
          team_name(teamName),
          team_role(teamRole),
          origin_lat(0.0),
          origin_lon(0.0),
          origin_alt(0.0),
          server_ip("127.0.0.1"),
          server_port(8087)
    {}
    
    // Set the sensor location (origin for local coordinate system)
    void setSensorLocation(double lat, double lon, double alt) {
        std::lock_guard<std::mutex> lock(message_mutex);
        origin_lat = lat;
        origin_lon = lon;
        origin_alt = alt;
    }
    
    // Set the server information
    void setServer(const std::string& ip, int port) {
        std::lock_guard<std::mutex> lock(message_mutex);
        server_ip = ip;
        server_port = port;
    }
    
    // Generate a CoT XML message for a drone detection
    std::string generateCoTMessage(const DroneDetection& drone) {
        std::lock_guard<std::mutex> lock(message_mutex);
        
        // Convert local coordinates to lat/lon
        double lat, lon, alt;
        localToLatLon(drone.x, drone.y, drone.z, lat, lon, alt);
        
        // Calculate speed in m/s
        float speed = std::sqrt(drone.vx * drone.vx + drone.vy * drone.vy + drone.vz * drone.vz);
        
        // Calculate heading in degrees (0 = North, 90 = East)
        float heading = 0.0f;
        if (speed > 0.1f) {  // Only calculate heading if moving
            heading = std::atan2(drone.vx, drone.vy) * RAD_TO_DEG;
            if (heading < 0) {
                heading += 360.0f;  // Convert to 0-360 range
            }
        }
        
        // Generate stale time (when the detection will be considered stale)
        auto stale_time = drone.detection_time + std::chrono::seconds(30);  // 30 seconds from detection
        
        // Format timestamps
        std::string time_str = formatTime(drone.detection_time);
        std::string stale_str = formatTime(stale_time);
        
        // Generate unique ID for this detection
        std::string uid = "DroneDetect." + std::to_string(drone.id);
        
        // Get classification type (default to unknown if not found)
        int class_type = drone.classification;
        if (class_type < 0 || class_type > 3) class_type = 0;
        
        // Format the CoT XML message
        std::stringstream xml;
        xml << "<?xml version='1.0' encoding='UTF-8' standalone='yes'?>";
        xml << "<event version='2.0' uid='" << uid << "' ";
        xml << "type='" << cot_type_map[class_type] << "' ";
        xml << "time='" << time_str << "' ";
        xml << "start='" << time_str << "' ";
        xml << "stale='" << stale_str << "' ";
        xml << "how='m-g'>";  // Method: GPS
        
        // Point position
        xml << "<point lat='" << std::fixed << std::setprecision(7) << lat << "' ";
        xml << "lon='" << std::fixed << std::setprecision(7) << lon << "' ";
        xml << "hae='" << std::fixed << std::setprecision(1) << alt << "' ";
        xml << "ce='10.0' le='5.0'/>";  // Circular and linear error (meters)
        
        // Detail section
        xml << "<detail>";
        
        // Track information
        xml << "<track course='" << std::fixed << std::setprecision(1) << heading << "' ";
        xml << "speed='" << std::fixed << std::setprecision(1) << speed << "'/>";
        
        // UAS information (drone specific)
        xml << "<uas>";
        xml << "<size>" << std::fixed << std::setprecision(1) << drone.size << "</size>";
        xml << "<confidence>" << std::fixed << std::setprecision(2) << drone.confidence << "</confidence>";
        xml << "<classification>" << classification_map[class_type] << "</classification>";
        xml << "</uas>";
        
        // Contact information
        xml << "<contact callsign='" << classification_map[class_type] << "-" << drone.id << "'/>";
        
        // Group information
        xml << "<__group name='" << team_name << "' role='" << team_role << "'/>";
        
        // Remarks
        xml << "<remarks>Detected via 3D voxel analysis. Confidence: " 
            << std::fixed << std::setprecision(2) << (drone.confidence * 100.0) << "%</remarks>";
        
        xml << "</detail>";
        xml << "</event>";
        
        return xml.str();
    }
    
    // Generate CoT messages for multiple drone detections
    std::vector<std::string> generateCoTMessages(const std::vector<DroneDetection>& detections) {
        std::vector<std::string> messages;
        
        for (const auto& detection : detections) {
            if (detection.confidence > 0.3) {  // Only report detections with sufficient confidence
                messages.push_back(generateCoTMessage(detection));
            }
        }
        
        return messages;
    }
    
    // Generate a status message for the sensor
    std::string generateStatusMessage(bool operational = true) {
        std::lock_guard<std::mutex> lock(message_mutex);
        
        auto now = std::chrono::system_clock::now();
        auto stale_time = now + std::chrono::minutes(5);  // Status valid for 5 minutes
        
        // Format timestamps
        std::string time_str = formatTime(now);
        std::string stale_str = formatTime(stale_time);
        
        // Format the CoT XML message
        std::stringstream xml;
        xml << "<?xml version='1.0' encoding='UTF-8' standalone='yes'?>";
        xml << "<event version='2.0' uid='" << sensor_uid << "' ";
        xml << "type='b-g-S-S-X-M' ";  // Type code for sensor
        xml << "time='" << time_str << "' ";
        xml << "start='" << time_str << "' ";
        xml << "stale='" << stale_str << "' ";
        xml << "how='m-g'>";  // Method: GPS
        
        // Point position (sensor location)
        xml << "<point lat='" << std::fixed << std::setprecision(7) << origin_lat << "' ";
        xml << "lon='" << std::fixed << std::setprecision(7) << origin_lon << "' ";
        xml << "hae='" << std::fixed << std::setprecision(1) << origin_alt << "' ";
        xml << "ce='10.0' le='5.0'/>";  // Circular and linear error (meters)
        
        // Detail section
        xml << "<detail>";
        
        // Sensor information
        xml << "<sensor>";
        xml << "<type>VOXEL_DRONE_DETECTOR</type>";
        xml << "<status>" << (operational ? "OPERATIONAL" : "DEGRADED") << "</status>";
        xml << "</sensor>";
        
        // Contact information
        xml << "<contact callsign='VOXEL_SENSOR'/>";
        
        // Group information
        xml << "<__group name='" << team_name << "' role='" << team_role << "'/>";
        
        // Remarks
        xml << "<remarks>3D Voxel-based Drone Detection System</remarks>";
        
        xml << "</detail>";
        xml << "</event>";
        
        return xml.str();
    }
    
    // Get the server IP
    std::string getServerIP() const {
        return server_ip;
    }
    
    // Get the server port
    int getServerPort() const {
        return server_port;
    }
}; 