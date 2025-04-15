#pragma once

#include <string>
#include <vector>
#include <ctime>
#include <random>
#include <memory>
#include <unordered_map>
#include <chrono>

// Simple struct to represent a 3D position with additional metadata
struct Detection {
    float x;           // X coordinate in meters (east/west)
    float y;           // Y coordinate in meters (north/south)
    float z;           // Z coordinate in meters (altitude)
    float confidence;  // Detection confidence (0.0-1.0)
    int classification; // Classification type (0=unknown, 1=drone, etc.)
    std::string uid;   // Unique identifier for this detection
    std::chrono::system_clock::time_point timestamp; // When the detection occurred
};

class CoTMessageGenerator {
public:
    // Constructor
    CoTMessageGenerator(
        const std::string& sender_uid = "",
        const std::string& team_name = "PIXELVOXEL",
        const std::string& team_role = "SENSOR");

    // Set location of the sensor system
    void setSensorLocation(double lat, double lon, double alt);
    
    // Generate a CoT message for a single detection
    std::string generateCoTMessage(const Detection& detection);
    
    // Generate CoT messages for multiple detections
    std::vector<std::string> generateCoTMessages(const std::vector<Detection>& detections);
    
    // Generate a sensor status/heartbeat message
    std::string generateStatusMessage(bool operational = true);
    
    // Set the sender UID
    void setSenderUID(const std::string& uid);
    
    // Set team information
    void setTeamInfo(const std::string& name, const std::string& role);
    
    // Convert from local coordinates to lat/lon/alt
    // x = east(+)/west(-), y = north(+)/south(-), z = altitude
    void localToGlobal(float x, float y, float z, double& lat, double& lon, double& alt) const;

private:
    // Generate a unique ID if one isn't provided
    std::string generateUID() const;
    
    // Format time in CoT format (ISO 8601)
    std::string formatTime(const std::chrono::system_clock::time_point& time) const;
    
    // Format time with milliseconds for stale/start/time attributes
    std::string formatTimeWithMilliseconds(const std::chrono::system_clock::time_point& time) const;
    
    // Get a human-readable classification
    std::string getClassificationString(int classification) const;
    
    // Get a CoT type string based on classification
    std::string getCoTType(int classification) const;
    
    // Member variables
    std::string sender_uid_;          // UID of the sender (this system)
    std::string team_name_;           // Team name 
    std::string team_role_;           // Team role
    
    // Sensor location (reference point for local coordinates)
    double sensor_lat_;               // Latitude in degrees
    double sensor_lon_;               // Longitude in degrees
    double sensor_alt_;               // Altitude in meters (above WGS84)
    
    // Random generator for UIDs
    mutable std::mt19937 rng_;
}; 