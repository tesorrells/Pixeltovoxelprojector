// cot_message_generator.cpp
//
// This file provides a class for generating Cursor on Target (CoT) messages
// for detected objects to be displayed on ATAK or other CoT-compatible systems.

#include "cot/cot_message_generator.h"

#include <sstream>
#include <iomanip>
#include <cmath>
#include <stdexcept> // For invalid arguments
#include <mutex> // For potential future thread safety needs if rng becomes shared

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Earth constants (WGS84 approximation)
constexpr double EARTH_RADIUS_METERS = 6378137.0;
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;

// --- Private Helper Functions --- 

// Format time in CoT format (ISO 8601 Zulu)
std::string CoTMessageGenerator::formatTime(const std::chrono::system_clock::time_point& time) const {
    return formatTimeWithMilliseconds(time); // Reuse the milliseconds version
}

// Format time with milliseconds for stale/start/time attributes
std::string CoTMessageGenerator::formatTimeWithMilliseconds(const std::chrono::system_clock::time_point& time) const {
    using namespace std::chrono;

    // Convert to milliseconds precision
    auto ms = duration_cast<milliseconds>(time.time_since_epoch()) % 1000;

    // Convert to time_t for formatting date/time part
    std::time_t t = system_clock::to_time_t(time);
    // Use gmtime_r or gmtime_s for thread safety if needed, though formatting itself isn't stateful
    #ifdef _WIN32
        std::tm tm_info_s;
        gmtime_s(&tm_info_s, &t);
        std::tm* tm_info = &tm_info_s;
    #else
        std::tm tm_info_s;
        std::tm* tm_info = gmtime_r(&t, &tm_info_s);
    #endif

    if (!tm_info) {
        return ""; // Error handling
    }

    std::stringstream ss;
    // Use put_time for formatting - check availability/compatibility
    // Alternative: manual formatting
    char buffer[30];
    strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", tm_info);
    ss << buffer;
    ss << "." << std::setfill('0') << std::setw(3) << ms.count() << "Z";

    return ss.str();
}

// Get a human-readable classification string (example)
std::string CoTMessageGenerator::getClassificationString(int classification) const {
    // Example map - customize as needed
    static const std::unordered_map<int, std::string> class_map = {
        {0, "Unknown"},
        {1, "Drone"},
        {2, "Bird"},
        {3, "Other"}
    };
    auto it = class_map.find(classification);
    return (it != class_map.end()) ? it->second : "Unknown";
}

// Get a CoT type string based on classification (SIDC based)
std::string CoTMessageGenerator::getCoTType(int classification) const {
    // Example map based on MIL-STD-2525 / APP-6
    static const std::unordered_map<int, std::string> type_map = {
        {0, "a-u-G"},           // Pending Unknown Ground (Generic)
        {1, "a-u-A-M-F-Q-r"},   // Assumed Friend Air Missile FixedWing RotaryWing SmallUAS
        {2, "a-n-A-C-F"},       // Neutral Air Civilian FixedWing (Bird approximation)
        {3, "a-u-G"}            // Pending Unknown Ground (Generic Other)
    };
     auto it = type_map.find(classification);
    return (it != type_map.end()) ? it->second : "a-u-G";
}

// Generate a unique ID (simple implementation)
std::string CoTMessageGenerator::generateUID() const {
    // Format: "UID-[random_number]"
    // A more robust implementation might use UUID libraries
    std::uniform_int_distribution<> dis(100000, 999999);
    // rng_ needs to be mutable if generateUID is const
    return sender_uid_ + "-" + std::to_string(dis(rng_)); 
}

// --- Constructor --- 
CoTMessageGenerator::CoTMessageGenerator(
    const std::string& sender_uid,
    const std::string& team_name,
    const std::string& team_role)
    : sender_uid_(sender_uid),
      team_name_(team_name),
      team_role_(team_role),
      sensor_lat_(0.0), sensor_lon_(0.0), sensor_alt_(0.0), // Default origin
      rng_(std::random_device{}()) // Seed the random generator
{
    if (sender_uid_.empty()) {
        // Generate a default UID if none provided
        sender_uid_ = "PixelVoxelSensor-" + std::to_string(std::uniform_int_distribution<>(100,999)(rng_));
    }
}

// --- Public Methods --- 

void CoTMessageGenerator::setSensorLocation(double lat, double lon, double alt) {
    // Add mutex lock here if multithreaded access is expected
    sensor_lat_ = lat;
    sensor_lon_ = lon;
    sensor_alt_ = alt;
}

void CoTMessageGenerator::setSenderUID(const std::string& uid) {
     // Add mutex lock here if multithreaded access is expected
     sender_uid_ = uid;
}

void CoTMessageGenerator::setTeamInfo(const std::string& name, const std::string& role) {
    // Add mutex lock here if multithreaded access is expected
    team_name_ = name;
    team_role_ = role;
}

// Convert local tangent plane coordinates (East, North, Up) relative to sensor origin to global WGS84
void CoTMessageGenerator::localToGlobal(float x_east, float y_north, float z_up, double& lat, double& lon, double& alt) const {
    // Basic calculation (flat Earth approximation near origin)
    double lat_rad = sensor_lat_ * DEG_TO_RAD;

    // Meters per degree calculation (simplified)
    double meters_per_lat_deg = 111132.92 - 559.82 * cos(2 * lat_rad) + 1.175 * cos(4 * lat_rad);
    double meters_per_lon_deg = 111412.84 * cos(lat_rad);

    if (std::abs(meters_per_lat_deg) < 1e-6 || std::abs(meters_per_lon_deg) < 1e-6) {
        // Avoid division by zero near poles or if origin is invalid
        lat = sensor_lat_; 
        lon = sensor_lon_;
        alt = sensor_alt_ + z_up;
        return;
    }

    lat = sensor_lat_ + (y_north / meters_per_lat_deg);
    lon = sensor_lon_ + (x_east / meters_per_lon_deg);
    alt = sensor_alt_ + z_up; // Altitude above WGS84 ellipsoid
}

std::string CoTMessageGenerator::generateCoTMessage(const Detection& detection) {
    // Convert local detection coordinates to global
    double lat, lon, alt;
    localToGlobal(detection.x, detection.y, detection.z, lat, lon, alt);

    // Generate timestamps
    auto now = std::chrono::system_clock::now(); 
    std::string time_str = formatTimeWithMilliseconds(now);
    // Stale time: e.g., 60 seconds from now
    std::string stale_str = formatTimeWithMilliseconds(now + std::chrono::seconds(60));

    // Get CoT type based on classification
    std::string cot_type = getCoTType(detection.classification);

    // Use provided UID or generate one based on sender UID + sequence/time
    std::string event_uid = detection.uid; 
    if (event_uid.empty()) {
        event_uid = generateUID(); // Generate if detection doesn't have one
    }

    // Format the CoT XML message
    std::stringstream xml;
    xml << std::fixed << std::setprecision(8); // Precision for lat/lon
    xml << "<?xml version='1.0' encoding='UTF-8' standalone='yes'?>";
    xml << "<event version='2.0' uid='" << event_uid << "' ";
    xml << "type='" << cot_type << "' ";
    xml << "time='" << time_str << "' ";
    xml << "start='" << time_str << "' "; // Detection time is 'now' for this simple version
    xml << "stale='" << stale_str << "' ";
    xml << "how='m-p'>"; // Method: Machine - Predicted/derived

    // Point position
    xml << "<point lat='" << lat << "' ";
    xml << "lon='" << lon << "' ";
    xml << "hae='" << std::setprecision(2) << alt << "' "; // Height above ellipsoid
    // Circular Error (CE) and Linear Error (LE) - example values
    float uncertainty_estimate = std::max(1.0f, detection.confidence > 0 ? (1.0f / detection.confidence) : 100.0f);
    xml << "ce='" << uncertainty_estimate << "' le='" << uncertainty_estimate << "'/>"; 

    // Detail section
    xml << "<detail>";
    xml << "<contact callsign='" << getClassificationString(detection.classification) << "-" << detection.uid.substr(0,4) << "'/>"; // Example callsign
    xml << "<precisionlocation altsrc='DTED0' geopointsrc='User'/>"; // Source info
    xml << "<status battery='100'/>"; // Example status
    xml << "<track course='0' speed='0'/>"; // Basic track info (can add velocity later)
    // Add custom fields if needed, e.g., confidence
     xml << "<sensor Fov='60.0'/>"; // Example sensor field
    xml << "</detail>";

    xml << "</event>";

    return xml.str();
}

std::vector<std::string> CoTMessageGenerator::generateCoTMessages(const std::vector<Detection>& detections) {
    std::vector<std::string> messages;
    messages.reserve(detections.size());
    for (const auto& det : detections) {
        messages.push_back(generateCoTMessage(det));
    }
    return messages;
}

std::string CoTMessageGenerator::generateStatusMessage(bool operational) {
    // Generate timestamps
    auto now = std::chrono::system_clock::now(); 
    std::string time_str = formatTimeWithMilliseconds(now);
    std::string stale_str = formatTimeWithMilliseconds(now + std::chrono::seconds(120)); // Longer stale time for status

    // Format the CoT XML message for sensor status
    std::stringstream xml;
    xml << std::fixed << std::setprecision(8);
    xml << "<?xml version='1.0' encoding='UTF-8' standalone='yes'?>";
    // Use the sender UID for the status message event UID
    xml << "<event version='2.0' uid='" << sender_uid_ << "' "; 
    xml << "type='b-m-p-s-p-i' "; // Platform Sensor Point Instrument (example type)
    xml << "time='" << time_str << "' ";
    xml << "start='" << time_str << "' ";
    xml << "stale='" << stale_str << "' ";
    xml << "how='h-e'>"; // Human - Estimated/manual

    // Point position (sensor location)
    xml << "<point lat='" << sensor_lat_ << "' ";
    xml << "lon='" << sensor_lon_ << "' ";
    xml << "hae='" << std::setprecision(2) << sensor_alt_ << "' ";
    xml << "ce='10' le='10'/>"; // Sensor position uncertainty

    // Detail section for status
    xml << "<detail>";
    xml << "<contact callsign='" << sender_uid_ << "'/>"; 
    xml << "<remarks>Sensor status: " << (operational ? "Operational" : "Offline") << ".</remarks>";
    xml << "<link uid='" << sender_uid_ << "' production_time='" << time_str << "' type='a-f-G' parent_callsign='" << team_name_ << "'/>"; // Link to parent unit/team
    xml << "<status readiness='" << (operational ? "true" : "false") << "'/>";
    xml << "<uid Droid='" << sender_uid_ << "'/>";
    xml << "<__group name='" << team_name_ << "' role='" << team_role_ << "'/>"; // TAK Group info
    xml << "</detail>";

    xml << "</event>";

    return xml.str();
} 