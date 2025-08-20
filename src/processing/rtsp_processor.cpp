// rtsp_processor.cpp
//
// Real-time RTSP stream processor for voxel-based 3D reconstruction
// This code:
//   1) Connects to multiple RTSP streams
//   2) Processes frames in real-time
//   3) Detects motion between consecutive frames for each camera
//   4) Casts rays (voxel DDA) for changed pixels
//   5) Accumulates in a shared 3D voxel grid
//   6) Optionally saves or visualizes the voxel grid

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <condition_variable>
#ifdef HAVE_ZMQ
#include <zmq.hpp>
#include <zlib.h>
#endif

#ifdef HAVE_DEPTHAI
#include "depthai/depthai.hpp"
#endif

// External libraries for JSON, OpenCV for stream handling
#include "nlohmann/json.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

// For convenience
using json = nlohmann::json;

//----------------------------------------------
// 1) Data Structures
//----------------------------------------------
struct Vec3 {
    float x, y, z;
};

struct Mat3 {
    float m[9];
};

enum class CameraType {
    RTSP,
    OAK_D_PRO,
    IMAGE_SEQUENCE
};

struct CameraInfo {
    int camera_index;
    CameraType camera_type;
    std::string rtsp_url;  // For RTSP cameras
    std::string device_id; // For OAK-D Pro cameras (optional, defaults to auto-detect)
    std::string test_data_path; // For image sequences (OAK-D Pro captured frames)
    Vec3 camera_position;
    float yaw, pitch, roll;
    float fov_degrees;
    // OAK-D Pro specific parameters
    bool use_depth;        // Whether to use depth data for enhanced reconstruction
    int rgb_resolution_width;  // RGB camera resolution
    int rgb_resolution_height;
    int fps;               // Target FPS
};

struct FrameData {
    int camera_index;
    cv::Mat frame;  // The actual frame from the camera
    std::chrono::steady_clock::time_point timestamp; // monotonic, immune to clock jumps
};

// Thread-safe frame queue
class FrameQueue {
private:
    std::queue<FrameData> queue;
    std::mutex mutex;
    std::condition_variable cond;
    static constexpr std::size_t MAX_Q = 30; // total frames kept in queue

public:
    void push(const FrameData& frame) {
        std::unique_lock<std::mutex> lock(mutex);
        // Drop oldest frames when queue is full to keep latency bounded
        while (queue.size() >= MAX_Q) {
            queue.pop();
        }
        queue.push(frame);
        cond.notify_one();
    }

    bool pop(FrameData& frame) {
        std::unique_lock<std::mutex> lock(mutex);
        if (queue.empty()) {
            return false;
        }
        frame = queue.front();
        queue.pop();
        return true;
    }

    bool waitAndPop(FrameData& frame, int timeout_ms) {
        std::unique_lock<std::mutex> lock(mutex);
        if (cond.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] { return !queue.empty(); })) {
            frame = queue.front();
            queue.pop();
            return true;
        }
        return false;
    }

    std::size_t size() {
        std::unique_lock<std::mutex> lock(mutex);
        return queue.size();
    }
};

//----------------------------------------------
// 2) Basic Math Helpers
//----------------------------------------------
static inline float deg2rad(float deg) {
    return deg * 3.14159265358979323846f / 180.0f;
}

static inline Vec3 normalize(const Vec3 &v) {
    float len = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if(len < 1e-12f) {
        return {0.f, 0.f, 0.f};
    }
    return { v.x/len, v.y/len, v.z/len };
}

// Multiply 3x3 matrix by Vec3
static inline Vec3 mat3_mul_vec3(const Mat3 &M, const Vec3 &v) {
    Vec3 r;
    r.x = M.m[0]*v.x + M.m[1]*v.y + M.m[2]*v.z;
    r.y = M.m[3]*v.x + M.m[4]*v.y + M.m[5]*v.z;
    r.z = M.m[6]*v.x + M.m[7]*v.y + M.m[8]*v.z;
    return r;
}

//----------------------------------------------
// 3) Euler -> Rotation Matrix
//----------------------------------------------
Mat3 rotation_matrix_yaw_pitch_roll(float yaw_deg, float pitch_deg, float roll_deg) {
    float y = deg2rad(yaw_deg);   // yaw (Z)
    float p = deg2rad(pitch_deg); // pitch (Y)
    float r = deg2rad(roll_deg);  // roll (X)
    
    // Rz(yaw)
    float cy = std::cos(y), sy = std::sin(y);
    float Rz[9] = { cy,-sy,0,  sy,cy,0,  0,0,1 };
    
    // Ry(pitch)
    float cp = std::cos(p), sp = std::sin(p);
    float Ry[9] = {  cp,0,sp,  0,1,0,  -sp,0,cp };
    
    // Rx(roll)
    float cr = std::cos(r), sr = std::sin(r);
    float Rx[9] = { 1,0,0,  0,cr,-sr,  0,sr,cr };

    // Helper to multiply 3x3
    auto matmul3x3 = [&](const float A[9], const float B[9], float C[9]){
        for(int row=0; row<3; ++row) {
            for(int col=0; col<3; ++col) {
                C[row*3+col] =
                    A[row*3+0]*B[0*3+col] +
                    A[row*3+1]*B[1*3+col] +
                    A[row*3+2]*B[2*3+col];
            }
        }
    };

    float Rtemp[9], Rfinal[9];
    matmul3x3(Rz, Ry, Rtemp);    // Rz * Ry
    matmul3x3(Rtemp, Rx, Rfinal); // (Rz*Ry)*Rx

    Mat3 out;
    for(int i=0; i<9; i++){
        out.m[i] = Rfinal[i];
    }
    return out;
}

//----------------------------------------------
// 4) Load Camera Configuration
//----------------------------------------------
std::vector<CameraInfo> load_camera_config(const std::string &json_path) {
    std::vector<CameraInfo> cameras;

    std::ifstream ifs(json_path);
    if(!ifs.is_open()){
        std::cerr << "ERROR: Cannot open " << json_path << std::endl;
        return cameras;
    }
    
    json j;
    ifs >> j;
    
    // Helper function to parse a single camera entry
    auto parse_camera_entry = [](const json& entry) -> CameraInfo {
        CameraInfo ci;
        ci.camera_index = entry.value("camera_index", 0);
        
        // Determine camera type
        std::string type_str = entry.value("camera_type", "rtsp");
        if (type_str == "oak_d_pro") {
            ci.camera_type = CameraType::OAK_D_PRO;
            ci.device_id = entry.value("device_id", "");
            ci.use_depth = entry.value("use_depth", false);
            ci.rgb_resolution_width = entry.value("rgb_resolution_width", 1920);
            ci.rgb_resolution_height = entry.value("rgb_resolution_height", 1080);
            ci.fps = entry.value("fps", 30);
            // For OAK-D Pro, use appropriate FOV based on sensor variant
            ci.fov_degrees = entry.value("fov_degrees", 69.f); // Default for IMX378
        } else {
            ci.camera_type = CameraType::RTSP;
            ci.rtsp_url = entry.value("rtsp_url", "");
            ci.fov_degrees = entry.value("fov_degrees", 60.f);
            // Set default values for OAK-D Pro fields
            ci.use_depth = false;
            ci.rgb_resolution_width = 1920;
            ci.rgb_resolution_height = 1080;
            ci.fps = 30;
        }
        
        ci.yaw = entry.value("yaw", 0.f);
        ci.pitch = entry.value("pitch", 0.f);
        ci.roll = entry.value("roll", 0.f);

        // camera_position array
        ci.camera_position = {0.f, 0.f, 0.f}; // default
        if (entry.contains("camera_position") && entry["camera_position"].is_array()) {
            auto arr = entry["camera_position"];
            if (arr.size() >= 3) {
                ci.camera_position.x = arr[0].get<float>();
                ci.camera_position.y = arr[1].get<float>();
                ci.camera_position.z = arr[2].get<float>();
            }
        }
        return ci;
    };

    // Check if JSON is an array (multiple cameras) or object (single camera)
    if (j.is_array()) {
        for (const auto &entry : j) {
            cameras.push_back(parse_camera_entry(entry));
        }
    } else if (j.is_object()) {
        cameras.push_back(parse_camera_entry(j));
    } else {
        std::cerr << "ERROR: JSON format not recognized.\n";
    }

    return cameras;
}

//----------------------------------------------
// 5) Image Processing & Motion Detection
//----------------------------------------------
struct ImageGray {
    int width;
    int height;
    std::vector<float> pixels;  // grayscale float
};

// Convert OpenCV Mat to our ImageGray structure
ImageGray convert_mat_to_gray(const cv::Mat &frame) {
    ImageGray img;

    // Fast path: convert in OpenCV vectorised code
    cv::Mat gray8;
    if (frame.channels() == 1) {
        gray8 = frame; // no copy
    } else {
        cv::cvtColor(frame, gray8, cv::COLOR_BGR2GRAY);
    }

    cv::Mat gray32f;
    gray8.convertTo(gray32f, CV_32F); // 0..255 floats

    img.width  = gray32f.cols;
    img.height = gray32f.rows;
    const float* startPtr = gray32f.ptr<float>();
    const float* endPtr   = startPtr + gray32f.total();
    img.pixels.assign(startPtr, endPtr);
    return img;
}

// Detect motion by absolute difference
// Returns a boolean mask + the difference for each pixel
struct MotionMask {
    int width;
    int height;
    std::vector<bool> changed;
    std::vector<float> diff; // absolute difference
};

MotionMask detect_motion(const ImageGray &prev, const ImageGray &next, float threshold) {
    MotionMask mm;
    if(prev.width != next.width || prev.height != next.height) {
        std::cerr << "Images differ in size. Can't do motion detection!\n";
        mm.width = 0;
        mm.height = 0;
        return mm;
    }
    mm.width = prev.width;
    mm.height = prev.height;
    mm.changed.resize(mm.width * mm.height, false);
    mm.diff.resize(mm.width * mm.height, 0.f);

    for(int i=0; i < mm.width*mm.height; i++){
        float d = std::fabs(prev.pixels[i] - next.pixels[i]);
        mm.diff[i] = d;
        mm.changed[i] = (d > threshold);
    }
    return mm;
}

//----------------------------------------------
// 6) Voxel DDA
//----------------------------------------------
struct RayStep {
    int ix, iy, iz;
    int step_count;
    float distance;
};

static inline float safe_div(float num, float den) {
    float eps = 1e-12f;
    if(std::fabs(den) < eps) {
        return std::numeric_limits<float>::infinity();
    }
    return num / den;
}

std::vector<RayStep> cast_ray_into_grid(
    const Vec3 &camera_pos, 
    const Vec3 &dir_normalized, 
    int N, 
    float voxel_size, 
    const Vec3 &grid_center)
{
    std::vector<RayStep> steps;
    steps.reserve(64);

    float half_size = 0.5f * (N * voxel_size);
    Vec3 grid_min = { grid_center.x - half_size,
                      grid_center.y - half_size,
                      grid_center.z - half_size };
    Vec3 grid_max = { grid_center.x + half_size,
                      grid_center.y + half_size,
                      grid_center.z + half_size };

    float t_min = 0.f;
    float t_max = std::numeric_limits<float>::infinity();

    // 1) Ray-box intersection
    for(int i=0; i<3; i++){
        float origin = (i==0)? camera_pos.x : ((i==1)? camera_pos.y : camera_pos.z);
        float d      = (i==0)? dir_normalized.x : ((i==1)? dir_normalized.y : dir_normalized.z);
        float mn     = (i==0)? grid_min.x : ((i==1)? grid_min.y : grid_min.z);
        float mx     = (i==0)? grid_max.x : ((i==1)? grid_max.y : grid_max.z);

        if(std::fabs(d) < 1e-12f){
            if(origin < mn || origin > mx){
                return steps; // no intersection
            }
        } else {
            float t1 = (mn - origin)/d;
            float t2 = (mx - origin)/d;
            float t_near = std::fmin(t1, t2);
            float t_far  = std::fmax(t1, t2);
            if(t_near > t_min) t_min = t_near;
            if(t_far  < t_max) t_max = t_far;
            if(t_min > t_max){
                return steps;
            }
        }
    }

    if(t_min < 0.f) t_min = 0.f;

    // 2) Start voxel
    Vec3 start_world = { camera_pos.x + t_min*dir_normalized.x,
                         camera_pos.y + t_min*dir_normalized.y,
                         camera_pos.z + t_min*dir_normalized.z };
    float fx = (start_world.x - grid_min.x)/voxel_size;
    float fy = (start_world.y - grid_min.y)/voxel_size;
    float fz = (start_world.z - grid_min.z)/voxel_size;

    int ix = int(fx);
    int iy = int(fy);
    int iz = int(fz);
    if(ix<0 || ix>=N || iy<0 || iy>=N || iz<0 || iz>=N) {
        return steps;
    }

    // 3) Step direction
    int step_x = (dir_normalized.x >= 0.f)? 1 : -1;
    int step_y = (dir_normalized.y >= 0.f)? 1 : -1;
    int step_z = (dir_normalized.z >= 0.f)? 1 : -1;

    auto boundary_in_world_x = [&](int i_x){ return grid_min.x + i_x*voxel_size; };
    auto boundary_in_world_y = [&](int i_y){ return grid_min.y + i_y*voxel_size; };
    auto boundary_in_world_z = [&](int i_z){ return grid_min.z + i_z*voxel_size; };

    int nx_x = ix + (step_x>0?1:0);
    int nx_y = iy + (step_y>0?1:0);
    int nx_z = iz + (step_z>0?1:0);

    float next_bx = boundary_in_world_x(nx_x);
    float next_by = boundary_in_world_y(nx_y);
    float next_bz = boundary_in_world_z(nx_z);

    float t_max_x = safe_div(next_bx - camera_pos.x, dir_normalized.x);
    float t_max_y = safe_div(next_by - camera_pos.y, dir_normalized.y);
    float t_max_z = safe_div(next_bz - camera_pos.z, dir_normalized.z);

    float t_delta_x = safe_div(voxel_size, std::fabs(dir_normalized.x));
    float t_delta_y = safe_div(voxel_size, std::fabs(dir_normalized.y));
    float t_delta_z = safe_div(voxel_size, std::fabs(dir_normalized.z));

    float t_current = t_min;
    int step_count = 0;

    // 4) Walk
    while(t_current <= t_max){
        RayStep rs;
        rs.ix = ix; 
        rs.iy = iy; 
        rs.iz = iz;
        rs.step_count = step_count;
        rs.distance = t_current;

        steps.push_back(rs);

        if(t_max_x < t_max_y && t_max_x < t_max_z){
            ix += step_x;
            t_current = t_max_x;
            t_max_x += t_delta_x;
        } else if(t_max_y < t_max_z){
            iy += step_y;
            t_current = t_max_y;
            t_max_y += t_delta_y;
        } else {
            iz += step_z;
            t_current = t_max_z;
            t_max_z += t_delta_z;
        }
        step_count++;
        if(ix<0 || ix>=N || iy<0 || iy>=N || iz<0 || iz>=N){
            break;
        }
    }

    return steps;
}

//----------------------------------------------
// 7) Camera Stream Thread
//----------------------------------------------
void camera_stream_thread(
    const CameraInfo& camera_info,
    std::shared_ptr<FrameQueue> frame_queue,
    std::atomic<bool>& running
) {
    // Force OpenCV to use the FFmpeg backend instead of GStreamer for RTSP.
    cv::VideoCapture cap(camera_info.rtsp_url, cv::CAP_FFMPEG);
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Failed to open RTSP stream: " << camera_info.rtsp_url << std::endl;
        return;
    }

    // Keep latency down (supported on recent OpenCV/FFmpeg builds)
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    cap.set(cv::CAP_PROP_OPEN_TIMEOUT_MSEC, 5000);
    cap.set(cv::CAP_PROP_READ_TIMEOUT_MSEC, 5000);

    // Set capture properties if needed
    // cap.set(cv::CAP_PROP_BUFFERSIZE, 1);  // Reduce buffer size for lower latency

    std::cout << "Camera " << camera_info.camera_index << " connected to: " << camera_info.rtsp_url << std::endl;

    cv::Mat frame;
    while (running) {
        if (cap.read(frame)) {
            if (frame.empty()) {
                std::cerr << "WARNING: Empty frame received from camera " << camera_info.camera_index << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // Create frame data and push to queue
            FrameData frameData;
            frameData.camera_index = camera_info.camera_index;
            frameData.frame = frame.clone();
            frameData.timestamp = std::chrono::steady_clock::now();
            
            frame_queue->push(frameData);
        } else {
            std::cerr << "ERROR: Failed to read frame from camera " << camera_info.camera_index << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // Attempt to reconnect
            cap.release();
            std::this_thread::sleep_for(std::chrono::seconds(1));
            cap.open(camera_info.rtsp_url);
            if (!cap.isOpened()) {
                std::cerr << "ERROR: Failed to reconnect to RTSP stream: " << camera_info.rtsp_url << std::endl;
            } else {
                std::cout << "Camera " << camera_info.camera_index << " reconnected." << std::endl;
            }
        }
    }
    
    cap.release();
    std::cout << "Camera " << camera_info.camera_index << " thread stopped." << std::endl;
}

#ifdef HAVE_DEPTHAI
//----------------------------------------------
// 7.1) OAK-D Pro Camera Stream Thread
//----------------------------------------------
void oak_d_pro_stream_thread(
    const CameraInfo& camera_info,
    std::shared_ptr<FrameQueue> frame_queue,
    std::atomic<bool>& running
) {
    try {
        // Create pipeline
        dai::Pipeline pipeline;

        // Define source and output
        auto camRgb = pipeline.create<dai::node::ColorCamera>();
        auto rgbOut = pipeline.create<dai::node::XLinkOut>();
        
        rgbOut->setStreamName("rgb");

        // Properties
        camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        if (camera_info.rgb_resolution_width == 4056 && camera_info.rgb_resolution_height == 3040) {
            camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_12_MP);
        } else if (camera_info.rgb_resolution_width == 1920 && camera_info.rgb_resolution_height == 1080) {
            camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        } else if (camera_info.rgb_resolution_width == 1280 && camera_info.rgb_resolution_height == 720) {
            camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
        }
        
        camRgb->setInterleaved(false);
        camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
        camRgb->setFps(camera_info.fps);

        // Linking
        camRgb->video.link(rgbOut->input);

        // Connect to device and start pipeline
        dai::Device device;
        if (!camera_info.device_id.empty()) {
            device = dai::Device(pipeline, camera_info.device_id);
        } else {
            device = dai::Device(pipeline);
        }

        // Output queue will be used to get the rgb frames from the output defined above
        auto q = device.getOutputQueue("rgb", 4, false);

        std::cout << "OAK-D Pro Camera " << camera_info.camera_index << " connected successfully." << std::endl;

        while (running) {
            auto inRgb = q->get<dai::ImgFrame>();
            if (inRgb) {
                // Convert from DepthAI format to OpenCV Mat
                cv::Mat frame = inRgb->getCvFrame();
                
                if (frame.empty()) {
                    std::cerr << "WARNING: Empty frame received from OAK-D Pro camera " 
                              << camera_info.camera_index << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }

                // Create frame data and push to queue
                FrameData frameData;
                frameData.camera_index = camera_info.camera_index;
                frameData.frame = frame.clone();
                frameData.timestamp = std::chrono::steady_clock::now();
                
                frame_queue->push(frameData);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        
        std::cout << "OAK-D Pro Camera " << camera_info.camera_index << " thread stopped." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "ERROR: OAK-D Pro Camera " << camera_info.camera_index 
                  << " failed: " << e.what() << std::endl;
    }
}
#endif

//----------------------------------------------
// 8) Ray Intersection Structure for Stereo
//----------------------------------------------
struct MotionRay {
    Vec3 origin;
    Vec3 direction;
    float intensity;
    int camera_index;
    std::chrono::steady_clock::time_point timestamp;
};

struct CameraMotionData {
    int camera_index;
    std::vector<MotionRay> motion_rays;
    std::chrono::steady_clock::time_point timestamp;
};

//----------------------------------------------
// 9) Stereo Processing Thread
//----------------------------------------------
void processing_thread(
    std::map<int, CameraInfo> camera_info_map,
    std::shared_ptr<FrameQueue> frame_queue,
    std::mutex& voxel_grid_mutex,
    std::vector<float>& voxel_grid,
    const int N,
    const float voxel_size,
    const Vec3& grid_center,
    float motion_threshold,
    float alpha,  // distance-based attenuation
    std::atomic<bool>& running,
#ifdef HAVE_ZMQ
    zmq::socket_t* pub_socket,
#else
    void* /*unused*/,
#endif
    int save_interval_seconds = 30
) {
    // Map to store the previous frame for each camera
    std::map<int, ImageGray> prev_frames;
    std::map<int, std::chrono::steady_clock::time_point> last_frame_times;
    
    // Buffer for stereo motion data - stores motion rays from each camera
    std::map<int, CameraMotionData> motion_buffer;
    const int max_time_diff_ms = 500; // Maximum time difference between cameras for stereo matching
    
    auto last_save_time = std::chrono::system_clock::now();
    
    while (running) {
        FrameData frame_data;
        if (frame_queue->waitAndPop(frame_data, 100)) {
            int camera_index = frame_data.camera_index;
            
            // Convert to our image format
            ImageGray curr_img = convert_mat_to_gray(frame_data.frame);
            
            // Check if we have a previous frame for this camera
            if (prev_frames.find(camera_index) == prev_frames.end()) {
                // First frame for this camera
                prev_frames[camera_index] = curr_img;
                last_frame_times[camera_index] = frame_data.timestamp;
                continue;
            }
            
            // Compute time difference between frames
            auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                frame_data.timestamp - last_frame_times[camera_index]).count();
            
            // Skip if frames are too close together (optional - helps with slower streams)
            if (time_diff < 30) { // 30ms minimum between processed frames
                continue;
            }
            
            // Detect motion
            auto& prev_img = prev_frames[camera_index];
            MotionMask mm = detect_motion(prev_img, curr_img, motion_threshold);
            
            if (mm.width > 0 && mm.height > 0) {
                // Get camera info
                const auto& cam_info = camera_info_map[camera_index];
                
                // Use the current frame's camera info for ray-casting
                Vec3 cam_pos = cam_info.camera_position;
                Mat3 cam_rot = rotation_matrix_yaw_pitch_roll(cam_info.yaw, cam_info.pitch, cam_info.roll);
                // --- camera intrinsics ---
                float fov_h_rad = deg2rad(cam_info.fov_degrees);
                float fx = (mm.width * 0.5f) / std::tan(fov_h_rad * 0.5f);
                float aspect = float(mm.width) / float(mm.height);
                float fov_v_rad = 2.f * std::atan((1.f / aspect) * std::tan(fov_h_rad * 0.5f));
                float fy = (mm.height * 0.5f) / std::tan(fov_v_rad * 0.5f);

                // Collect motion rays for this camera
                CameraMotionData cmd;
                cmd.camera_index = camera_index;
                cmd.timestamp = frame_data.timestamp;
                cmd.motion_rays.clear();
                
                for (int v = 0; v < mm.height; v++) {
                    for (int u = 0; u < mm.width; u++) {
                        if (!mm.changed[v * mm.width + u])
                            continue; // skip if no motion

                        float pix_val = mm.diff[v * mm.width + u];
                        if (pix_val < 1e-3f)
                            continue;

                        // Build local camera direction
                        float x = (float(u) - 0.5f * mm.width)  / fx;
                        float y = -(float(v) - 0.5f * mm.height) / fy;
                        Vec3 ray_cam = { x, y, -1.f };
                        ray_cam = normalize(ray_cam);

                        // Transform to world
                        Vec3 ray_world = mat3_mul_vec3(cam_rot, ray_cam);
                        ray_world = normalize(ray_world);

                        // Store the motion ray
                        MotionRay ray;
                        ray.origin = cam_pos;
                        ray.direction = ray_world;
                        ray.intensity = pix_val;
                        ray.camera_index = camera_index;
                        ray.timestamp = frame_data.timestamp;
                        cmd.motion_rays.push_back(ray);
                    }
                }
                
                // Store motion data for this camera
                motion_buffer[camera_index] = cmd;
                
                if (cmd.motion_rays.size() > 0) {
                    std::cout << "[DEBUG] Camera " << camera_index << " detected " 
                              << cmd.motion_rays.size() << " motion rays" << std::endl;
                }
                
                // Check if we have recent motion data from other cameras for stereo processing
                bool has_stereo_data = false;
                for (const auto& other_cam : camera_info_map) {
                    int other_cam_id = other_cam.first;
                    if (other_cam_id == camera_index) continue;
                    
                    if (motion_buffer.find(other_cam_id) != motion_buffer.end()) {
                        auto time_diff_stereo = std::chrono::duration_cast<std::chrono::milliseconds>(
                            cmd.timestamp - motion_buffer[other_cam_id].timestamp).count();
                        
                        if (std::abs(time_diff_stereo) < max_time_diff_ms) {
                            has_stereo_data = true;
                            break;
                        }
                    }
                }
                
                // Process stereo reconstruction if we have synchronized data
                if (has_stereo_data && motion_buffer.size() >= 2) {
                    std::cout << "[DEBUG] Processing stereo with " << motion_buffer.size() 
                              << " cameras" << std::endl;
                    
                    // More efficient approach: find ray intersections between camera pairs
                    std::vector<float> intersection_grid(N*N*N, 0.f);
                    bool any_intersection = false;
                    int total_cameras_with_evidence = 0;
                    int intersection_count = 0;
                    
                    // Get list of cameras with recent data
                    std::vector<std::pair<int, const CameraMotionData*>> active_cameras;
                    for (const auto& cam_data : motion_buffer) {
                        auto time_diff_cam = std::chrono::duration_cast<std::chrono::milliseconds>(
                            cmd.timestamp - cam_data.second.timestamp).count();
                        if (std::abs(time_diff_cam) <= max_time_diff_ms) {
                            active_cameras.push_back({cam_data.first, &cam_data.second});
                        }
                    }
                    
                    std::cout << "[DEBUG] Found " << active_cameras.size() << " active cameras" << std::endl;
                    
                    // For each pair of cameras, find ray intersections
                    for (size_t i = 0; i < active_cameras.size(); i++) {
                        for (size_t j = i + 1; j < active_cameras.size(); j++) {
                            const auto& cam1_data = *active_cameras[i].second;
                            const auto& cam2_data = *active_cameras[j].second;
                            
                            std::cout << "[DEBUG] Checking " << cam1_data.motion_rays.size() 
                                      << " vs " << cam2_data.motion_rays.size() << " rays" << std::endl;
                            
                            // Sample rays to avoid O(n²) complexity - much more selective
                            int max_rays_to_check = 200;  // Reduced from 1000
                            int ray1_step = std::max(1, (int)cam1_data.motion_rays.size() / max_rays_to_check);
                            int ray2_step = std::max(1, (int)cam2_data.motion_rays.size() / max_rays_to_check);
                            
                            for (size_t r1 = 0; r1 < cam1_data.motion_rays.size(); r1 += ray1_step) {
                                for (size_t r2 = 0; r2 < cam2_data.motion_rays.size(); r2 += ray2_step) {
                                    const auto& ray1 = cam1_data.motion_rays[r1];
                                    const auto& ray2 = cam2_data.motion_rays[r2];
                                    
                                    // Only process strong motion rays
                                    if (ray1.intensity < 30.0f || ray2.intensity < 30.0f) continue;
                                    
                                    // Find closest approach between two rays
                                    Vec3 w0 = {ray1.origin.x - ray2.origin.x,
                                              ray1.origin.y - ray2.origin.y, 
                                              ray1.origin.z - ray2.origin.z};
                                    
                                    float a = ray1.direction.x * ray1.direction.x + 
                                             ray1.direction.y * ray1.direction.y + 
                                             ray1.direction.z * ray1.direction.z;
                                    float b = ray1.direction.x * ray2.direction.x + 
                                             ray1.direction.y * ray2.direction.y + 
                                             ray1.direction.z * ray2.direction.z;
                                    float c = ray2.direction.x * ray2.direction.x + 
                                             ray2.direction.y * ray2.direction.y + 
                                             ray2.direction.z * ray2.direction.z;
                                    float d = ray1.direction.x * w0.x + 
                                             ray1.direction.y * w0.y + 
                                             ray1.direction.z * w0.z;
                                    float e = ray2.direction.x * w0.x + 
                                             ray2.direction.y * w0.y + 
                                             ray2.direction.z * w0.z;
                                    
                                    float denom = a * c - b * b;
                                    if (std::abs(denom) < 1e-10) continue; // Parallel rays
                                    
                                    float t1 = (b * e - c * d) / denom;
                                    float t2 = (a * e - b * d) / denom;
                                    
                                    if (t1 < 0 || t2 < 0) continue; // Behind cameras
                                    
                                    // Points of closest approach
                                    Vec3 p1 = {ray1.origin.x + t1 * ray1.direction.x,
                                              ray1.origin.y + t1 * ray1.direction.y,
                                              ray1.origin.z + t1 * ray1.direction.z};
                                    Vec3 p2 = {ray2.origin.x + t2 * ray2.direction.x,
                                              ray2.origin.y + t2 * ray2.direction.y,
                                              ray2.origin.z + t2 * ray2.direction.z};
                                    
                                    // Distance between closest points
                                    float dist = std::sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                                                          (p1.y - p2.y) * (p1.y - p2.y) +
                                                          (p1.z - p2.z) * (p1.z - p2.z));
                                    
                                    // If rays intersect closely, add to voxel grid
                                    if (dist < voxel_size * 1.5f) { // Much stricter intersection threshold
                                        Vec3 intersection = {(p1.x + p2.x) * 0.5f,
                                                           (p1.y + p2.y) * 0.5f,
                                                           (p1.z + p2.z) * 0.5f};
                                        
                                        // Convert to voxel coordinates
                                        int ix = (int)((intersection.x - grid_center.x) / voxel_size + N/2.0f);
                                        int iy = (int)((intersection.y - grid_center.y) / voxel_size + N/2.0f);
                                        int iz = (int)((intersection.z - grid_center.z) / voxel_size + N/2.0f);
                                        
                                        // Debug: Print a few intersection coordinates
                                        static int debug_count = 0;
                                        if (debug_count < 5) {
                                            std::cout << "[DEBUG] Intersection at (" << intersection.x << ", " 
                                                      << intersection.y << ", " << intersection.z 
                                                      << ") -> voxel (" << ix << ", " << iy << ", " << iz << ")" << std::endl;
                                            debug_count++;
                                        }
                                        
                                        if (ix >= 0 && ix < N && iy >= 0 && iy < N && iz >= 0 && iz < N) {
                                            int idx = ix * N * N + iy * N + iz;
                                            float weight = 1.0f / (1.0f + dist * dist / (voxel_size * voxel_size));
                                            float intensity = (ray1.intensity + ray2.intensity) * weight;
                                            intersection_grid[idx] += intensity;
                                            any_intersection = true;
                                            intersection_count++;
                                            
                                            // Debug: Count successful voxel updates
                                            static int voxel_update_count = 0;
                                            if (voxel_update_count < 3) {
                                                std::cout << "[DEBUG] Updated voxel (" << ix << ", " << iy << ", " << iz 
                                                          << ") with intensity " << intensity << std::endl;
                                                voxel_update_count++;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    
                    std::cout << "[DEBUG] Found " << intersection_count << " ray intersections" << std::endl;
                    
                    // Apply intersection results to main voxel grid
                    if (any_intersection) {
                        std::lock_guard<std::mutex> lock(voxel_grid_mutex);
                        for (int i = 0; i < N*N*N; i++) {
                            if (intersection_grid[i] > 0) {
                                voxel_grid[i] += intersection_grid[i] * alpha; // Scale by attenuation factor
                            }
                        }
                        
                        auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                        std::cout << "[" << std::put_time(std::localtime(&t), "%H:%M:%S")
                                  << "] Stereo intersection: " << intersection_count 
                                  << " intersections found, updated voxels." << std::endl;
                    }
                    
                    // Clean up old motion data (keep only recent)
                    auto now = std::chrono::steady_clock::now();
                    for (auto it = motion_buffer.begin(); it != motion_buffer.end();) {
                        auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - it->second.timestamp).count();
                        if (age_ms > max_time_diff_ms * 3) { // Keep data for 3x the sync window
                            it = motion_buffer.erase(it);
                        } else {
                            ++it;
                        }
                    }
                }
            }
            
#ifdef HAVE_ZMQ
            // Publish compressed voxel grid occasionally for live viewer
            static int frame_count = 0;
            if (++frame_count % 30 == 0 && pub_socket && pub_socket->handle() != nullptr) {
                std::lock_guard<std::mutex> grid_lock(voxel_grid_mutex);

                struct Meta { int N; float voxel_size; } meta{N, voxel_size};

                // Compress voxel vector (fast)
                uLong src_len = voxel_grid.size() * sizeof(float);
                uLong dst_len = compressBound(src_len);
                std::vector<uint8_t> compressed(dst_len);
                if (compress2(compressed.data(), &dst_len,
                              reinterpret_cast<const Bytef*>(voxel_grid.data()),
                              src_len, Z_BEST_SPEED) == Z_OK) {
                    compressed.resize(dst_len);

                    zmq::message_t m_meta(sizeof(meta));
                    memcpy(m_meta.data(), &meta, sizeof(meta));

                    zmq::message_t m_data(compressed.size());
                    memcpy(m_data.data(), compressed.data(), compressed.size());

                    pub_socket->send(m_meta, zmq::send_flags::sndmore);
                    pub_socket->send(m_data, zmq::send_flags::dontwait);
                }
            }
#endif
            
            // Update previous frame and timestamp
            prev_frames[camera_index] = curr_img;
            last_frame_times[camera_index] = frame_data.timestamp;
        }
        
        // Check if it's time to save the voxel grid
        auto now = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_save_time).count();
        
        if (elapsed >= save_interval_seconds) {
            // Save voxel grid
            {
                std::lock_guard<std::mutex> lock(voxel_grid_mutex);
                
                std::string output_bin = "voxel_grid_" + 
                    std::to_string(std::chrono::system_clock::to_time_t(now)) + ".bin";
                
                std::ofstream ofs(output_bin, std::ios::binary);
                if (!ofs) {
                    std::cerr << "Cannot open output file: " << output_bin << "\n";
                } else {
                    // Write metadata (N, voxel_size)
                    ofs.write(reinterpret_cast<const char*>(&N), sizeof(int));
                    ofs.write(reinterpret_cast<const char*>(&voxel_size), sizeof(float));
                    // Write the data
                    ofs.write(reinterpret_cast<const char*>(voxel_grid.data()), voxel_grid.size() * sizeof(float));
                    ofs.close();
                    std::cout << "Saved voxel grid to " << output_bin << "\n";
                }
            }
            
            last_save_time = now;
        }
    }
}

//----------------------------------------------
// 9) Main Function
//----------------------------------------------
int main(int argc, char** argv) {
    setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp|stimeout;5000000|max_delay;0", 1);

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <camera_config.json> [output_dir] [save_interval_seconds]\n";
        std::cerr << "camera_config.json: JSON file with camera parameters and RTSP URLs\n";
        std::cerr << "output_dir: Directory to save voxel grids (default: .)\n";
        std::cerr << "save_interval_seconds: How often to save voxel grid (default: 30 seconds)\n";
        return 1;
    }
    
    std::string config_path = argv[1];
    std::string output_dir = (argc > 2) ? argv[2] : ".";
    int save_interval = (argc > 3) ? std::stoi(argv[3]) : 30;

    // Create output directory if it doesn't exist
    std::string mkdir_cmd = "mkdir -p " + output_dir;
    system(mkdir_cmd.c_str());

    //------------------------------------------
    // 9.1) Load camera configuration
    //------------------------------------------
    std::vector<CameraInfo> cameras = load_camera_config(config_path);
    if (cameras.empty()) {
        std::cerr << "No cameras configured.\n";
        return 1;
    }

    std::map<int, CameraInfo> camera_info_map;
    for (const auto& camera : cameras) {
        camera_info_map[camera.camera_index] = camera;
    }

    std::cout << "Loaded " << cameras.size() << " camera configurations\n";

    //------------------------------------------
    // 9.2) Create a 3D voxel grid
    //------------------------------------------
    const int N = 500;
    const float voxel_size = 6.f;
    // Center of the voxel grid
    Vec3 grid_center = {0.f, 0.f, 500.f};

    std::vector<float> voxel_grid(N*N*N, 0.f);
    std::mutex voxel_grid_mutex;

    //------------------------------------------
    // 9.3) Start camera and processing threads
    //------------------------------------------
    std::atomic<bool> running(true);
    auto frame_queue = std::make_shared<FrameQueue>();
    
    std::vector<std::thread> camera_threads;
    for (const auto& camera : cameras) {
        if (camera.camera_type == CameraType::RTSP) {
            camera_threads.emplace_back(camera_stream_thread, 
                                       camera, 
                                       frame_queue, 
                                       std::ref(running));
        } 
#ifdef HAVE_DEPTHAI
        else if (camera.camera_type == CameraType::OAK_D_PRO) {
            camera_threads.emplace_back(oak_d_pro_stream_thread, 
                                       camera, 
                                       frame_queue, 
                                       std::ref(running));
        }
#endif
        else {
            std::cerr << "ERROR: Unknown camera type for camera " << camera.camera_index << std::endl;
        }
    }
    
    // Processing thread
    float motion_threshold = 25.0f;  // much higher threshold to reduce noise
    float alpha = 0.1f;            // distance-based attenuation
    
#ifdef HAVE_ZMQ
    zmq::context_t zmq_ctx(1);
    zmq::socket_t  zmq_pub(zmq_ctx, zmq::socket_type::pub);

    const char* portEnv = std::getenv("ZMQ_PORT");
    std::string port    = portEnv ? portEnv : "5556";
    std::string address = "tcp://127.0.0.1:" + port;

    try {
        zmq_pub.bind(address);
        std::cout << "ZeroMQ publisher bound to " << address << '\n';
    } catch(const zmq::error_t& e) {
        std::cerr << "ERROR: cannot bind ZeroMQ socket: " << e.what()
                  << "  (" << address << ")\n";
        return 2;
    }
#endif

    std::thread processor(processing_thread,
                         camera_info_map,
                         frame_queue,
                         std::ref(voxel_grid_mutex),
                         std::ref(voxel_grid),
                         N,
                         voxel_size,
                         grid_center,
                         motion_threshold,
                         alpha,
                         std::ref(running),
#ifdef HAVE_ZMQ
                         &zmq_pub,
#else
                         nullptr,
#endif
                         save_interval);
    
    // Basic console interface
    std::cout << "Real-time RTSP stream processing started.\n";
    std::cout << "Press 'q' to quit, 's' to save current voxel grid.\n";
    
    char cmd;
    while (running) {
        cmd = std::cin.get();
        if (cmd == 'q' || cmd == 'Q') {
            running = false;
        } else if (cmd == 's' || cmd == 'S') {
            // Save the current voxel grid
            std::lock_guard<std::mutex> lock(voxel_grid_mutex);
            
            auto now = std::chrono::system_clock::now();
            std::string output_bin = output_dir + "/voxel_grid_manual_" + 
                std::to_string(std::chrono::system_clock::to_time_t(now)) + ".bin";
            
            std::ofstream ofs(output_bin, std::ios::binary);
            if (!ofs) {
                std::cerr << "Cannot open output file: " << output_bin << "\n";
            } else {
                // Write metadata (N, voxel_size)
                ofs.write(reinterpret_cast<const char*>(&N), sizeof(int));
                ofs.write(reinterpret_cast<const char*>(&voxel_size), sizeof(float));
                // Write the data
                ofs.write(reinterpret_cast<const char*>(voxel_grid.data()), voxel_grid.size() * sizeof(float));
                ofs.close();
                std::cout << "Saved voxel grid to " << output_bin << "\n";
            }
        }
    }
    
    // Wait for threads to finish
    for (auto& thread : camera_threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    if (processor.joinable()) {
        processor.join();
    }
    
    std::cout << "All threads stopped. Exiting.\n";
    return 0;
} 