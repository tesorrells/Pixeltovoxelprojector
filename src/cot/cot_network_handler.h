#pragma once

#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>

class CoTNetworkHandler {
public:
    // Constructor: Takes server IP and port
    CoTNetworkHandler(const std::string& ip = "127.0.0.1", int port = 8087);

    // Destructor: Cleans up resources (socket, thread)
    ~CoTNetworkHandler();

    // Start the background sending thread
    // Returns true if successful, false otherwise (e.g., failed initial connection)
    bool start();

    // Stop the background sending thread and disconnect
    void stop();

    // Queue a single message for sending
    void queueMessage(const std::string& message);

    // Queue multiple messages for sending
    void queueMessages(const std::vector<std::string>& messages);

    // Update the target server IP and port
    // Note: This might require reconnecting if called while running.
    void setServer(const std::string& ip, int port);

    // Get the current size of the outgoing message queue
    size_t getQueueSize() const; // Made const

    // Check if the handler is currently connected to the server
    bool isConnected() const; // Made const

private:
    // Connect to the server (internal helper)
    bool connectToServer();

    // Send a single message (internal helper)
    bool sendMessage(const std::string& message);

    // Background thread function for sending messages from the queue
    void sendThreadFunction();

    std::string server_ip_;
    int server_port_;
    
    // Socket related members (platform-dependent types)
#ifdef _WIN32
    // Windows specific socket types (e.g., SOCKET)
    void* socket_fd_; // Use void* or include Winsock2.h here carefully
#else
    // POSIX specific socket types
    int socket_fd_;
#endif
    
    std::atomic<bool> connected_;

    // Thread-safe queue for outgoing messages
    std::queue<std::string> message_queue_;
    mutable std::mutex queue_mutex_; // Mutable for const methods like getQueueSize
    std::condition_variable queue_cv_;

    // Control flags and thread
    std::atomic<bool> running_;
    std::thread send_thread_;
}; 