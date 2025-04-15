#pragma once

#include <string>
#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

class CoTNetworkHandler {
private:
    std::string server_ip;
    int server_port;
    int socket_fd;
    bool connected;
    
    // Thread-safe queue for outgoing messages
    std::queue<std::string> message_queue;
    mutable std::mutex queue_mutex;
    std::condition_variable queue_cv;
    
    // Control flags
    std::atomic<bool> running;
    std::thread send_thread;
    
    // Connect to the server
    bool connectToServer();
    
    // Send a single message
    bool sendMessage(const std::string& message);
    
    // Message sending thread
    void sendThreadFunction();
    
public:
    // Constructor
    CoTNetworkHandler(const std::string& ip = "127.0.0.1", int port = 8087);
    
    // Destructor
    ~CoTNetworkHandler();
    
    // No copy or move
    CoTNetworkHandler(const CoTNetworkHandler&) = delete;
    CoTNetworkHandler& operator=(const CoTNetworkHandler&) = delete;
    CoTNetworkHandler(CoTNetworkHandler&&) = delete;
    CoTNetworkHandler& operator=(CoTNetworkHandler&&) = delete;
    
    // Start the message sending thread
    bool start();
    
    // Stop the message sending thread
    void stop();
    
    // Queue a message to be sent
    void queueMessage(const std::string& message);
    
    // Queue multiple messages to be sent
    void queueMessages(const std::vector<std::string>& messages);
    
    // Set server information
    void setServer(const std::string& ip, int port);
    
    // Get queue size
    size_t getQueueSize() const;
    
    // Check if connected
    bool isConnected() const;
}; 