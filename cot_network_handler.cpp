#include <string>
#include <vector>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <cstring>

// Socket libraries
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <fcntl.h>

// Socket includes for cross-platform support
#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    #define CLOSE_SOCKET closesocket
    typedef int socklen_t;
#else
    #include <sys/socket.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <netinet/in.h>
    #include <fcntl.h>
    #include <errno.h>
    #define CLOSE_SOCKET close
    #define SOCKET int
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
#endif

class CoTNetworkHandler {
private:
    std::string server_ip;
    int server_port;
    int socket_fd;
    bool connected;
    
    // Thread-safe queue for outgoing messages
    std::queue<std::string> message_queue;
    std::mutex queue_mutex;
    std::condition_variable queue_cv;
    
    // Control flags
    std::atomic<bool> running;
    std::thread send_thread;
    
    // Connect to the server
    bool connectToServer() {
        if (connected) {
            return true; // Already connected
        }

        // Create a socket
        socket_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd == INVALID_SOCKET) {
            std::cerr << "Failed to create socket" << std::endl;
            return false;
        }

        // Set up the server address structure
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(server_port);

        // Convert IPv4 address from text to binary form
        if (inet_pton(AF_INET, server_ip.c_str(), &server_addr.sin_addr) <= 0) {
            std::cerr << "Invalid address/ Address not supported: " << server_ip << std::endl;
            CLOSE_SOCKET(socket_fd);
            socket_fd = INVALID_SOCKET;
            return false;
        }

        // Connect to the server
        if (connect(socket_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "Connection failed to " << server_ip << ":" << server_port << std::endl;
            CLOSE_SOCKET(socket_fd);
            socket_fd = INVALID_SOCKET;
            return false;
        }

        connected = true;
        std::cout << "Connected to " << server_ip << ":" << server_port << std::endl;
        return true;
    }
    
    // Send a single message
    bool sendMessage(const std::string& message) {
        if (!connected) {
            if (!connectToServer()) {
                return false;
            }
        }

        size_t message_length = message.length();
        const char* message_data = message.c_str();
        
        // Send the message
        ssize_t bytes_sent = send(socket_fd, message_data, message_length, 0);
        if (bytes_sent < 0) {
            std::cerr << "Failed to send message" << std::endl;
            // Connection might be lost, try to reconnect next time
            connected = false;
            CLOSE_SOCKET(socket_fd);
            socket_fd = INVALID_SOCKET;
            return false;
        } else if (bytes_sent < static_cast<ssize_t>(message_length)) {
            std::cerr << "Warning: Sent only " << bytes_sent << " out of " << message_length << " bytes" << std::endl;
        }

        return true;
    }
    
    // Message sending thread
    void sendThreadFunction() {
        while (running) {
            std::string message;
            
            // Wait for a message or timeout after 1 second to check running flag
            {
                std::unique_lock<std::mutex> lock(queue_mutex);
                queue_cv.wait_for(lock, std::chrono::seconds(1), [this] {
                    return !message_queue.empty() || !running;
                });
                
                if (!running) {
                    break;
                }
                
                if (message_queue.empty()) {
                    continue;
                }
                
                message = message_queue.front();
                message_queue.pop();
            }
            
            // Try to send the message
            if (!sendMessage(message)) {
                // If sending fails, try to reconnect and retry once
                if (connectToServer()) {
                    if (!sendMessage(message)) {
                        // If it still fails, requeue the message
                        std::lock_guard<std::mutex> lock(queue_mutex);
                        message_queue.push(message);
                    }
                } else {
                    // If reconnection fails, requeue the message
                    std::lock_guard<std::mutex> lock(queue_mutex);
                    message_queue.push(message);
                    
                    // Sleep to avoid constant reconnection attempts
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                }
            }
        }
    }
    
public:
    // Constructor
    CoTNetworkHandler(const std::string& ip = "127.0.0.1", int port = 8087)
        : server_ip(ip),
          server_port(port),
          socket_fd(INVALID_SOCKET),
          connected(false),
          running(false) {
        
        #ifdef _WIN32
        // Initialize Winsock
        WSADATA wsaData;
        int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (result != 0) {
            std::cerr << "WSAStartup failed: " << result << std::endl;
        }
        #endif
    }
    
    // Destructor
    ~CoTNetworkHandler() {
        stop();

        if (socket_fd != INVALID_SOCKET) {
            CLOSE_SOCKET(socket_fd);
        }

        #ifdef _WIN32
        WSACleanup();
        #endif
    }
    
    // Start the message sending thread
    bool start() {
        if (running) {
            return true; // Already started
        }
        
        running = true;
        
        // Start the send thread
        send_thread = std::thread(&CoTNetworkHandler::sendThreadFunction, this);
        
        return true;
    }
    
    // Stop the message sending thread
    void stop() {
        if (!running) {
            return; // Already stopped
        }
        
        // Signal the thread to stop
        running = false;
        queue_cv.notify_all();
        
        // Wait for the thread to finish
        if (send_thread.joinable()) {
            send_thread.join();
        }
        
        // Close the socket
        if (socket_fd != INVALID_SOCKET) {
            CLOSE_SOCKET(socket_fd);
            socket_fd = INVALID_SOCKET;
        }
        
        connected = false;
    }
    
    // Queue a message to be sent
    void queueMessage(const std::string& message) {
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            message_queue.push(message);
        }
        queue_cv.notify_one();
    }
    
    // Queue multiple messages to be sent
    void queueMessages(const std::vector<std::string>& messages) {
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            for (const auto& message : messages) {
                message_queue.push(message);
            }
        }
        queue_cv.notify_one();
    }
    
    // Set server information
    void setServer(const std::string& ip, int port) {
        // Reconnect only if params changed
        if (ip == server_ip && port == server_port) {
            return;
        }
        
        bool was_running = running;
        if (was_running) {
            stop();
        }
        
        server_ip = ip;
        server_port = port;
        
        if (was_running) {
            start();
        }
    }
    
    // Get queue size
    size_t getQueueSize() const {
        std::lock_guard<std::mutex> lock(queue_mutex);
        return message_queue.size();
    }
    
    // Check if connected
    bool isConnected() const {
        return connected;
    }
}; 
}; 