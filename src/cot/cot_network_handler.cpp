#include "cot/cot_network_handler.h"

#include <iostream>
#include <stdexcept>
#include <chrono>
#include <cstring> // For memset

// Socket includes for cross-platform support
#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    #define CLOSE_SOCKET closesocket
    typedef int socklen_t;
    #define SOCKET_LAST_ERROR WSAGetLastError()
#else
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <netdb.h>
    #include <fcntl.h>
    #include <errno.h>
    #define CLOSE_SOCKET close
    #define SOCKET int
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
    #define SOCKET_LAST_ERROR errno
#endif

// --- Constructor --- 
CoTNetworkHandler::CoTNetworkHandler(const std::string& ip, int port)
    : server_ip_(ip),
      server_port_(port),
#ifdef _WIN32
      socket_fd_(nullptr), // Initialize appropriately for SOCKET type
#else
      socket_fd_(INVALID_SOCKET),
#endif
      connected_(false),
      running_(false) 
{
    #ifdef _WIN32
    // Initialize Winsock
    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (result != 0) {
        // Consider throwing an exception or logging an error
        std::cerr << "WSAStartup failed: " << result << std::endl;
    }
    #endif
}

// --- Destructor --- 
CoTNetworkHandler::~CoTNetworkHandler() {
    stop(); // Ensure thread is stopped and socket is closed

    #ifdef _WIN32
    WSACleanup();
    #endif
}

// --- Public Methods --- 

bool CoTNetworkHandler::start() {
    if (running_) {
        return true; // Already running
    }

    // Attempt initial connection before starting thread
    if (!connectToServer()) {
         std::cerr << "Initial connection failed. Send thread not started." << std::endl;
        // Optionally allow starting anyway and retrying later?
        // return false;
    }

    running_ = true;
    // Detach or join? Join seems safer for cleanup.
    // Consider using std::jthread in C++20
    send_thread_ = std::thread(&CoTNetworkHandler::sendThreadFunction, this);

    return true;
}

void CoTNetworkHandler::stop() {
    if (!running_) {
        return; // Already stopped
    }

    running_ = false;
    queue_cv_.notify_one(); // Wake up the send thread if it's waiting

    if (send_thread_.joinable()) {
        send_thread_.join();
    }

    // Close the socket if it's open
    if (connected_) {
        #ifdef _WIN32
             if (socket_fd_ != nullptr) { CLOSE_SOCKET((SOCKET)socket_fd_); socket_fd_ = nullptr; }
        #else
             if (socket_fd_ != INVALID_SOCKET) { CLOSE_SOCKET(socket_fd_); socket_fd_ = INVALID_SOCKET; }
        #endif
        connected_ = false;
        std::cout << "Disconnected." << std::endl;
    }
}

void CoTNetworkHandler::queueMessage(const std::string& message) {
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        message_queue_.push(message);
    }
    queue_cv_.notify_one(); // Notify the sender thread
}

void CoTNetworkHandler::queueMessages(const std::vector<std::string>& messages) {
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        for (const auto& msg : messages) {
            message_queue_.push(msg);
        }
    }
    queue_cv_.notify_one(); // Notify the sender thread
}

void CoTNetworkHandler::setServer(const std::string& ip, int port) {
    std::lock_guard<std::mutex> lock(queue_mutex_); // Lock queue to prevent sending during change?
    bool was_connected = connected_;
    
    server_ip_ = ip;
    server_port_ = port;
    
    // If we were connected, disconnect to force reconnection with new details
    if (was_connected) {
        connected_ = false;
        #ifdef _WIN32
             if (socket_fd_ != nullptr) { CLOSE_SOCKET((SOCKET)socket_fd_); socket_fd_ = nullptr; }
        #else
             if (socket_fd_ != INVALID_SOCKET) { CLOSE_SOCKET(socket_fd_); socket_fd_ = INVALID_SOCKET; }
        #endif
         std::cout << "Server changed, disconnected. Will reconnect on next send attempt." << std::endl;
    }
}

size_t CoTNetworkHandler::getQueueSize() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return message_queue_.size();
}

bool CoTNetworkHandler::isConnected() const {
    return connected_; // Atomic bool, no lock needed
}

// --- Private Methods --- 

bool CoTNetworkHandler::connectToServer() {
    // This function assumes it's called within a context where thread safety
    // is handled (e.g., by the send thread or during start/stop)
    if (connected_) {
        return true; // Already connected
    }

    // Close previous socket if any (e.g., after a failed attempt)
    #ifdef _WIN32
        if (socket_fd_ != nullptr) { CLOSE_SOCKET((SOCKET)socket_fd_); socket_fd_ = nullptr; }
        SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    #else
        if (socket_fd_ != INVALID_SOCKET) { CLOSE_SOCKET(socket_fd_); socket_fd_ = INVALID_SOCKET; }
        SOCKET sock = socket(AF_INET, SOCK_STREAM, 0);
    #endif

    if (sock == INVALID_SOCKET) {
        std::cerr << "Failed to create socket. Error: " << SOCKET_LAST_ERROR << std::endl;
        return false;
    }

    // Set up the server address structure
    struct sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr)); // Clear the struct
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port_);

    // Convert IPv4 address from text to binary form
    #ifdef _WIN32
        // inet_pton is available in newer Windows SDKs with Winsock2
        int pton_res = inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr);
    #else
        int pton_res = inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr);
    #endif

    if (pton_res <= 0) {
        if (pton_res == 0)
             std::cerr << "Invalid address format: " << server_ip_ << std::endl;
        else
             std::cerr << "inet_pton failed. Error: " << SOCKET_LAST_ERROR << std::endl;
        CLOSE_SOCKET(sock);
        return false;
    }

    // Connect to the server
    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
        std::cerr << "Connection failed to " << server_ip_ << ":" << server_port_
                  << ". Error: " << SOCKET_LAST_ERROR << std::endl;
        CLOSE_SOCKET(sock);
        return false;
    }
    
    // Set socket to non-blocking? Maybe not necessary for simple sender.
    // Consider timeouts on send() instead.

    #ifdef _WIN32
        socket_fd_ = (void*)sock;
    #else
        socket_fd_ = sock;
    #endif
    connected_ = true;
    std::cout << "Connected to " << server_ip_ << ":" << server_port_ << std::endl;
    return true;
}

bool CoTNetworkHandler::sendMessage(const std::string& message) {
    // This function assumes it's called within a context where thread safety
    // is handled (e.g., by the send thread)
    if (!connected_) {
        std::cerr << "Attempted to send while not connected." << std::endl;
        // Attempt to reconnect implicitly?
        if (!connectToServer()) {
             return false; // Reconnection failed
        }
        // If connectToServer succeeded, connected_ is true now.
    }

    size_t message_length = message.length();
    const char* message_data = message.c_str();
    ssize_t total_bytes_sent = 0;

    // Loop to ensure all data is sent (sockets might not send all in one go)
    while (total_bytes_sent < static_cast<ssize_t>(message_length)) {
        #ifdef _WIN32
            ssize_t bytes_sent = send((SOCKET)socket_fd_, message_data + total_bytes_sent, message_length - total_bytes_sent, 0);
        #else
            // Use MSG_NOSIGNAL to prevent SIGPIPE on Linux if the connection breaks
            ssize_t bytes_sent = send(socket_fd_, message_data + total_bytes_sent, message_length - total_bytes_sent, MSG_NOSIGNAL);
        #endif

        if (bytes_sent == SOCKET_ERROR) {
            #ifdef _WIN32
                int error = WSAGetLastError();
                if (error == WSAEWOULDBLOCK) {
                     // This shouldn't happen with default blocking sockets
                     std::cerr << "Send would block (unexpected)." << std::endl;
                     std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Avoid busy-waiting
                     continue;
                }
            #else
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                     // Shouldn't happen with default blocking sockets
                     std::cerr << "Send would block (unexpected)." << std::endl;
                      std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Avoid busy-waiting
                     continue;
                }
            #endif
            std::cerr << "Failed to send message. Error: " << SOCKET_LAST_ERROR << std::endl;
            // Connection might be lost, mark as disconnected
            connected_ = false;
            #ifdef _WIN32
                if (socket_fd_ != nullptr) { CLOSE_SOCKET((SOCKET)socket_fd_); socket_fd_ = nullptr; }
            #else
                if (socket_fd_ != INVALID_SOCKET) { CLOSE_SOCKET(socket_fd_); socket_fd_ = INVALID_SOCKET; }
            #endif
            return false;
        } else if (bytes_sent == 0) {
             std::cerr << "Send returned 0 bytes, connection likely closed by peer." << std::endl;
             connected_ = false;
             #ifdef _WIN32
                if (socket_fd_ != nullptr) { CLOSE_SOCKET((SOCKET)socket_fd_); socket_fd_ = nullptr; }
            #else
                if (socket_fd_ != INVALID_SOCKET) { CLOSE_SOCKET(socket_fd_); socket_fd_ = INVALID_SOCKET; }
            #endif
             return false;
        }
        
        total_bytes_sent += bytes_sent;
    }

    return true;
}

void CoTNetworkHandler::sendThreadFunction() {
    while (running_) {
        std::string message;
        bool got_message = false;

        // Wait for a message or timeout to check running flag
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            if (queue_cv_.wait_for(lock, std::chrono::seconds(1), [this] {
                return !message_queue_.empty() || !running_;
            })) {
                if (!running_) {
                    break; // Exit if stop() was called
                }
                if (!message_queue_.empty()) {
                    message = std::move(message_queue_.front()); // Use move
                    message_queue_.pop();
                    got_message = true;
                }
            }
            // If wait_for timed out, loop continues and checks running_
        }

        if (got_message) {
            // Attempt to send the message
            if (!sendMessage(message)) {
                std::cerr << "Send failed. Attempting to reconnect..." << std::endl;
                // Sleep briefly before potentially retrying connection
                std::this_thread::sleep_for(std::chrono::seconds(2)); 
                
                // Try to reconnect and send again
                if (connectToServer()) {
                    if (!sendMessage(message)) {
                        std::cerr << "Send failed after reconnect. Re-queueing message." << std::endl;
                        // Re-queue the message if sending still fails
                        std::lock_guard<std::mutex> lock(queue_mutex_);
                        message_queue_.push(std::move(message)); // Re-queue moved message
                    } else {
                         std::cout << "Message sent successfully after reconnect." << std::endl;
                    }
                } else {
                    std::cerr << "Reconnect failed. Re-queueing message." << std::endl;
                    // Re-queue the message if reconnection fails
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    message_queue_.push(std::move(message)); // Re-queue moved message
                    
                    // Sleep longer if connection fails repeatedly
                    std::this_thread::sleep_for(std::chrono::seconds(5)); 
                }
            }
        }
    }
    std::cout << "Send thread stopped." << std::endl;
} 