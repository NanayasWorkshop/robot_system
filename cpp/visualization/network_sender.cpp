#include "network_sender.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

namespace delta {

NetworkSender::NetworkSender() 
    : socket_fd_(-1), target_ip_("127.0.0.1"), target_port_(9999), is_connected_(false)
    , total_packets_sent_(0), total_bytes_sent_(0), failed_sends_(0) {
}

NetworkSender::~NetworkSender() {
    close();
}

// =============================================================================
// CONNECTION MANAGEMENT
// =============================================================================

bool NetworkSender::initialize(const std::string& target_ip, uint16_t target_port) {
    // Store connection parameters
    target_ip_ = target_ip;
    target_port_ = target_port;
    
    // Create socket
    if (!create_socket()) {
        std::cerr << "NetworkSender: Failed to create socket" << std::endl;
        return false;
    }
    
    // Configure address
    if (!configure_address()) {
        std::cerr << "NetworkSender: Failed to configure address" << std::endl;
        close();
        return false;
    }
    
    is_connected_ = true;
    
    std::cout << "NetworkSender: Initialized UDP sender to " 
              << target_ip_ << ":" << target_port_ << std::endl;
    
    return true;
}

void NetworkSender::close() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
    is_connected_ = false;
}

// =============================================================================
// DATA TRANSMISSION
// =============================================================================

bool NetworkSender::send_data(const void* data, size_t size) {
    if (!is_connected_ || socket_fd_ < 0) {
        update_statistics(0);
        return false;
    }
    
    // Create destination address
    struct sockaddr_in dest_addr;
    std::memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(target_port_);
    
    if (inet_pton(AF_INET, target_ip_.c_str(), &dest_addr.sin_addr) <= 0) {
        std::cerr << "NetworkSender: Invalid target IP address: " << target_ip_ << std::endl;
        update_statistics(0);
        return false;
    }
    
    // Send data
    ssize_t bytes_sent = sendto(socket_fd_, data, size, 0, 
                               (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    
    if (bytes_sent < 0) {
        std::cerr << "NetworkSender: Failed to send data: " << strerror(errno) << std::endl;
        update_statistics(0);
        return false;
    }
    
    if (static_cast<size_t>(bytes_sent) != size) {
        std::cerr << "NetworkSender: Partial send - expected " << size 
                  << " bytes, sent " << bytes_sent << " bytes" << std::endl;
        update_statistics(static_cast<size_t>(bytes_sent));
        return false;
    }
    
    update_statistics(static_cast<size_t>(bytes_sent));
    return true;
}

// =============================================================================
// STATISTICS AND DIAGNOSTICS
// =============================================================================

NetworkSender::NetworkStats NetworkSender::get_statistics() const {
    NetworkStats stats;
    stats.packets_sent = total_packets_sent_;
    stats.bytes_sent = total_bytes_sent_;
    stats.failed_sends = failed_sends_;
    
    if (total_packets_sent_ + failed_sends_ > 0) {
        stats.success_rate = static_cast<double>(total_packets_sent_) / 
                           (total_packets_sent_ + failed_sends_) * 100.0;
    }
    
    return stats;
}

void NetworkSender::reset_statistics() {
    total_packets_sent_ = 0;
    total_bytes_sent_ = 0;
    failed_sends_ = 0;
}

std::string NetworkSender::get_connection_info() const {
    return "UDP → " + target_ip_ + ":" + std::to_string(target_port_) + 
           " (fd: " + std::to_string(socket_fd_) + 
           ", connected: " + (is_connected_ ? "yes" : "no") + ")";
}

// =============================================================================
// INTERNAL METHODS
// =============================================================================

bool NetworkSender::create_socket() {
    // Create UDP socket
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "NetworkSender: Failed to create socket: " << strerror(errno) << std::endl;
        return false;
    }
    
    // Set socket options for better performance
    int enable = 1;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) < 0) {
        std::cerr << "NetworkSender: Warning - failed to set SO_REUSEADDR" << std::endl;
    }
    
    return true;
}

bool NetworkSender::configure_address() {
    // Validate IP address format
    struct sockaddr_in test_addr;
    if (inet_pton(AF_INET, target_ip_.c_str(), &test_addr.sin_addr) <= 0) {
        std::cerr << "NetworkSender: Invalid IP address format: " << target_ip_ << std::endl;
        return false;
    }
    
    // Validate port range
    if (target_port_ == 0) {
        std::cerr << "NetworkSender: Invalid port number: " << target_port_ << std::endl;
        return false;
    }
    
    return true;
}

void NetworkSender::update_statistics(size_t bytes_sent) const {
    if (bytes_sent > 0) {
        total_packets_sent_++;
        total_bytes_sent_ += bytes_sent;
    } else {
        failed_sends_++;
    }
}

} // namespace delta