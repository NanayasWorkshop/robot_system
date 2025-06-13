#pragma once

#include <string>
#include <cstdint>

namespace delta {

/**
 * Simple UDP sender for streaming visualization data
 * Linux-specific implementation using BSD sockets
 */
class NetworkSender {
private:
    int socket_fd_;
    std::string target_ip_;
    uint16_t target_port_;
    bool is_connected_;
    
    // Statistics
    mutable uint64_t total_packets_sent_;
    mutable uint64_t total_bytes_sent_;
    mutable uint64_t failed_sends_;
    
public:
    NetworkSender();
    ~NetworkSender();
    
    // =============================================================================
    // CONNECTION MANAGEMENT
    // =============================================================================
    
    /**
     * Initialize UDP sender
     * @param target_ip Target IP address (default: "127.0.0.1")
     * @param target_port Target port (default: 9999)
     * @return true if successful
     */
    bool initialize(const std::string& target_ip = "127.0.0.1", uint16_t target_port = 9999);
    
    /**
     * Close connection and cleanup
     */
    void close();
    
    /**
     * Check if sender is ready
     * @return true if ready to send
     */
    bool is_ready() const { return is_connected_; }
    
    // =============================================================================
    // DATA TRANSMISSION
    // =============================================================================
    
    /**
     * Send binary data packet
     * @param data Pointer to data buffer
     * @param size Size of data in bytes
     * @return true if sent successfully
     */
    bool send_data(const void* data, size_t size);
    
    /**
     * Send binary data with automatic size detection
     * @param packet Packet structure to send
     * @return true if sent successfully
     */
    template<typename PacketType>
    bool send_packet(const PacketType& packet) {
        return send_data(&packet, sizeof(PacketType));
    }
    
    // =============================================================================
    // STATISTICS AND DIAGNOSTICS
    // =============================================================================
    
    /**
     * Get transmission statistics
     */
    struct NetworkStats {
        uint64_t packets_sent;
        uint64_t bytes_sent;
        uint64_t failed_sends;
        double success_rate;
        
        NetworkStats() : packets_sent(0), bytes_sent(0), failed_sends(0), success_rate(0.0) {}
    };
    
    /**
     * Get current network statistics
     * @return Network transmission stats
     */
    NetworkStats get_statistics() const;
    
    /**
     * Reset statistics counters
     */
    void reset_statistics();
    
    /**
     * Get connection info as string
     * @return Connection information
     */
    std::string get_connection_info() const;

private:
    // =============================================================================
    // INTERNAL METHODS
    // =============================================================================
    
    /**
     * Create and configure UDP socket
     * @return true if successful
     */
    bool create_socket();
    
    /**
     * Configure socket address
     * @return true if successful
     */
    bool configure_address();
    
    /**
     * Update statistics after send attempt
     * @param bytes_sent Number of bytes sent (0 if failed)
     */
    void update_statistics(size_t bytes_sent) const;
};

} // namespace delta