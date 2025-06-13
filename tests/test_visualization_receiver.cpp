#include <iostream>
#include <iomanip>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "../cpp/visualization/visualization_data.hpp"

using namespace delta;

/**
 * Test Visualization Receiver
 * Simple UDP receiver that listens for visualization packets and prints their contents
 */
class TestVisualizationReceiver {
private:
    int socket_fd_;
    uint16_t listen_port_;
    bool is_running_;
    
    // Statistics
    uint64_t total_packets_received_;
    uint64_t robot_packets_;
    uint64_t human_packets_;
    uint64_t collision_packets_;
    uint64_t sync_packets_;
    uint64_t unknown_packets_;
    
public:
    TestVisualizationReceiver(uint16_t port = 9999) 
        : socket_fd_(-1), listen_port_(port), is_running_(false)
        , total_packets_received_(0), robot_packets_(0), human_packets_(0)
        , collision_packets_(0), sync_packets_(0), unknown_packets_(0) {}
    
    ~TestVisualizationReceiver() {
        shutdown();
    }
    
    bool initialize() {
        std::cout << "🎯 Initializing Test Visualization Receiver..." << std::endl;
        
        // Create UDP socket
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            std::cerr << "❌ Failed to create socket: " << strerror(errno) << std::endl;
            return false;
        }
        
        // Set socket options
        int enable = 1;
        if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) < 0) {
            std::cerr << "⚠️  Warning: Failed to set SO_REUSEADDR" << std::endl;
        }
        
        // Bind to listen address
        struct sockaddr_in listen_addr;
        std::memset(&listen_addr, 0, sizeof(listen_addr));
        listen_addr.sin_family = AF_INET;
        listen_addr.sin_addr.s_addr = INADDR_ANY;
        listen_addr.sin_port = htons(listen_port_);
        
        if (bind(socket_fd_, (struct sockaddr*)&listen_addr, sizeof(listen_addr)) < 0) {
            std::cerr << "❌ Failed to bind socket to port " << listen_port_ 
                      << ": " << strerror(errno) << std::endl;
            close(socket_fd_);
            return false;
        }
        
        is_running_ = true;
        std::cout << "✅ Receiver initialized, listening on UDP port " << listen_port_ << std::endl;
        return true;
    }
    
    void shutdown() {
        is_running_ = false;
        if (socket_fd_ >= 0) {
            close(socket_fd_);
            socket_fd_ = -1;
        }
    }
    
    void run_receiver_loop() {
        std::cout << "👂 Starting receiver loop..." << std::endl;
        std::cout << "   Press Ctrl+C to stop" << std::endl;
        std::cout << std::endl;
        
        uint8_t buffer[65536]; // 64KB buffer
        struct sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);
        
        while (is_running_) {
            // Receive packet
            ssize_t bytes_received = recvfrom(socket_fd_, buffer, sizeof(buffer), 0,
                                            (struct sockaddr*)&sender_addr, &sender_len);
            
            if (bytes_received < 0) {
                if (is_running_) {
                    std::cerr << "❌ Failed to receive data: " << strerror(errno) << std::endl;
                }
                break;
            }
            
            if (bytes_received == 0) {
                continue;
            }
            
            // Process received packet
            process_packet(buffer, static_cast<size_t>(bytes_received));
            total_packets_received_++;
            
            // Print statistics every 100 packets
            if (total_packets_received_ % 100 == 0) {
                print_statistics();
            }
        }
    }
    
private:
    
    void process_packet(const uint8_t* data, size_t size) {
        if (size < sizeof(PacketHeader)) {
            std::cerr << "⚠️  Received packet too small for header (" << size << " bytes)" << std::endl;
            unknown_packets_++;
            return;
        }
        
        // Extract header
        PacketHeader header;
        std::memcpy(&header, data, sizeof(PacketHeader));
        
        PacketType packet_type = static_cast<PacketType>(header.packet_type);
        const uint8_t* packet_data = data + sizeof(PacketHeader);
        size_t data_size = size - sizeof(PacketHeader);
        
        // Validate header
        if (header.data_size != data_size) {
            std::cerr << "⚠️  Header data size mismatch: expected " << header.data_size 
                      << ", got " << data_size << std::endl;
        }
        
        // Process based on packet type
        switch (packet_type) {
            case PacketType::ROBOT_CAPSULES:
                process_robot_capsules_packet(packet_data, data_size, header);
                robot_packets_++;
                break;
                
            case PacketType::HUMAN_POSE:
                process_human_pose_packet(packet_data, data_size, header);
                human_packets_++;
                break;
                
            case PacketType::COLLISION_CONTACTS:
                process_collision_contacts_packet(packet_data, data_size, header);
                collision_packets_++;
                break;
                
            case PacketType::FRAME_SYNC:
                process_frame_sync_packet(packet_data, data_size, header);
                sync_packets_++;
                break;
                
            default:
                std::cout << "❓ Unknown packet type: " << header.packet_type << std::endl;
                unknown_packets_++;
                break;
        }
    }
    
    void process_robot_capsules_packet(const uint8_t* data, size_t size, const PacketHeader& header) {
        if (size < sizeof(RobotCapsulesPacket)) {
            std::cerr << "⚠️  Robot capsules packet too small" << std::endl;
            return;
        }
        
        RobotCapsulesPacket packet;
        std::memcpy(&packet, data, sizeof(RobotCapsulesPacket));
        
        std::cout << "🤖 Robot Capsules [Frame " << header.frame_id << "]: " 
                  << packet.capsule_count << " capsules" << std::endl;
        
        for (uint32_t i = 0; i < std::min(packet.capsule_count, 3u); ++i) {
            const auto& cap = packet.capsules[i];
            std::cout << "   Capsule " << i << ": ("
                      << std::fixed << std::setprecision(1)
                      << cap.start_x << "," << cap.start_y << "," << cap.start_z << ") → ("
                      << cap.end_x << "," << cap.end_y << "," << cap.end_z << ") r=" << cap.radius
                      << std::endl;
        }
        if (packet.capsule_count > 3) {
            std::cout << "   ... and " << (packet.capsule_count - 3) << " more capsules" << std::endl;
        }
    }
    
    void process_human_pose_packet(const uint8_t* data, size_t size, const PacketHeader& header) {
        if (size >= sizeof(HumanPosePacket)) {
            HumanPosePacket packet;
            std::memcpy(&packet, data, sizeof(HumanPosePacket));
            
            std::cout << "👤 Human Pose [Frame " << header.frame_id << "]: " 
                      << packet.joint_count << " joints, " << packet.vertex_count << " vertices" << std::endl;
            
            // Show first few joints
            for (uint32_t i = 0; i < std::min(packet.joint_count, 3u); ++i) {
                const auto& joint = packet.joints[i];
                std::cout << "   Joint " << i << ": ("
                          << std::fixed << std::setprecision(1)
                          << joint.x << "," << joint.y << "," << joint.z << ")" << std::endl;
            }
        } else if (size >= sizeof(HumanVerticesPacket)) {
            HumanVerticesPacket packet;
            std::memcpy(&packet, data, sizeof(HumanVerticesPacket));
            
            std::cout << "👤 Human Vertices [Frame " << header.frame_id << "]: batch " 
                      << packet.batch_index << "/" << packet.total_batches 
                      << " (" << packet.vertex_count << " vertices)" << std::endl;
        } else {
            std::cout << "⚠️  Human packet too small (" << size << " bytes)" << std::endl;
        }
    }
    
    void process_collision_contacts_packet(const uint8_t* data, size_t size, const PacketHeader& header) {
        if (size < sizeof(CollisionContactsPacket)) {
            std::cerr << "⚠️  Collision contacts packet too small" << std::endl;
            return;
        }
        
        CollisionContactsPacket packet;
        std::memcpy(&packet, data, sizeof(CollisionContactsPacket));
        
        std::cout << "💥 Collision Contacts [Frame " << header.frame_id << "]: ";
        if (packet.has_collision) {
            std::cout << packet.contact_count << " contacts, max depth: " 
                      << std::fixed << std::setprecision(2) << packet.max_penetration_depth << std::endl;
            
            for (uint32_t i = 0; i < std::min(packet.contact_count, 2u); ++i) {
                const auto& contact = packet.contacts[i];
                std::cout << "   Contact " << i << ": pos("
                          << std::fixed << std::setprecision(1)
                          << contact.contact_x << "," << contact.contact_y << "," << contact.contact_z
                          << ") depth=" << std::setprecision(2) << contact.penetration_depth
                          << " capsule=" << contact.robot_capsule_index << std::endl;
            }
        } else {
            std::cout << "No collision" << std::endl;
        }
    }
    
    void process_frame_sync_packet(const uint8_t* data, size_t size, const PacketHeader& /* header */) {
        if (size < sizeof(FrameSyncPacket)) {
            std::cerr << "⚠️  Frame sync packet too small" << std::endl;
            return;
        }
        
        FrameSyncPacket packet;
        std::memcpy(&packet, data, sizeof(FrameSyncPacket));
        
        std::cout << "⏱️  Frame Sync [" << packet.frame_id << "]: " 
                  << packet.total_packets_this_frame << " packets, "
                  << "compute time: " << std::fixed << std::setprecision(2) << packet.computation_time_ms << "ms"
                  << std::endl;
        std::cout << std::endl; // Blank line between frames
    }
    
    void print_statistics() {
        std::cout << std::endl;
        std::cout << "📊 STATISTICS (every 100 packets):" << std::endl;
        std::cout << "   Total received: " << total_packets_received_ << std::endl;
        std::cout << "   Robot packets: " << robot_packets_ << std::endl;
        std::cout << "   Human packets: " << human_packets_ << std::endl;
        std::cout << "   Collision packets: " << collision_packets_ << std::endl;
        std::cout << "   Sync packets: " << sync_packets_ << std::endl;
        std::cout << "   Unknown packets: " << unknown_packets_ << std::endl;
        std::cout << std::endl;
    }
};

int main() {
    std::cout << "🎯 TEST VISUALIZATION RECEIVER" << std::endl;
    std::cout << "==============================" << std::endl;
    
    TestVisualizationReceiver receiver(9999);
    
    if (!receiver.initialize()) {
        std::cerr << "❌ Failed to initialize receiver" << std::endl;
        return 1;
    }
    
    std::cout << "📡 Listening for visualization packets on UDP port 9999..." << std::endl;
    std::cout << "   Start the sender in another terminal with: ./test_visualization_sender" << std::endl;
    std::cout << std::endl;
    
    try {
        receiver.run_receiver_loop();
    } catch (const std::exception& e) {
        std::cerr << "❌ Exception: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "👋 Receiver shutdown" << std::endl;
    return 0;
}