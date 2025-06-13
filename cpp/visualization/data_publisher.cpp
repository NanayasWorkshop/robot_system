#include "data_publisher.hpp"
#include "../core/timing.hpp"
#include <iostream>
#include <algorithm>

namespace delta {

DataPublisher::DataPublisher() 
    : network_sender_(std::make_unique<NetworkSender>())
    , current_frame_id_(0), target_fps_(30.0), is_running_(false)
    , total_frames_published_(0), total_publish_time_ms_(0.0) {
    start_time_ = std::chrono::steady_clock::now();
}

DataPublisher::~DataPublisher() {
    shutdown();
}

// =============================================================================
// INITIALIZATION AND CONFIGURATION
// =============================================================================

bool DataPublisher::initialize(const std::string& target_ip, uint16_t target_port, double fps) {
    target_fps_ = fps;
    
    if (!network_sender_->initialize(target_ip, target_port)) {
        std::cerr << "DataPublisher: Failed to initialize network sender" << std::endl;
        return false;
    }
    
    is_running_ = true;
    start_time_ = std::chrono::steady_clock::now();
    current_frame_id_ = 0;
    
    std::cout << "DataPublisher: Initialized at " << fps << " FPS → " 
              << target_ip << ":" << target_port << std::endl;
    
    return true;
}

void DataPublisher::shutdown() {
    is_running_ = false;
    if (network_sender_) {
        network_sender_->close();
    }
}

bool DataPublisher::is_ready() const {
    return is_running_ && network_sender_ && network_sender_->is_ready();
}

// =============================================================================
// DATA PUBLISHING INTERFACE
// =============================================================================

bool DataPublisher::publish_collision_frame(
    const std::vector<CapsuleData>& robot_capsules,
    const std::vector<Eigen::Vector3d>& human_joints,
    const std::vector<Eigen::Vector3d>& human_vertices,
    const CollisionResult& collision_result,
    double computation_time_ms) {
    
    if (!is_ready()) {
        return false;
    }
    
    double publish_time = 0.0;
    {
        ScopedTimer timer(publish_time);
        
        // Validate input data
        if (!validate_input_data(robot_capsules, human_joints)) {
            return false;
        }
        
        // FIXED: Increment frame ID once per complete frame
        uint32_t frame_id = get_next_frame_id();
        
        uint32_t packets_sent = 0;
        bool success = true;
        
        // 1. Publish robot capsules
        if (success && !robot_capsules.empty()) {
            success &= publish_robot_capsules_with_frame_id(robot_capsules, frame_id);
            if (success) packets_sent++;
        }
        
        // 2. Publish human pose
        if (success && !human_joints.empty()) {
            success &= publish_human_pose_with_frame_id(human_joints, human_vertices, frame_id);
            if (success) packets_sent++;
            
            // Send vertices separately if present
            if (success && !human_vertices.empty()) {
                success &= send_human_vertices_batched_with_frame_id(human_vertices, frame_id);
                // Note: batched sends count as multiple packets
            }
        }
        
        // 3. Publish collision contacts
        if (success) {
            success &= publish_collision_contacts_with_frame_id(collision_result, frame_id);
            if (success) packets_sent++;
        }
        
        // 4. Send frame synchronization
        if (success) {
            success &= send_frame_sync_with_frame_id(packets_sent, computation_time_ms, frame_id);
        }
        
        if (!success) {
            std::cerr << "DataPublisher: Failed to publish complete frame " 
                      << frame_id << std::endl;
            return false;
        }
    }
    
    update_statistics(publish_time);
    return true;
}

bool DataPublisher::publish_layer_states(const LayerStates& layer_states) {
    if (!is_ready()) {
        return false;
    }
    
    // Convert layer states to visualization format
    CollisionLayersPacket layers_packet = convert_layer_states(layer_states);
    
    // Create packet header
    PacketHeader header = create_packet_header<CollisionLayersPacket>(
        PacketType::COLLISION_LAYERS, get_next_frame_id(), get_current_timestamp_ms());
    
    // Serialize and send
    uint8_t buffer[get_total_packet_size<CollisionLayersPacket>()];
    size_t packet_size = serialize_packet(buffer, header, layers_packet);
    
    return network_sender_->send_data(buffer, packet_size);
}

// =============================================================================
// INDIVIDUAL PACKET PUBLISHING (Updated with frame ID parameter)
// =============================================================================

bool DataPublisher::publish_robot_capsules(const std::vector<CapsuleData>& robot_capsules) {
    return publish_robot_capsules_with_frame_id(robot_capsules, get_next_frame_id());
}

bool DataPublisher::publish_robot_capsules_with_frame_id(const std::vector<CapsuleData>& robot_capsules, uint32_t frame_id) {
    if (!is_ready()) {
        return false;
    }
    
    // Convert to visualization format
    RobotCapsulesPacket capsules_packet = convert_robot_capsules(robot_capsules);
    
    // Create packet header with provided frame ID
    PacketHeader header = create_packet_header<RobotCapsulesPacket>(
        PacketType::ROBOT_CAPSULES, frame_id, get_current_timestamp_ms());
    
    // Serialize and send
    uint8_t buffer[get_total_packet_size<RobotCapsulesPacket>()];
    size_t packet_size = serialize_packet(buffer, header, capsules_packet);
    
    return network_sender_->send_data(buffer, packet_size);
}

bool DataPublisher::publish_human_pose(const std::vector<Eigen::Vector3d>& human_joints,
                                      const std::vector<Eigen::Vector3d>& human_vertices) {
    return publish_human_pose_with_frame_id(human_joints, human_vertices, get_next_frame_id());
}

bool DataPublisher::publish_human_pose_with_frame_id(const std::vector<Eigen::Vector3d>& human_joints,
                                                    const std::vector<Eigen::Vector3d>& human_vertices,
                                                    uint32_t frame_id) {
    if (!is_ready()) {
        return false;
    }
    
    // Convert to visualization format
    HumanPosePacket pose_packet = convert_human_pose(human_joints);
    pose_packet.vertex_count = static_cast<uint32_t>(human_vertices.size());
    
    // Create packet header with provided frame ID
    PacketHeader header = create_packet_header<HumanPosePacket>(
        PacketType::HUMAN_POSE, frame_id, get_current_timestamp_ms());
    
    // Serialize and send
    uint8_t buffer[get_total_packet_size<HumanPosePacket>()];
    size_t packet_size = serialize_packet(buffer, header, pose_packet);
    
    return network_sender_->send_data(buffer, packet_size);
}

bool DataPublisher::publish_collision_contacts(const CollisionResult& collision_result) {
    return publish_collision_contacts_with_frame_id(collision_result, get_next_frame_id());
}

bool DataPublisher::publish_collision_contacts_with_frame_id(const CollisionResult& collision_result, uint32_t frame_id) {
    if (!is_ready()) {
        return false;
    }
    
    // Convert to visualization format
    CollisionContactsPacket contacts_packet = convert_collision_contacts(collision_result);
    
    // Create packet header with provided frame ID
    PacketHeader header = create_packet_header<CollisionContactsPacket>(
        PacketType::COLLISION_CONTACTS, frame_id, get_current_timestamp_ms());
    
    // Serialize and send
    uint8_t buffer[get_total_packet_size<CollisionContactsPacket>()];
    size_t packet_size = serialize_packet(buffer, header, contacts_packet);
    
    return network_sender_->send_data(buffer, packet_size);
}

bool DataPublisher::send_frame_sync(uint32_t total_packets, double computation_time_ms) {
    return send_frame_sync_with_frame_id(total_packets, computation_time_ms, current_frame_id_);
}

bool DataPublisher::send_frame_sync_with_frame_id(uint32_t total_packets, double computation_time_ms, uint32_t frame_id) {
    if (!is_ready()) {
        return false;
    }
    
    // Create frame sync packet
    FrameSyncPacket sync_packet(frame_id, get_current_timestamp_ms(), 
                               total_packets, static_cast<float>(computation_time_ms));
    
    // Create packet header with provided frame ID
    PacketHeader header = create_packet_header<FrameSyncPacket>(
        PacketType::FRAME_SYNC, frame_id, get_current_timestamp_ms());
    
    // Serialize and send
    uint8_t buffer[get_total_packet_size<FrameSyncPacket>()];
    size_t packet_size = serialize_packet(buffer, header, sync_packet);
    
    return network_sender_->send_data(buffer, packet_size);
}

// =============================================================================
// PERFORMANCE AND DIAGNOSTICS
// =============================================================================

DataPublisher::PublisherStats DataPublisher::get_statistics() const {
    PublisherStats stats;
    stats.frames_published = total_frames_published_;
    
    if (total_frames_published_ > 0) {
        stats.avg_publish_time_ms = total_publish_time_ms_ / total_frames_published_;
    }
    
    // Calculate current FPS
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_seconds = std::chrono::duration<double>(current_time - start_time_).count();
    if (elapsed_seconds > 0.0) {
        stats.current_fps = total_frames_published_ / elapsed_seconds;
    }
    
    stats.network_stats = network_sender_->get_statistics();
    
    return stats;
}

void DataPublisher::reset_statistics() {
    total_frames_published_ = 0;
    total_publish_time_ms_ = 0.0;
    start_time_ = std::chrono::steady_clock::now();
    network_sender_->reset_statistics();
}

std::string DataPublisher::get_debug_info() const {
    auto stats = get_statistics();
    
    std::string result = "DataPublisher Debug Info:\n";
    result += "  Status: " + std::string(is_ready() ? "Ready" : "Not Ready") + "\n";
    result += "  Target FPS: " + std::to_string(target_fps_) + "\n";
    result += "  Current FPS: " + std::to_string(stats.current_fps) + "\n";
    result += "  Frames Published: " + std::to_string(stats.frames_published) + "\n";
    result += "  Avg Publish Time: " + std::to_string(stats.avg_publish_time_ms) + "ms\n";
    result += "  Network: " + network_sender_->get_connection_info() + "\n";
    result += "  Packets Sent: " + std::to_string(stats.network_stats.packets_sent) + "\n";
    result += "  Success Rate: " + std::to_string(stats.network_stats.success_rate) + "%\n";
    
    return result;
}

// =============================================================================
// INTERNAL CONVERSION METHODS
// =============================================================================

RobotCapsulesPacket DataPublisher::convert_robot_capsules(const std::vector<CapsuleData>& capsules) {
    RobotCapsulesPacket packet;
    packet.capsule_count = std::min(static_cast<uint32_t>(capsules.size()), 
                                   static_cast<uint32_t>(16)); // Max array size
    
    for (uint32_t i = 0; i < packet.capsule_count; ++i) {
        const auto& cap = capsules[i];
        packet.capsules[i] = CapsuleData_Viz(
            static_cast<float>(cap.start_point.x()), static_cast<float>(cap.start_point.y()), static_cast<float>(cap.start_point.z()),
            static_cast<float>(cap.end_point.x()), static_cast<float>(cap.end_point.y()), static_cast<float>(cap.end_point.z()),
            static_cast<float>(cap.radius)
        );
    }
    
    return packet;
}

HumanPosePacket DataPublisher::convert_human_pose(const std::vector<Eigen::Vector3d>& joints) {
    HumanPosePacket packet;
    packet.joint_count = std::min(static_cast<uint32_t>(joints.size()), 
                                 static_cast<uint32_t>(24)); // STAR joint count
    
    for (uint32_t i = 0; i < packet.joint_count; ++i) {
        const auto& joint = joints[i];
        packet.joints[i] = JointData_Viz(
            static_cast<float>(joint.x()), static_cast<float>(joint.y()), static_cast<float>(joint.z())
        );
    }
    
    return packet;
}

CollisionContactsPacket DataPublisher::convert_collision_contacts(const CollisionResult& collision_result) {
    CollisionContactsPacket packet;
    packet.has_collision = collision_result.has_collision ? 1 : 0;
    packet.max_penetration_depth = static_cast<float>(collision_result.max_penetration_depth);
    packet.contact_count = std::min(static_cast<uint32_t>(collision_result.contacts.size()), 
                                   static_cast<uint32_t>(64)); // Max array size
    
    for (uint32_t i = 0; i < packet.contact_count; ++i) {
        const auto& contact = collision_result.contacts[i];
        packet.contacts[i] = ContactData_Viz(
            static_cast<float>(contact.contact_point.x()), static_cast<float>(contact.contact_point.y()), static_cast<float>(contact.contact_point.z()),
            static_cast<float>(contact.surface_normal.x()), static_cast<float>(contact.surface_normal.y()), static_cast<float>(contact.surface_normal.z()),
            static_cast<float>(contact.penetration_depth),
            contact.robot_capsule_index
        );
    }
    
    return packet;
}

CollisionLayersPacket DataPublisher::convert_layer_states(const LayerStates& layer_states) {
    CollisionLayersPacket packet;
    
    // Convert Layer 3 primitives
    packet.layer3_count = std::min(static_cast<uint32_t>(layer_states.layer3_primitives.size()), 
                                  static_cast<uint32_t>(16));
    for (uint32_t i = 0; i < packet.layer3_count; ++i) {
        const auto& prim = layer_states.layer3_primitives[i];
        packet.layer3_primitives[i].start_x = static_cast<float>(prim.start_point.x());
        packet.layer3_primitives[i].start_y = static_cast<float>(prim.start_point.y());
        packet.layer3_primitives[i].start_z = static_cast<float>(prim.start_point.z());
        packet.layer3_primitives[i].end_x = static_cast<float>(prim.end_point.x());
        packet.layer3_primitives[i].end_y = static_cast<float>(prim.end_point.y());
        packet.layer3_primitives[i].end_z = static_cast<float>(prim.end_point.z());
        packet.layer3_primitives[i].radius = static_cast<float>(prim.radius);
        packet.layer3_primitives[i].is_active = prim.is_active ? 1 : 0;
    }
    
    // Convert Layer 2 primitives
    packet.layer2_count = std::min(static_cast<uint32_t>(layer_states.layer2_primitives.size()), 
                                  static_cast<uint32_t>(32));
    for (uint32_t i = 0; i < packet.layer2_count; ++i) {
        const auto& prim = layer_states.layer2_primitives[i];
        packet.layer2_primitives[i].start_x = static_cast<float>(prim.start_point.x());
        packet.layer2_primitives[i].start_y = static_cast<float>(prim.start_point.y());
        packet.layer2_primitives[i].start_z = static_cast<float>(prim.start_point.z());
        packet.layer2_primitives[i].end_x = static_cast<float>(prim.end_point.x());
        packet.layer2_primitives[i].end_y = static_cast<float>(prim.end_point.y());
        packet.layer2_primitives[i].end_z = static_cast<float>(prim.end_point.z());
        packet.layer2_primitives[i].radius = static_cast<float>(prim.radius);
        packet.layer2_primitives[i].is_active = prim.is_active ? 1 : 0;
    }
    
    // Convert Layer 1 primitives
    packet.layer1_count = std::min(static_cast<uint32_t>(layer_states.layer1_primitives.size()), 
                                  static_cast<uint32_t>(128));
    for (uint32_t i = 0; i < packet.layer1_count; ++i) {
        const auto& prim = layer_states.layer1_primitives[i];
        packet.layer1_primitives[i].center_x = static_cast<float>(prim.center.x());
        packet.layer1_primitives[i].center_y = static_cast<float>(prim.center.y());
        packet.layer1_primitives[i].center_z = static_cast<float>(prim.center.z());
        packet.layer1_primitives[i].radius = static_cast<float>(prim.radius);
        packet.layer1_primitives[i].is_active = prim.is_active ? 1 : 0;
    }
    
    return packet;
}

bool DataPublisher::send_human_vertices_batched(const std::vector<Eigen::Vector3d>& vertices) {
    return send_human_vertices_batched_with_frame_id(vertices, current_frame_id_);
}

bool DataPublisher::send_human_vertices_batched_with_frame_id(const std::vector<Eigen::Vector3d>& vertices, uint32_t frame_id) {
    if (vertices.empty()) {
        return true;
    }
    
    const size_t batch_size = 1000; // From HumanVerticesPacket definition
    const size_t total_vertices = vertices.size();
    const uint32_t total_batches = static_cast<uint32_t>((total_vertices + batch_size - 1) / batch_size);
    
    for (uint32_t batch_idx = 0; batch_idx < total_batches; ++batch_idx) {
        HumanVerticesPacket vertices_packet;
        vertices_packet.batch_index = batch_idx;
        vertices_packet.total_batches = total_batches;
        
        // Calculate vertices for this batch
        size_t start_idx = batch_idx * batch_size;
        size_t end_idx = std::min(start_idx + batch_size, total_vertices);
        vertices_packet.vertex_count = static_cast<uint32_t>(end_idx - start_idx);
        
        // Copy vertices to packet
        for (size_t i = 0; i < vertices_packet.vertex_count; ++i) {
            const auto& vertex = vertices[start_idx + i];
            vertices_packet.vertices[i] = VertexData_Viz(
                static_cast<float>(vertex.x()), static_cast<float>(vertex.y()), static_cast<float>(vertex.z())
            );
        }
        
        // Create packet header with provided frame ID (using HUMAN_POSE type with batch info)
        PacketHeader header = create_packet_header<HumanVerticesPacket>(
            PacketType::HUMAN_POSE, frame_id, get_current_timestamp_ms());
        
        // Serialize and send
        uint8_t buffer[get_total_packet_size<HumanVerticesPacket>()];
        size_t packet_size = serialize_packet(buffer, header, vertices_packet);
        
        if (!network_sender_->send_data(buffer, packet_size)) {
            std::cerr << "DataPublisher: Failed to send vertex batch " << batch_idx << "/" << total_batches << std::endl;
            return false;
        }
    }
    
    return true;
}

// =============================================================================
// UTILITY METHODS
// =============================================================================

uint32_t DataPublisher::get_current_timestamp_ms() const {
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time_);
    return static_cast<uint32_t>(elapsed.count());
}

uint32_t DataPublisher::get_next_frame_id() {
    return ++current_frame_id_;
}

void DataPublisher::update_statistics(double publish_time_ms) const {
    total_frames_published_++;
    total_publish_time_ms_ += publish_time_ms;
}

bool DataPublisher::validate_input_data(const std::vector<CapsuleData>& robot_capsules,
                                       const std::vector<Eigen::Vector3d>& human_joints) const {
    // Check robot capsules
    if (robot_capsules.size() > 16) {
        std::cerr << "DataPublisher: Too many robot capsules (" << robot_capsules.size() 
                  << "), max 16 supported" << std::endl;
        return false;
    }
    
    // Check human joints
    if (human_joints.size() > 24) {
        std::cerr << "DataPublisher: Too many human joints (" << human_joints.size() 
                  << "), max 24 supported" << std::endl;
        return false;
    }
    
    // Validate capsule data
    for (const auto& capsule : robot_capsules) {
        if (capsule.radius <= 0.0) {
            std::cerr << "DataPublisher: Invalid capsule radius: " << capsule.radius << std::endl;
            return false;
        }
    }
    
    // Validate joint positions (check for NaN/Inf)
    for (const auto& joint : human_joints) {
        if (!std::isfinite(joint.x()) || !std::isfinite(joint.y()) || !std::isfinite(joint.z())) {
            std::cerr << "DataPublisher: Invalid joint position: " 
                      << joint.x() << ", " << joint.y() << ", " << joint.z() << std::endl;
            return false;
        }
    }
    
    return true;
}

} // namespace delta