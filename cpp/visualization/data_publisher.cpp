#include "data_publisher.hpp"
#include <iostream>
#include <iomanip>

namespace delta {

DataPublisher::DataPublisher() 
    : current_frame_id_(0), target_fps_(30.0), is_running_(false),
      total_frames_published_(0), total_publish_time_ms_(0.0) {
    
    network_sender_ = std::make_unique<NetworkSender>();
    start_time_ = std::chrono::steady_clock::now();
}

DataPublisher::~DataPublisher() {
    shutdown();
}

// =============================================================================
// INITIALIZATION AND CONFIGURATION
// =============================================================================

bool DataPublisher::initialize(const std::string& target_ip, 
                               uint16_t target_port,
                               double fps) {
    
    std::cout << "🚀 Initializing DataPublisher..." << std::endl;
    
    if (!network_sender_->initialize(target_ip, target_port)) {
        std::cerr << "❌ Failed to initialize NetworkSender" << std::endl;
        return false;
    }
    
    target_fps_ = fps;
    is_running_ = true;
    
    std::cout << "✅ DataPublisher initialized (target: " << target_ip 
              << ":" << target_port << ", " << fps << " FPS)" << std::endl;
    
    return true;
}

void DataPublisher::shutdown() {
    if (is_running_) {
        is_running_ = false;
        if (network_sender_) {
            network_sender_->close();
        }
        std::cout << "📡 DataPublisher shutdown" << std::endl;
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
    
    double total_time_ms = 0.0;
    ScopedTimer timer(total_time_ms);
    
    uint32_t frame_id = get_next_frame_id();
    uint32_t packet_count = 0;
    
    // Publish robot capsules
    if (publish_robot_capsules_with_frame_id(robot_capsules, frame_id)) {
        packet_count++;
    }
    
    // Publish human pose (joints + vertices)
    if (publish_human_pose_with_frame_id(human_joints, human_vertices, frame_id)) {
        packet_count++;
    }
    
    // Publish collision contacts
    if (publish_collision_contacts_with_frame_id(collision_result, frame_id)) {
        packet_count++;
    }
    
    // Send frame sync
    bool sync_success = send_frame_sync_with_frame_id(packet_count, computation_time_ms, frame_id);
    if (sync_success) {
        packet_count++;
    }
    
    // Update statistics
    update_statistics(total_time_ms);
    
    return packet_count > 0;
}

bool DataPublisher::publish_layer_states(const LayerStates& layer_states) {
    if (!is_ready()) {
        return false;
    }
    
    double publish_time_ms = 0.0;
    ScopedTimer timer(publish_time_ms);
    
    // Convert layer states to visualization format
    auto layer_packet = convert_layer_states(layer_states);
    
    // Create packet header
    uint32_t frame_id = current_frame_id_; // Use current frame ID
    uint32_t timestamp = get_current_timestamp_ms();
    auto header = create_packet_header<CollisionLayersPacket>(
        PacketType::COLLISION_LAYERS, frame_id, timestamp);
    
    // Serialize and send
    uint8_t buffer[get_total_packet_size<CollisionLayersPacket>()];
    size_t packet_size = serialize_packet(buffer, header, layer_packet);
    
    bool success = network_sender_->send_data(buffer, packet_size);
    
    // Update statistics
    update_statistics(publish_time_ms);
    
    return success;
}

// =============================================================================
// INDIVIDUAL PACKET PUBLISHING (Advanced usage)
// =============================================================================

bool DataPublisher::publish_robot_capsules(const std::vector<CapsuleData>& robot_capsules) {
    uint32_t frame_id = get_next_frame_id();
    return publish_robot_capsules_with_frame_id(robot_capsules, frame_id);
}

bool DataPublisher::publish_human_pose(const std::vector<Eigen::Vector3d>& human_joints,
                                       const std::vector<Eigen::Vector3d>& human_vertices) {
    uint32_t frame_id = get_next_frame_id();
    return publish_human_pose_with_frame_id(human_joints, human_vertices, frame_id);
}

bool DataPublisher::publish_collision_contacts(const CollisionResult& collision_result) {
    uint32_t frame_id = get_next_frame_id();
    return publish_collision_contacts_with_frame_id(collision_result, frame_id);
}

bool DataPublisher::send_frame_sync(uint32_t total_packets, double computation_time_ms) {
    uint32_t frame_id = get_next_frame_id();
    return send_frame_sync_with_frame_id(total_packets, computation_time_ms, frame_id);
}

// =============================================================================
// PERFORMANCE AND DIAGNOSTICS
// =============================================================================

DataPublisher::PublisherStats DataPublisher::get_statistics() const {
    PublisherStats stats;
    stats.frames_published = total_frames_published_;
    stats.avg_publish_time_ms = (total_frames_published_ > 0) ? 
        (total_publish_time_ms_ / total_frames_published_) : 0.0;
    
    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    double elapsed_seconds = std::chrono::duration<double>(elapsed).count();
    stats.current_fps = (elapsed_seconds > 0) ? (total_frames_published_ / elapsed_seconds) : 0.0;
    
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
    
    std::ostringstream info;
    info << "DataPublisher Debug Info:" << std::endl;
    info << "  Status: " << (is_running_ ? "Running" : "Stopped") << std::endl;
    info << "  Target FPS: " << target_fps_ << std::endl;
    info << "  Current FPS: " << std::fixed << std::setprecision(1) << stats.current_fps << std::endl;
    info << "  Frames published: " << stats.frames_published << std::endl;
    info << "  Avg publish time: " << std::setprecision(2) << stats.avg_publish_time_ms << " ms" << std::endl;
    info << "  Network: " << stats.network_stats.packets_sent << " packets, " 
         << stats.network_stats.success_rate << "% success" << std::endl;
    
    return info.str();
}

// =============================================================================
// INTERNAL CONVERSION METHODS
// =============================================================================

RobotCapsulesPacket DataPublisher::convert_robot_capsules(const std::vector<CapsuleData>& capsules) {
    RobotCapsulesPacket packet;
    packet.capsule_count = std::min(static_cast<uint32_t>(capsules.size()), 16U);
    
    for (size_t i = 0; i < packet.capsule_count; ++i) {
        const auto& capsule = capsules[i];
        auto& viz_capsule = packet.capsules[i];
        
        viz_capsule.start_x = static_cast<float>(capsule.start_point.x());
        viz_capsule.start_y = static_cast<float>(capsule.start_point.y());
        viz_capsule.start_z = static_cast<float>(capsule.start_point.z());
        viz_capsule.end_x = static_cast<float>(capsule.end_point.x());
        viz_capsule.end_y = static_cast<float>(capsule.end_point.y());
        viz_capsule.end_z = static_cast<float>(capsule.end_point.z());
        viz_capsule.radius = static_cast<float>(capsule.radius);
    }
    
    return packet;
}

HumanPosePacket DataPublisher::convert_human_pose(const std::vector<Eigen::Vector3d>& joints) {
    HumanPosePacket packet;
    packet.joint_count = std::min(static_cast<uint32_t>(joints.size()), 24U);
    packet.vertex_count = 0; // Vertices sent separately
    
    for (size_t i = 0; i < packet.joint_count; ++i) {
        const auto& joint = joints[i];
        auto& viz_joint = packet.joints[i];
        
        viz_joint.x = static_cast<float>(joint.x());
        viz_joint.y = static_cast<float>(joint.y());
        viz_joint.z = static_cast<float>(joint.z());
    }
    
    return packet;
}

CollisionContactsPacket DataPublisher::convert_collision_contacts(const CollisionResult& collision_result) {
    CollisionContactsPacket packet;
    packet.contact_count = std::min(static_cast<uint32_t>(collision_result.contacts.size()), 64U);
    packet.has_collision = collision_result.has_collision ? 1 : 0;
    packet.max_penetration_depth = static_cast<float>(collision_result.max_penetration_depth);
    
    for (size_t i = 0; i < packet.contact_count; ++i) {
        const auto& contact = collision_result.contacts[i];
        auto& viz_contact = packet.contacts[i];
        
        viz_contact.contact_x = static_cast<float>(contact.contact_point.x());
        viz_contact.contact_y = static_cast<float>(contact.contact_point.y());
        viz_contact.contact_z = static_cast<float>(contact.contact_point.z());
        viz_contact.normal_x = static_cast<float>(contact.surface_normal.x());
        viz_contact.normal_y = static_cast<float>(contact.surface_normal.y());
        viz_contact.normal_z = static_cast<float>(contact.surface_normal.z());
        viz_contact.penetration_depth = static_cast<float>(contact.penetration_depth);
        viz_contact.robot_capsule_index = static_cast<uint32_t>(contact.robot_capsule_index);
    }
    
    return packet;
}

CollisionLayersPacket DataPublisher::convert_layer_states(const LayerStates& layer_states) {
    CollisionLayersPacket packet;
    
    // Initialize counts
    packet.layer3_count = 0;
    packet.layer2_count = 0; 
    packet.layer1_count = 0;
    
    // Layer 3 - Always show all 9 primitives (always active)
    const auto& layer3_primitives = layer_states.layer3_primitives;
    packet.layer3_count = std::min(static_cast<uint32_t>(layer3_primitives.size()), 16U);
    
    for (size_t i = 0; i < packet.layer3_count; ++i) {
        const auto& primitive = layer3_primitives[i];
        auto& viz_data = packet.layer3_primitives[i];
        
        viz_data.start_x = static_cast<float>(primitive.start_point.x());
        viz_data.start_y = static_cast<float>(primitive.start_point.y());
        viz_data.start_z = static_cast<float>(primitive.start_point.z());
        viz_data.end_x = static_cast<float>(primitive.end_point.x());
        viz_data.end_y = static_cast<float>(primitive.end_point.y());
        viz_data.end_z = static_cast<float>(primitive.end_point.z());
        viz_data.radius = static_cast<float>(primitive.radius);
        viz_data.is_active = 1; // Layer 3 always active
    }
    
    // Layer 2 - Only show ACTIVE primitives (selective loading)
    auto active_layer2_indices = layer_states.get_all_active_layer2_indices();
    packet.layer2_count = std::min(static_cast<uint32_t>(active_layer2_indices.size()), 32U);
    
    for (size_t i = 0; i < packet.layer2_count; ++i) {
        int layer2_idx = active_layer2_indices[i];
        if (layer2_idx >= 0 && layer2_idx < static_cast<int>(layer_states.layer2_primitives.size())) {
            const auto& primitive = layer_states.layer2_primitives[layer2_idx];
            auto& viz_data = packet.layer2_primitives[i];
            
            viz_data.start_x = static_cast<float>(primitive.start_point.x());
            viz_data.start_y = static_cast<float>(primitive.start_point.y());
            viz_data.start_z = static_cast<float>(primitive.start_point.z());
            viz_data.end_x = static_cast<float>(primitive.end_point.x());
            viz_data.end_y = static_cast<float>(primitive.end_point.y());
            viz_data.end_z = static_cast<float>(primitive.end_point.z());
            viz_data.radius = static_cast<float>(primitive.radius);
            viz_data.is_active = 1; // Only sending active ones
        }
    }
    
    // Layer 1 - Only show ACTIVE primitives (selective loading)
    auto active_layer1_indices = layer_states.get_all_active_layer1_indices();
    packet.layer1_count = std::min(static_cast<uint32_t>(active_layer1_indices.size()), 128U);
    
    for (size_t i = 0; i < packet.layer1_count; ++i) {
        int layer1_idx = active_layer1_indices[i];
        if (layer1_idx >= 0 && layer1_idx < static_cast<int>(layer_states.layer1_primitives.size())) {
            const auto& primitive = layer_states.layer1_primitives[layer1_idx];
            auto& viz_data = packet.layer1_primitives[i];
            
            viz_data.center_x = static_cast<float>(primitive.center.x());
            viz_data.center_y = static_cast<float>(primitive.center.y());
            viz_data.center_z = static_cast<float>(primitive.center.z());
            viz_data.radius = static_cast<float>(primitive.radius);
            viz_data.is_active = 1; // Only sending active ones
        }
    }
    
    return packet;
}

bool DataPublisher::send_human_vertices_batched(const std::vector<Eigen::Vector3d>& vertices) {
    uint32_t frame_id = current_frame_id_;
    return send_human_vertices_batched_with_frame_id(vertices, frame_id);
}

// =============================================================================
// FRAME ID SPECIFIC METHODS
// =============================================================================

bool DataPublisher::publish_robot_capsules_with_frame_id(const std::vector<CapsuleData>& robot_capsules, uint32_t frame_id) {
    if (!is_ready() || robot_capsules.empty()) {
        return false;
    }
    
    auto packet = convert_robot_capsules(robot_capsules);
    uint32_t timestamp = get_current_timestamp_ms();
    auto header = create_packet_header<RobotCapsulesPacket>(PacketType::ROBOT_CAPSULES, frame_id, timestamp);
    
    uint8_t buffer[get_total_packet_size<RobotCapsulesPacket>()];
    size_t packet_size = serialize_packet(buffer, header, packet);
    
    return network_sender_->send_data(buffer, packet_size);
}

bool DataPublisher::publish_human_pose_with_frame_id(const std::vector<Eigen::Vector3d>& human_joints,
                                                    const std::vector<Eigen::Vector3d>& human_vertices,
                                                    uint32_t frame_id) {
    if (!is_ready()) {
        return false;
    }
    
    bool success = true;
    
    // Send joints
    if (!human_joints.empty()) {
        auto pose_packet = convert_human_pose(human_joints);
        uint32_t timestamp = get_current_timestamp_ms();
        auto header = create_packet_header<HumanPosePacket>(PacketType::HUMAN_POSE, frame_id, timestamp);
        
        uint8_t buffer[get_total_packet_size<HumanPosePacket>()];
        size_t packet_size = serialize_packet(buffer, header, pose_packet);
        
        success &= network_sender_->send_data(buffer, packet_size);
    }
    
    // Send vertices in batches
    if (!human_vertices.empty()) {
        success &= send_human_vertices_batched_with_frame_id(human_vertices, frame_id);
    }
    
    return success;
}

bool DataPublisher::publish_collision_contacts_with_frame_id(const CollisionResult& collision_result, uint32_t frame_id) {
    if (!is_ready()) {
        return false;
    }
    
    auto packet = convert_collision_contacts(collision_result);
    uint32_t timestamp = get_current_timestamp_ms();
    auto header = create_packet_header<CollisionContactsPacket>(PacketType::COLLISION_CONTACTS, frame_id, timestamp);
    
    uint8_t buffer[get_total_packet_size<CollisionContactsPacket>()];
    size_t packet_size = serialize_packet(buffer, header, packet);
    
    return network_sender_->send_data(buffer, packet_size);
}

bool DataPublisher::send_frame_sync_with_frame_id(uint32_t total_packets, double computation_time_ms, uint32_t frame_id) {
    if (!is_ready()) {
        return false;
    }
    
    uint32_t timestamp = get_current_timestamp_ms();
    FrameSyncPacket sync_packet(frame_id, timestamp, total_packets, static_cast<float>(computation_time_ms));
    auto header = create_packet_header<FrameSyncPacket>(PacketType::FRAME_SYNC, frame_id, timestamp);
    
    uint8_t buffer[get_total_packet_size<FrameSyncPacket>()];
    size_t packet_size = serialize_packet(buffer, header, sync_packet);
    
    return network_sender_->send_data(buffer, packet_size);
}

bool DataPublisher::send_human_vertices_batched_with_frame_id(const std::vector<Eigen::Vector3d>& vertices, uint32_t frame_id) {
    if (vertices.empty()) {
        return true;
    }
    
    const size_t vertices_per_batch = 500; // Reduced from 1000 to 500
    size_t total_batches = (vertices.size() + vertices_per_batch - 1) / vertices_per_batch;
    
    bool all_success = true;
    
    for (size_t batch_idx = 0; batch_idx < total_batches; ++batch_idx) {
        HumanVerticesPacket vertex_packet;
        vertex_packet.batch_index = static_cast<uint32_t>(batch_idx);
        vertex_packet.total_batches = static_cast<uint32_t>(total_batches);
        
        size_t start_idx = batch_idx * vertices_per_batch;
        size_t end_idx = std::min(start_idx + vertices_per_batch, vertices.size());
        vertex_packet.vertex_count = static_cast<uint32_t>(end_idx - start_idx);
        
        // Fill vertex data
        for (size_t i = 0; i < vertex_packet.vertex_count; ++i) {
            const auto& vertex = vertices[start_idx + i];
            auto& viz_vertex = vertex_packet.vertices[i];
            
            viz_vertex.x = static_cast<float>(vertex.x());
            viz_vertex.y = static_cast<float>(vertex.y());
            viz_vertex.z = static_cast<float>(vertex.z());
        }
        
        // Send batch
        uint32_t timestamp = get_current_timestamp_ms();
        auto header = create_packet_header<HumanVerticesPacket>(PacketType::HUMAN_POSE, frame_id, timestamp);
        
        uint8_t buffer[get_total_packet_size<HumanVerticesPacket>()];
        size_t packet_size = serialize_packet(buffer, header, vertex_packet);
        
        all_success &= network_sender_->send_data(buffer, packet_size);
    }
    
    return all_success;
}

// =============================================================================
// UTILITY METHODS
// =============================================================================

uint32_t DataPublisher::get_current_timestamp_ms() const {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - start_time_;
    return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count());
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
    
    if (robot_capsules.empty()) {
        std::cerr << "⚠️  No robot capsules provided" << std::endl;
        return false;
    }
    
    if (human_joints.size() != 24) {
        std::cerr << "⚠️  Expected 24 human joints, got " << human_joints.size() << std::endl;
        return false;
    }
    
    // Validate finite values
    for (const auto& capsule : robot_capsules) {
        if (!capsule.start_point.allFinite() || !capsule.end_point.allFinite() || 
            !std::isfinite(capsule.radius)) {
            std::cerr << "⚠️  Invalid robot capsule data" << std::endl;
            return false;
        }
    }
    
    for (const auto& joint : human_joints) {
        if (!joint.allFinite()) {
            std::cerr << "⚠️  Invalid human joint data" << std::endl;
            return false;
        }
    }
    
    return true;
}

} // namespace delta