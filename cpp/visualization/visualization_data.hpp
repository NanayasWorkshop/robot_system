#pragma once

#include <cstdint>
#include <cstring>

namespace delta {

// =============================================================================
// PACKET HEADERS AND TYPES
// =============================================================================

enum class PacketType : uint32_t {
    ROBOT_CAPSULES = 1,
    HUMAN_POSE = 2,
    COLLISION_LAYERS = 3,
    COLLISION_CONTACTS = 4,
    FRAME_SYNC = 5
};

struct PacketHeader {
    uint32_t packet_type;    // PacketType enum value
    uint32_t data_size;      // Size of following data in bytes
    uint32_t frame_id;       // Frame number for sync
    uint32_t timestamp_ms;   // Milliseconds since start
    
    PacketHeader() : packet_type(0), data_size(0), frame_id(0), timestamp_ms(0) {}
    
    PacketHeader(PacketType type, uint32_t size, uint32_t frame, uint32_t time)
        : packet_type(static_cast<uint32_t>(type)), data_size(size), 
          frame_id(frame), timestamp_ms(time) {}
};

// =============================================================================
// ROBOT VISUALIZATION DATA
// =============================================================================

struct CapsuleData_Viz {
    float start_x, start_y, start_z;
    float end_x, end_y, end_z;
    float radius;
    
    CapsuleData_Viz() = default;
    CapsuleData_Viz(float sx, float sy, float sz, float ex, float ey, float ez, float r)
        : start_x(sx), start_y(sy), start_z(sz), end_x(ex), end_y(ey), end_z(ez), radius(r) {}
};

struct RobotCapsulesPacket {
    uint32_t capsule_count;
    CapsuleData_Viz capsules[16];  // Max reasonable robot segments
    
    RobotCapsulesPacket() : capsule_count(0) {}
};

// =============================================================================
// HUMAN POSE VISUALIZATION DATA
// =============================================================================

struct JointData_Viz {
    float x, y, z;
    
    JointData_Viz() = default;
    JointData_Viz(float px, float py, float pz) : x(px), y(py), z(pz) {}
};

struct VertexData_Viz {
    float x, y, z;
    
    VertexData_Viz() = default;
    VertexData_Viz(float px, float py, float pz) : x(px), y(py), z(pz) {}
};

struct HumanPosePacket {
    uint32_t joint_count;
    JointData_Viz joints[24];      // STAR joint count
    uint32_t vertex_count;
    // Note: vertices sent separately due to size
    
    HumanPosePacket() : joint_count(0), vertex_count(0) {}
};

struct HumanVerticesPacket {
    uint32_t vertex_count;
    uint32_t batch_index;          // For splitting large vertex arrays
    uint32_t total_batches;
    VertexData_Viz vertices[1000]; // Batch size
    
    HumanVerticesPacket() : vertex_count(0), batch_index(0), total_batches(0) {}
};

// =============================================================================
// COLLISION LAYER VISUALIZATION DATA
// =============================================================================

struct Layer3Data_Viz {
    float start_x, start_y, start_z;
    float end_x, end_y, end_z;
    float radius;
    uint8_t is_active;             // 0 = inactive, 1 = active
    
    Layer3Data_Viz() : radius(0.0f), is_active(0) {}
};

struct Layer2Data_Viz {
    float start_x, start_y, start_z;
    float end_x, end_y, end_z;
    float radius;
    uint8_t is_active;
    
    Layer2Data_Viz() : radius(0.0f), is_active(0) {}
};

struct Layer1Data_Viz {
    float center_x, center_y, center_z;
    float radius;
    uint8_t is_active;
    
    Layer1Data_Viz() : radius(0.0f), is_active(0) {}
};

struct CollisionLayersPacket {
    uint32_t layer3_count;
    Layer3Data_Viz layer3_primitives[16];   // Max layer 3 count
    
    uint32_t layer2_count;
    Layer2Data_Viz layer2_primitives[32];   // Max layer 2 count
    
    uint32_t layer1_count;
    Layer1Data_Viz layer1_primitives[128];  // Max layer 1 count
    
    CollisionLayersPacket() : layer3_count(0), layer2_count(0), layer1_count(0) {}
};

// =============================================================================
// COLLISION CONTACT VISUALIZATION DATA
// =============================================================================

struct ContactData_Viz {
    float contact_x, contact_y, contact_z;
    float normal_x, normal_y, normal_z;
    float penetration_depth;
    uint32_t robot_capsule_index;
    
    ContactData_Viz() : penetration_depth(0.0f), robot_capsule_index(0) {}
    ContactData_Viz(float cx, float cy, float cz, float nx, float ny, float nz, 
                   float depth, uint32_t capsule_idx)
        : contact_x(cx), contact_y(cy), contact_z(cz)
        , normal_x(nx), normal_y(ny), normal_z(nz)
        , penetration_depth(depth), robot_capsule_index(capsule_idx) {}
};

struct CollisionContactsPacket {
    uint32_t contact_count;
    uint8_t has_collision;         // 0 = no collision, 1 = collision detected
    float max_penetration_depth;
    ContactData_Viz contacts[64];  // Max contacts per frame
    
    CollisionContactsPacket() : contact_count(0), has_collision(0), max_penetration_depth(0.0f) {}
};

// =============================================================================
// FRAME SYNCHRONIZATION
// =============================================================================

struct FrameSyncPacket {
    uint32_t frame_id;
    uint32_t timestamp_ms;
    uint32_t total_packets_this_frame;
    float computation_time_ms;
    
    FrameSyncPacket() : frame_id(0), timestamp_ms(0), total_packets_this_frame(0), 
                       computation_time_ms(0.0f) {}
    FrameSyncPacket(uint32_t frame, uint32_t time, uint32_t packet_count, float comp_time)
        : frame_id(frame), timestamp_ms(time), total_packets_this_frame(packet_count),
          computation_time_ms(comp_time) {}
};

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

// Calculate packet size including header
template<typename PacketData>
constexpr size_t get_total_packet_size() {
    return sizeof(PacketHeader) + sizeof(PacketData);
}

// Helper to create packet header
template<typename PacketData>
PacketHeader create_packet_header(PacketType type, uint32_t frame_id, uint32_t timestamp_ms) {
    return PacketHeader(type, sizeof(PacketData), frame_id, timestamp_ms);
}

// Serialize packet with header into buffer
template<typename PacketData>
size_t serialize_packet(void* buffer, const PacketHeader& header, const PacketData& data) {
    uint8_t* buf = static_cast<uint8_t*>(buffer);
    std::memcpy(buf, &header, sizeof(PacketHeader));
    std::memcpy(buf + sizeof(PacketHeader), &data, sizeof(PacketData));
    return sizeof(PacketHeader) + sizeof(PacketData);
}

} // namespace delta