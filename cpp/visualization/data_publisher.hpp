#pragma once

#include "visualization_data.hpp"
#include "network_sender.hpp"
#include "../collision/collision_detection_engine.hpp"
#include "../collision_blocks/capsule_creation_block.hpp"
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <chrono>

namespace delta {

/**
 * Data Publisher - streams collision system results to visualizer
 * Receives data from test programs, packages and streams via UDP
 */
class DataPublisher {
private:
    std::unique_ptr<NetworkSender> network_sender_;
    
    // Frame tracking
    uint32_t current_frame_id_;
    std::chrono::steady_clock::time_point start_time_;
    
    // Configuration
    double target_fps_;
    bool is_running_;
    
    // Statistics
    mutable uint64_t total_frames_published_;
    mutable double total_publish_time_ms_;
    
public:
    DataPublisher();
    ~DataPublisher();
    
    // =============================================================================
    // INITIALIZATION AND CONFIGURATION
    // =============================================================================
    
    /**
     * Initialize publisher
     * @param target_ip Target IP for visualization (default: "127.0.0.1")
     * @param target_port Target port (default: 9999)
     * @param fps Target frames per second (default: 30.0)
     * @return true if successful
     */
    bool initialize(const std::string& target_ip = "127.0.0.1", 
                   uint16_t target_port = 9999,
                   double fps = 30.0);
    
    /**
     * Shutdown publisher
     */
    void shutdown();
    
    /**
     * Check if publisher is ready
     * @return true if ready to publish
     */
    bool is_ready() const;
    
    // =============================================================================
    // DATA PUBLISHING INTERFACE
    // =============================================================================
    
    /**
     * Publish complete collision frame
     * Main interface - publishes all visualization data for one frame
     * 
     * @param robot_capsules Robot represented as capsules
     * @param human_joints Human joint positions (24 joints)
     * @param human_vertices Human mesh vertices (optional, can be empty)
     * @param collision_result Complete collision detection results
     * @param computation_time_ms Time taken for collision computation
     * @return true if published successfully
     */
    bool publish_collision_frame(
        const std::vector<CapsuleData>& robot_capsules,
        const std::vector<Eigen::Vector3d>& human_joints,
        const std::vector<Eigen::Vector3d>& human_vertices,
        const CollisionResult& collision_result,
        double computation_time_ms = 0.0
    );
    
    /**
     * Publish layer states
     * Publishes collision layer activation states
     * 
     * @param layer_states Current layer states from LayerManager
     * @return true if published successfully
     */
    bool publish_layer_states(const LayerStates& layer_states);
    
    // =============================================================================
    // INDIVIDUAL PACKET PUBLISHING (Advanced usage)
    // =============================================================================
    
    /**
     * Publish robot capsules only
     * @param robot_capsules Robot capsule data
     * @return true if published successfully
     */
    bool publish_robot_capsules(const std::vector<CapsuleData>& robot_capsules);
    
    /**
     * Publish human pose only
     * @param human_joints Joint positions
     * @param human_vertices Mesh vertices (optional)
     * @return true if published successfully
     */
    bool publish_human_pose(const std::vector<Eigen::Vector3d>& human_joints,
                           const std::vector<Eigen::Vector3d>& human_vertices = {});
    
    /**
     * Publish collision contacts only
     * @param collision_result Collision detection results
     * @return true if published successfully
     */
    bool publish_collision_contacts(const CollisionResult& collision_result);
    
    /**
     * Send frame synchronization packet
     * @param total_packets Number of packets sent this frame
     * @param computation_time_ms Computation time for this frame
     * @return true if published successfully
     */
    bool send_frame_sync(uint32_t total_packets, double computation_time_ms = 0.0);
    
    // =============================================================================
    // PERFORMANCE AND DIAGNOSTICS
    // =============================================================================
    
    /**
     * Get publisher performance statistics
     */
    struct PublisherStats {
        uint64_t frames_published;
        double avg_publish_time_ms;
        double current_fps;
        NetworkSender::NetworkStats network_stats;
        
        PublisherStats() : frames_published(0), avg_publish_time_ms(0.0), current_fps(0.0) {}
    };
    
    /**
     * Get current statistics
     * @return Publisher performance stats
     */
    PublisherStats get_statistics() const;
    
    /**
     * Reset statistics
     */
    void reset_statistics();
    
    /**
     * Get debug information
     * @return Debug info string
     */
    std::string get_debug_info() const;

private:
    // =============================================================================
    // INTERNAL CONVERSION METHODS
    // =============================================================================
    
    /**
     * Convert collision system capsules to visualization format
     * @param capsules Input capsules from collision system
     * @return Visualization capsule packet
     */
    RobotCapsulesPacket convert_robot_capsules(const std::vector<CapsuleData>& capsules);
    
    /**
     * Convert human pose to visualization format
     * @param joints Joint positions
     * @return Visualization pose packet
     */
    HumanPosePacket convert_human_pose(const std::vector<Eigen::Vector3d>& joints);
    
    /**
     * Convert collision results to visualization format
     * @param collision_result Collision detection results
     * @return Visualization contacts packet
     */
    CollisionContactsPacket convert_collision_contacts(const CollisionResult& collision_result);
    
    /**
     * Convert layer states to visualization format
     * @param layer_states Layer states from collision system
     * @return Visualization layers packet
     */
    CollisionLayersPacket convert_layer_states(const LayerStates& layer_states);
    
    /**
     * Send human vertices in batches (for large meshes)
     * @param vertices Vertex array
     * @return true if all batches sent successfully
     */
    bool send_human_vertices_batched(const std::vector<Eigen::Vector3d>& vertices);
    
    // =============================================================================
    // FRAME ID SPECIFIC METHODS (NEW)
    // =============================================================================
    
    /**
     * Publish robot capsules with specific frame ID
     * @param robot_capsules Robot capsule data
     * @param frame_id Frame ID to use
     * @return true if published successfully
     */
    bool publish_robot_capsules_with_frame_id(const std::vector<CapsuleData>& robot_capsules, uint32_t frame_id);
    
    /**
     * Publish human pose with specific frame ID
     * @param human_joints Joint positions
     * @param human_vertices Mesh vertices (optional)
     * @param frame_id Frame ID to use
     * @return true if published successfully
     */
    bool publish_human_pose_with_frame_id(const std::vector<Eigen::Vector3d>& human_joints,
                                         const std::vector<Eigen::Vector3d>& human_vertices,
                                         uint32_t frame_id);
    
    /**
     * Publish collision contacts with specific frame ID
     * @param collision_result Collision detection results
     * @param frame_id Frame ID to use
     * @return true if published successfully
     */
    bool publish_collision_contacts_with_frame_id(const CollisionResult& collision_result, uint32_t frame_id);
    
    /**
     * Send frame synchronization packet with specific frame ID
     * @param total_packets Number of packets sent this frame
     * @param computation_time_ms Computation time for this frame
     * @param frame_id Frame ID to use
     * @return true if published successfully
     */
    bool send_frame_sync_with_frame_id(uint32_t total_packets, double computation_time_ms, uint32_t frame_id);
    
    /**
     * Send human vertices in batches with specific frame ID
     * @param vertices Vertex array
     * @param frame_id Frame ID to use
     * @return true if all batches sent successfully
     */
    bool send_human_vertices_batched_with_frame_id(const std::vector<Eigen::Vector3d>& vertices, uint32_t frame_id);
    
    // =============================================================================
    // UTILITY METHODS
    // =============================================================================
    
    /**
     * Get current timestamp in milliseconds since start
     * @return Timestamp in ms
     */
    uint32_t get_current_timestamp_ms() const;
    
    /**
     * Get next frame ID and increment counter
     * @return Current frame ID
     */
    uint32_t get_next_frame_id();
    
    /**
     * Update statistics after publishing
     * @param publish_time_ms Time taken for this publish
     */
    void update_statistics(double publish_time_ms) const;
    
    /**
     * Validate input data
     * @param robot_capsules Robot capsules to validate
     * @param human_joints Human joints to validate
     * @return true if data is valid
     */
    bool validate_input_data(const std::vector<CapsuleData>& robot_capsules,
                            const std::vector<Eigen::Vector3d>& human_joints) const;
};

} // namespace delta