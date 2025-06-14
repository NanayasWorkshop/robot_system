#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "../collision/collision_detection_engine.hpp"
#include "../collision/layer_manager.hpp"
#include "bridge_common.hpp"

namespace delta {

/**
 * STAR Collision Bridge (SIMPLIFIED)
 * Validates and passes STAR human model data directly to collision detection system
 * No coordinate transformation - collision system now operates in STAR coordinate space
 */
class STARCollisionBridge {
public:
    /**
     * Update collision detection engine with new STAR pose (simplified)
     * Main interface for STAR bridge - validates and passes data directly
     * 
     * @param engine Collision detection engine to update
     * @param star_joints STAR joint positions (24 joints, Y-up, meters)
     * @param debug_logging Enable minimal debug output (default: false)
     * @return BridgeResult with success status or error info
     */
    static BridgeResult<bool> update_collision_from_star(
        CollisionDetectionEngine& engine,
        const std::vector<Eigen::Vector3d>& star_joints,
        bool debug_logging = false);
    
    /**
     * Direct LayerManager pose update (simplified)
     * Updates collision layers directly with STAR data (no transformation)
     * 
     * @param layer_manager LayerManager to update
     * @param star_joints STAR joint positions (24 joints, Y-up, meters)
     * @param debug_logging Enable debug output
     * @return BridgeResult with success status or error info
     */
    static BridgeResult<bool> update_layer_manager_from_star(
        LayerManager& layer_manager,
        const std::vector<Eigen::Vector3d>& star_joints,
        bool debug_logging = false);

private:
    /**
     * Validate STAR joint positions
     * Check count, finite values, reasonable ranges
     * 
     * @param star_joints STAR joint positions to validate
     * @return true if valid, false otherwise
     */
    static bool validate_star_joints(const std::vector<Eigen::Vector3d>& star_joints);
    
    /**
     * Print minimal debug information
     * Only prints essential info to avoid console spam
     * 
     * @param message Debug message
     * @param debug_logging Whether debug is enabled
     */
    static void debug_print(const std::string& message, bool debug_logging);
    
    /**
     * Get coordinate system info string for debugging
     * @param joints Joint positions to analyze
     * @return String with coordinate system info
     */
    static std::string get_coordinate_info(const std::vector<Eigen::Vector3d>& joints);

public:
    // Constants for reference
    static constexpr int EXPECTED_JOINT_COUNT = 24;  // STAR joint count
    
    // STAR joint names for reference and debugging
    static const std::vector<std::string> STAR_JOINT_NAMES;
};

} // namespace delta