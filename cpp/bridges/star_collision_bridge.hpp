#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "../collision/collision_detection_engine.hpp"
#include "../collision/layer_manager.hpp"
#include "bridge_common.hpp"

namespace delta {

/**
 * STAR Collision Bridge
 * Converts STAR human model data to update collision detection system
 * Handles coordinate transformation from STAR (Y-up, meters) to collision system (Z-up, millimeters)
 */
class STARCollisionBridge {
public:
    /**
     * Update collision detection engine with new STAR pose
     * Main interface for STAR bridge - handles complete pose update
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
     * Transform STAR joint positions to collision coordinate system
     * Coordinate transformation: STAR (Y-up, meters) → Collision (Z-up, millimeters)
     * 
     * @param star_joints STAR joint positions (24 joints)
     * @param debug_logging Enable debug output
     * @return BridgeResult with transformed joint positions or error info
     */
    static BridgeResult<std::vector<Eigen::Vector3d>> transform_star_to_collision_coords(
        const std::vector<Eigen::Vector3d>& star_joints,
        bool debug_logging = false);
    
    /**
     * Direct LayerManager pose update (for advanced usage)
     * Updates collision layers without going through CollisionDetectionEngine
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
     * Transform single STAR point to collision coordinate system
     * Core coordinate transformation logic
     * 
     * @param star_point Point in STAR coordinates (Y-up, meters)
     * @return Point in collision coordinates (Z-up, millimeters)
     */
    static Eigen::Vector3d transform_point_star_to_collision(const Eigen::Vector3d& star_point);
    
    /**
     * Validate STAR joint positions
     * Check count, finite values, reasonable ranges
     * 
     * @param star_joints STAR joint positions to validate
     * @return true if valid, false otherwise
     */
    static bool validate_star_joints(const std::vector<Eigen::Vector3d>& star_joints);
    
    /**
     * Validate transformed joint positions
     * Check for reasonable values after coordinate transformation
     * 
     * @param transformed_joints Transformed joint positions
     * @return true if valid, false otherwise
     */
    static bool validate_transformed_joints(const std::vector<Eigen::Vector3d>& transformed_joints);
    
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
    // Coordinate system constants for reference
    static constexpr double METERS_TO_MM = 1000.0;  // STAR uses meters, collision uses millimeters
    static constexpr int EXPECTED_JOINT_COUNT = 24;  // STAR joint count
    
    // STAR joint names for reference and debugging
    static const std::vector<std::string> STAR_JOINT_NAMES;
};

} // namespace delta