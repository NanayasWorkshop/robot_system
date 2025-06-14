#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "../collision_blocks/capsule_creation_block.hpp"
#include "../blocks/fabrik_solver_block.hpp"
#include "../blocks/segment_block.hpp"
#include "../core/constants.hpp"
#include "bridge_common.hpp"

namespace delta {

/**
 * Robot Collision Bridge
 * Converts FABRIK solver results to robot capsule representation for collision detection
 * UPDATED: Now supports transformation to STAR coordinate system for direct collision
 */
class RobotCollisionBridge {
public:
    /**
     * Convert FABRIK refinement result to collision capsules (robot coordinates)
     * Original function for robot coordinate system (Z-up, mm)
     * 
     * @param fabrik_result Result from FabrikSolverBlock::solve_with_prismatic_refinement()
     * @param robot_radius Robot radius (default: ROBOT_RADIUS from constants)
     * @param debug_logging Enable minimal debug output (default: false)
     * @return BridgeResult with vector of CapsuleData or error info
     */
    static BridgeResult<std::vector<CapsuleData>> convert_fabrik_to_capsules(
        const FabrikRefinementResult& fabrik_result,
        double robot_radius = ROBOT_RADIUS,
        bool debug_logging = false);
    
    /**
     * Convert basic FABRIK result to collision capsules (robot coordinates)
     * For compatibility with basic FABRIK solver (Z-up, mm)
     * 
     * @param fabrik_result Result from FabrikSolverBlock::solve()
     * @param robot_radius Robot radius (default: ROBOT_RADIUS from constants)
     * @param debug_logging Enable minimal debug output (default: false)
     * @return BridgeResult with vector of CapsuleData or error info
     */
    static BridgeResult<std::vector<CapsuleData>> convert_basic_fabrik_to_capsules(
        const FabrikSolverResult& fabrik_result,
        double robot_radius = ROBOT_RADIUS,
        bool debug_logging = false);

    /**
     * NEW: Convert FABRIK refinement result to STAR coordinate capsules
     * Transforms robot capsules from Z-up mm to Y-up meters for direct STAR collision
     * 
     * @param fabrik_result Result from FabrikSolverBlock::solve_with_prismatic_refinement()
     * @param robot_radius Robot radius (default: ROBOT_RADIUS from constants)
     * @param robot_offset Robot position offset in STAR coordinates (default: zero)
     * @param debug_logging Enable minimal debug output (default: false)
     * @return BridgeResult with vector of CapsuleData in STAR coordinates or error info
     */
    static BridgeResult<std::vector<CapsuleData>> convert_fabrik_to_capsules_star_coords(
        const FabrikRefinementResult& fabrik_result,
        double robot_radius = ROBOT_RADIUS,
        const Eigen::Vector3d& robot_offset = Eigen::Vector3d::Zero(),
        bool debug_logging = false);
    
    /**
     * NEW: Convert basic FABRIK result to STAR coordinate capsules
     * Transforms robot capsules from Z-up mm to Y-up meters for direct STAR collision
     * 
     * @param fabrik_result Result from FabrikSolverBlock::solve()
     * @param robot_radius Robot radius (default: ROBOT_RADIUS from constants)
     * @param robot_offset Robot position offset in STAR coordinates (default: zero)
     * @param debug_logging Enable minimal debug output (default: false)
     * @return BridgeResult with vector of CapsuleData in STAR coordinates or error info
     */
    static BridgeResult<std::vector<CapsuleData>> convert_basic_fabrik_to_capsules_star_coords(
        const FabrikSolverResult& fabrik_result,
        double robot_radius = ROBOT_RADIUS,
        const Eigen::Vector3d& robot_offset = Eigen::Vector3d::Zero(),
        bool debug_logging = false);

private:
    /**
     * Extract S-points from FABRIK joint positions
     * Core conversion logic from joints to segment positions
     * 
     * @param joint_positions Joint positions from FABRIK
     * @param debug_logging Enable debug output
     * @return BridgeResult with S-points or error info
     */
    static BridgeResult<std::vector<Eigen::Vector3d>> extract_s_points_from_joints(
        const std::vector<Eigen::Vector3d>& joint_positions,
        bool debug_logging = false);
    
    /**
     * NEW: Transform robot point from robot coordinates to STAR coordinates with offset
     * Robot (Z-up, mm) → STAR (Y-up, meters) + offset
     * 
     * @param robot_point Point in robot coordinates
     * @param offset Offset to add in STAR coordinates (meters)
     * @return Point in STAR coordinates with offset applied
     */
    static Eigen::Vector3d transform_robot_to_star(const Eigen::Vector3d& robot_point, 
                                                   const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());
    
    /**
     * Transform capsules from robot to STAR coordinates with offset
     * Helper function for coordinate transformation
     * 
     * @param robot_capsules Capsules in robot coordinates
     * @param offset Offset to add in STAR coordinates (meters)
     * @param debug_logging Enable debug output
     * @return BridgeResult with transformed capsules or error info
     */
    static BridgeResult<std::vector<CapsuleData>> transform_capsules_robot_to_star(
        const std::vector<CapsuleData>& robot_capsules,
        const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(),
        bool debug_logging = false);
    
    /**
     * Validate FABRIK result structure
     * Check if FABRIK solver succeeded and has valid data
     * 
     * @param fabrik_result FABRIK result to validate
     * @return true if valid, false otherwise
     */
    static bool validate_fabrik_result(const FabrikRefinementResult& fabrik_result);
    
    /**
     * Validate basic FABRIK result structure
     */
    static bool validate_basic_fabrik_result(const FabrikSolverResult& fabrik_result);
    
    /**
     * Validate joint positions array
     * Check for finite values, reasonable count, etc.
     * 
     * @param joint_positions Joint positions to validate
     * @return true if valid, false otherwise
     */
    static bool validate_joint_positions(const std::vector<Eigen::Vector3d>& joint_positions);
    
    /**
     * Print minimal debug information
     * Only prints essential info to avoid console spam
     * 
     * @param message Debug message
     * @param debug_logging Whether debug is enabled
     */
    static void debug_print(const std::string& message, bool debug_logging);
};

} // namespace delta