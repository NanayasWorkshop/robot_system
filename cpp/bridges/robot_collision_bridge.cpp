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
 */
class RobotCollisionBridge {
public:
    /**
     * Convert FABRIK refinement result to collision capsules
     * Main interface for robot bridge
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
     * Convert basic FABRIK result to collision capsules (fallback)
     * For compatibility with basic FABRIK solver
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