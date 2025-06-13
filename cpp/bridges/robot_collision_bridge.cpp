#include "robot_collision_bridge.hpp"
#include <iostream>
#include <iomanip>

namespace delta {

// =============================================================================
// MAIN BRIDGE INTERFACE IMPLEMENTATIONS
// =============================================================================

BridgeResult<std::vector<CapsuleData>> RobotCollisionBridge::convert_fabrik_to_capsules(
    const FabrikRefinementResult& fabrik_result,
    double robot_radius,
    bool debug_logging) {
    
    double total_time_ms = 0.0;
    ScopedTimer timer(total_time_ms);
    
    // Validate FABRIK refinement result
    if (!validate_fabrik_result(fabrik_result)) {
        return BridgeResult<std::vector<CapsuleData>>("Invalid FABRIK refinement result", total_time_ms);
    }
    
    debug_print("Converting FABRIK refinement result to capsules", debug_logging);
    
    // Extract joint positions from refinement result
    const auto& joint_positions = fabrik_result.final_joints;
    
    // Convert joints to S-points
    auto s_points_result = extract_s_points_from_joints(joint_positions, debug_logging);
    if (!s_points_result.success) {
        return BridgeResult<std::vector<CapsuleData>>("S-point extraction failed: " + s_points_result.error_message, total_time_ms);
    }
    
    // Create capsule chain from S-points
    auto capsule_result = CapsuleCreationBlock::create_capsule_chain(s_points_result.data, robot_radius);
    if (!capsule_result.creation_successful) {
        return BridgeResult<std::vector<CapsuleData>>("Capsule creation failed: " + capsule_result.error_message, total_time_ms);
    }
    
    debug_print("Created " + std::to_string(capsule_result.capsules.size()) + " robot capsules", debug_logging);
    
    return BridgeResult<std::vector<CapsuleData>>(capsule_result.capsules, total_time_ms);
}

BridgeResult<std::vector<CapsuleData>> RobotCollisionBridge::convert_basic_fabrik_to_capsules(
    const FabrikSolverResult& fabrik_result,
    double robot_radius,
    bool debug_logging) {
    
    double total_time_ms = 0.0;
    ScopedTimer timer(total_time_ms);
    
    // Validate basic FABRIK result
    if (!validate_basic_fabrik_result(fabrik_result)) {
        return BridgeResult<std::vector<CapsuleData>>("Invalid FABRIK solver result", total_time_ms);
    }
    
    debug_print("Converting basic FABRIK result to capsules", debug_logging);
    
    // Extract joint positions from basic result
    const auto& joint_positions = fabrik_result.final_joints;
    
    // Convert joints to S-points
    auto s_points_result = extract_s_points_from_joints(joint_positions, debug_logging);
    if (!s_points_result.success) {
        return BridgeResult<std::vector<CapsuleData>>("S-point extraction failed: " + s_points_result.error_message, total_time_ms);
    }
    
    // Create capsule chain from S-points
    auto capsule_result = CapsuleCreationBlock::create_capsule_chain(s_points_result.data, robot_radius);
    if (!capsule_result.creation_successful) {
        return BridgeResult<std::vector<CapsuleData>>("Capsule creation failed: " + capsule_result.error_message, total_time_ms);
    }
    
    debug_print("Created " + std::to_string(capsule_result.capsules.size()) + " robot capsules", debug_logging);
    
    return BridgeResult<std::vector<CapsuleData>>(capsule_result.capsules, total_time_ms);
}

// =============================================================================
// PRIVATE HELPER IMPLEMENTATIONS
// =============================================================================

BridgeResult<std::vector<Eigen::Vector3d>> RobotCollisionBridge::extract_s_points_from_joints(
    const std::vector<Eigen::Vector3d>& joint_positions,
    bool debug_logging) {
    
    double extraction_time_ms = 0.0;
    ScopedTimer timer(extraction_time_ms);
    
    // Validate joint positions
    if (!validate_joint_positions(joint_positions)) {
        return BridgeResult<std::vector<Eigen::Vector3d>>("Invalid joint positions", extraction_time_ms);
    }
    
    debug_print("Extracting S-points from " + std::to_string(joint_positions.size()) + " joints", debug_logging);
    
    // For robot arm, S-points are typically the joint positions themselves
    // or calculated as midpoints between consecutive joints
    std::vector<Eigen::Vector3d> s_points;
    
    if (joint_positions.size() < 2) {
        return BridgeResult<std::vector<Eigen::Vector3d>>("Need at least 2 joint positions", extraction_time_ms);
    }
    
    // Method 1: Use joint positions directly as S-points (segment centers)
    // This assumes FABRIK already provides good segment positioning
    for (size_t i = 0; i < joint_positions.size() - 1; ++i) {
        // Calculate midpoint between consecutive joints as S-point
        Eigen::Vector3d s_point = (joint_positions[i] + joint_positions[i + 1]) * 0.5;
        s_points.push_back(s_point);
        
        if (debug_logging && i < 3) {
            debug_print("S-point " + std::to_string(i) + ": (" + 
                       std::to_string(s_point.x()) + ", " + 
                       std::to_string(s_point.y()) + ", " + 
                       std::to_string(s_point.z()) + ")", debug_logging);
        }
    }
    
    debug_print("Extracted " + std::to_string(s_points.size()) + " S-points", debug_logging);
    
    return BridgeResult<std::vector<Eigen::Vector3d>>(s_points, extraction_time_ms);
}

bool RobotCollisionBridge::validate_fabrik_result(const FabrikRefinementResult& fabrik_result) {
    // Check if FABRIK solving was successful
    if (!fabrik_result.solving_successful) {
        std::cerr << "❌ Robot bridge: FABRIK refinement failed: " << fabrik_result.error_message << std::endl;
        return false;
    }
    
    // Check convergence
    if (!fabrik_result.converged) {
        std::cerr << "⚠️  Robot bridge: FABRIK did not converge (max iterations reached)" << std::endl;
        // Continue anyway - partial solution might be usable
    }
    
    // Validate joint positions
    return validate_joint_positions(fabrik_result.final_joints);
}

bool RobotCollisionBridge::validate_basic_fabrik_result(const FabrikSolverResult& fabrik_result) {
    // Check if FABRIK solving was successful
    if (!fabrik_result.solving_successful) {
        std::cerr << "❌ Robot bridge: FABRIK solving failed: " << fabrik_result.error_message << std::endl;
        return false;
    }
    
    // Check convergence
    if (!fabrik_result.converged) {
        std::cerr << "⚠️  Robot bridge: FABRIK did not converge (max iterations reached)" << std::endl;
        // Continue anyway - partial solution might be usable
    }
    
    // Validate joint positions
    return validate_joint_positions(fabrik_result.final_joints);
}

bool RobotCollisionBridge::validate_joint_positions(const std::vector<Eigen::Vector3d>& joint_positions) {
    // Check joint count
    if (joint_positions.empty()) {
        std::cerr << "❌ Robot bridge: No joint positions provided" << std::endl;
        return false;
    }
    
    if (joint_positions.size() > 16) {  // Reasonable upper limit
        std::cerr << "❌ Robot bridge: Too many joint positions (" << joint_positions.size() << ")" << std::endl;
        return false;
    }
    
    // Check for finite values
    for (size_t i = 0; i < joint_positions.size(); ++i) {
        if (!joint_positions[i].allFinite()) {
            std::cerr << "❌ Robot bridge: Joint " << i << " contains non-finite values" << std::endl;
            return false;
        }
        
        // Check for reasonable coordinate ranges (millimeters)
        const auto& joint = joint_positions[i];
        if (joint.norm() > 10000.0) {  // 10 meters seems reasonable for robot reach
            std::cerr << "⚠️  Robot bridge: Joint " << i << " is very far from origin: " 
                      << joint.norm() << "mm" << std::endl;
            // Continue anyway - might be a large robot
        }
    }
    
    return true;
}

void RobotCollisionBridge::debug_print(const std::string& message, bool debug_logging) {
    if (debug_logging) {
        std::cout << "  " << message << std::endl;
    }
}

} // namespace delta