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
    
    debug_print("Converting FABRIK refinement result to capsules (robot coordinates)", debug_logging);
    
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
    
    debug_print("Created " + std::to_string(capsule_result.capsules.size()) + " robot capsules (Z-up, mm)", debug_logging);
    
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
    
    debug_print("Converting basic FABRIK result to capsules (robot coordinates)", debug_logging);
    
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
    
    debug_print("Created " + std::to_string(capsule_result.capsules.size()) + " robot capsules (Z-up, mm)", debug_logging);
    
    return BridgeResult<std::vector<CapsuleData>>(capsule_result.capsules, total_time_ms);
}

// =============================================================================
// NEW: STAR COORDINATE TRANSFORMATION INTERFACE
// =============================================================================

BridgeResult<std::vector<CapsuleData>> RobotCollisionBridge::convert_fabrik_to_capsules_star_coords(
    const FabrikRefinementResult& fabrik_result,
    double robot_radius,
    const Eigen::Vector3d& robot_offset,
    bool debug_logging) {
    
    double total_time_ms = 0.0;
    ScopedTimer timer(total_time_ms);
    
    debug_print("Converting FABRIK refinement result to STAR coordinate capsules with offset", debug_logging);
    
    // First get capsules in robot coordinates
    auto robot_result = convert_fabrik_to_capsules(fabrik_result, robot_radius, debug_logging);
    if (!robot_result.success) {
        return robot_result;
    }
    
    // Transform to STAR coordinates with offset
    auto star_result = transform_capsules_robot_to_star(robot_result.data, robot_offset, debug_logging);
    if (!star_result.success) {
        return BridgeResult<std::vector<CapsuleData>>("Robot→STAR transformation failed: " + star_result.error_message, total_time_ms);
    }
    
    debug_print("Transformed " + std::to_string(star_result.data.size()) + " capsules to STAR coordinates (Y-up, meters) with offset", debug_logging);
    
    return BridgeResult<std::vector<CapsuleData>>(star_result.data, total_time_ms);
}

BridgeResult<std::vector<CapsuleData>> RobotCollisionBridge::convert_basic_fabrik_to_capsules_star_coords(
    const FabrikSolverResult& fabrik_result,
    double robot_radius,
    const Eigen::Vector3d& robot_offset,
    bool debug_logging) {
    
    double total_time_ms = 0.0;
    ScopedTimer timer(total_time_ms);
    
    debug_print("Converting basic FABRIK result to STAR coordinate capsules with offset", debug_logging);
    
    // First get capsules in robot coordinates
    auto robot_result = convert_basic_fabrik_to_capsules(fabrik_result, robot_radius, debug_logging);
    if (!robot_result.success) {
        return robot_result;
    }
    
    // Transform to STAR coordinates with offset
    auto star_result = transform_capsules_robot_to_star(robot_result.data, robot_offset, debug_logging);
    if (!star_result.success) {
        return BridgeResult<std::vector<CapsuleData>>("Robot→STAR transformation failed: " + star_result.error_message, total_time_ms);
    }
    
    debug_print("Transformed " + std::to_string(star_result.data.size()) + " capsules to STAR coordinates (Y-up, meters) with offset", debug_logging);
    
    return BridgeResult<std::vector<CapsuleData>>(star_result.data, total_time_ms);
}

// =============================================================================
// PRIVATE COORDINATE TRANSFORMATION IMPLEMENTATIONS
// =============================================================================

Eigen::Vector3d RobotCollisionBridge::transform_robot_to_star(const Eigen::Vector3d& robot_point, 
                                                           const Eigen::Vector3d& offset) {
    // Robot coordinate system: Z-up, millimeters
    // STAR coordinate system: Y-up, meters
    
    // Step 1: Transform and scale
    // X stays X (left-right unchanged)
    // Y (forward in robot) becomes -Z (backward in STAR, to match orientation)  
    // Z (up in robot) becomes Y (up in STAR)
    // Scale: millimeters → meters (÷ 1000)
    
    Eigen::Vector3d star_point(
        robot_point.x() / 1000.0,    // X unchanged, mm → meters
        robot_point.z() / 1000.0,    // Y = Z (Z-up → Y-up)
        -robot_point.y() / 1000.0    // Z = -Y (flip to match STAR orientation)
    );
    
    // Step 2: Add offset in STAR coordinates (meters)
    return star_point + offset;
}

BridgeResult<std::vector<CapsuleData>> RobotCollisionBridge::transform_capsules_robot_to_star(
    const std::vector<CapsuleData>& robot_capsules,
    const Eigen::Vector3d& offset,
    bool debug_logging) {
    
    double transform_time_ms = 0.0;
    ScopedTimer timer(transform_time_ms);
    
    if (robot_capsules.empty()) {
        return BridgeResult<std::vector<CapsuleData>>("No robot capsules to transform", transform_time_ms);
    }
    
    debug_print("Transforming " + std::to_string(robot_capsules.size()) + " capsules: Robot (Z-up, mm) → STAR (Y-up, meters) + offset(" +
               std::to_string(offset.x()) + ", " + std::to_string(offset.y()) + ", " + std::to_string(offset.z()) + ")", debug_logging);
    
    std::vector<CapsuleData> star_capsules;
    star_capsules.reserve(robot_capsules.size());
    
    for (size_t i = 0; i < robot_capsules.size(); ++i) {
        const auto& robot_capsule = robot_capsules[i];
        
        CapsuleData star_capsule;
        star_capsule.start_point = transform_robot_to_star(robot_capsule.start_point, offset);
        star_capsule.end_point = transform_robot_to_star(robot_capsule.end_point, offset);
        star_capsule.radius = robot_capsule.radius / 1000.0;  // mm → meters
        star_capsule.length = robot_capsule.length / 1000.0;  // mm → meters
        
        star_capsules.push_back(star_capsule);
        
        // Debug first few capsules if enabled
        if (debug_logging && i < 3) {
            debug_print("Capsule " + std::to_string(i) + ": " +
                       "Robot(" + std::to_string(robot_capsule.start_point.x()) + ", " + 
                       std::to_string(robot_capsule.start_point.y()) + ", " + 
                       std::to_string(robot_capsule.start_point.z()) + ") → " +
                       "STAR(" + std::to_string(star_capsule.start_point.x()) + ", " + 
                       std::to_string(star_capsule.start_point.y()) + ", " + 
                       std::to_string(star_capsule.start_point.z()) + ")", debug_logging);
        }
    }
    
    debug_print("Transformation complete: " + std::to_string(star_capsules.size()) + " capsules ready for STAR collision with offset applied", debug_logging);
    
    return BridgeResult<std::vector<CapsuleData>>(star_capsules, transform_time_ms);
}

// =============================================================================
// PRIVATE HELPER IMPLEMENTATIONS (unchanged)
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
        
        // Check for reasonable coordinate ranges (millimeters for robot)
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