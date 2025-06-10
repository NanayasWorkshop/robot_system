#include "fabrik_backward_block.hpp"
#include <cmath>
#include <algorithm>

namespace delta {

FabrikBackwardResult FabrikBackwardBlock::calculate(const std::vector<Eigen::Vector3d>& joint_positions,
                                                   const Eigen::Vector3d& target_position,
                                                   const std::vector<double>& segment_lengths) {
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        // Input validation
        if (joint_positions.empty()) {
            throw std::invalid_argument("Joint positions cannot be empty");
        }
        if (segment_lengths.size() != joint_positions.size() - 1) {
            throw std::invalid_argument("Segment lengths must be one less than joint positions");
        }
        
        // Perform single backward pass
        std::vector<Eigen::Vector3d> updated_positions = single_backward_pass(joint_positions, target_position, segment_lengths);
        
        // Calculate distance to base
        double distance_to_base = calculate_distance_to_base(updated_positions);
        
        return FabrikBackwardResult(updated_positions, distance_to_base, calculation_time_ms);
    }
}

std::vector<Eigen::Vector3d> FabrikBackwardBlock::single_backward_pass(const std::vector<Eigen::Vector3d>& joint_positions,
                                                                      const Eigen::Vector3d& target_position,
                                                                      const std::vector<double>& segment_lengths) {
    // *** FIX: Create separate arrays for input (NEVER modified) and output (working copy) ***
    const std::vector<Eigen::Vector3d>& original_positions = joint_positions;  // FROZEN reference - never touch!
    std::vector<Eigen::Vector3d> updated_positions = joint_positions;          // Working copy - modify freely
    
    int num_joints = static_cast<int>(updated_positions.size());
    
    if (num_joints == 0) {
        return updated_positions;
    }
    
    // Step 1: Set end-effector (last joint) to target position
    updated_positions[num_joints - 1] = target_position;
    
    // Step 2: Work backwards from second-to-last joint to base
    for (int i = num_joints - 2; i >= 0; --i) {
        // J_next: The joint already positioned in this backward pass (UPDATED position)
        const Eigen::Vector3d& J_next = updated_positions[i + 1];
        
        // *** FIX: J_current_original - ALWAYS use ORIGINAL input position ***
        const Eigen::Vector3d& J_current_original = original_positions[i];  // ← NEVER use updated_positions[i]!
        
        // Get segment length for this segment
        double segment_length = segment_lengths[i];
        
        if (segment_length < 1e-10) {
            // Zero length segment - place at same position
            updated_positions[i] = J_next;
            continue;
        }
        
        // Calculate desired direction: from J_next toward J_current_original
        Eigen::Vector3d desired_direction = J_current_original - J_next;
        
        // Apply cone constraint if needed (for spherical joints)
        Eigen::Vector3d final_direction = apply_cone_constraint_if_needed(desired_direction, original_positions, updated_positions, i);
        
        // Place joint i at segment_length distance from J_next in final direction
        if (final_direction.norm() > 1e-10) {
            Eigen::Vector3d placement_direction = final_direction.normalized();
            updated_positions[i] = J_next + placement_direction * segment_length;
        } else {
            // Fallback direction if calculation fails
            Eigen::Vector3d fallback_direction;
            if (i > 0) {
                // Point toward previous joint (use ORIGINAL position)
                fallback_direction = original_positions[i-1] - J_next;
            } else {
                // Base joint - point toward origin
                fallback_direction = Eigen::Vector3d(0, 0, 0) - J_next;
            }
            
            if (fallback_direction.norm() < 1e-10) {
                fallback_direction = Eigen::Vector3d(0, 0, -1); // Ultimate fallback
            }
            
            updated_positions[i] = J_next + fallback_direction.normalized() * segment_length;
        }
    }
    
    return updated_positions;
}

Eigen::Vector3d FabrikBackwardBlock::apply_cone_constraint_if_needed(const Eigen::Vector3d& desired_direction,
                                                                    const std::vector<Eigen::Vector3d>& original_positions,
                                                                    const std::vector<Eigen::Vector3d>& updated_positions,
                                                                    int joint_index) {
    // Apply cone constraint for spherical joints
    int num_joints = static_cast<int>(original_positions.size());
    
    // Check if we can apply cone constraint
    // Need joint i+1 (apex) and joint i+2 (direction reference)
    bool can_apply_constraint = (joint_index + 2 < num_joints);
    
    if (!can_apply_constraint || desired_direction.norm() < 1e-10) {
        return desired_direction;
    }
    
    // *** FIX: Use UPDATED position for apex (already placed), ORIGINAL for direction reference ***
    const Eigen::Vector3d& J_apex_cone = updated_positions[joint_index + 1];  // Already placed in this pass
    const Eigen::Vector3d& J_direction_ref = original_positions[joint_index + 2];  // Use ORIGINAL reference
    
    // Cone axis points from reference point toward apex
    Eigen::Vector3d cone_axis = J_apex_cone - J_direction_ref;
    
    if (cone_axis.norm() < 1e-10) {
        return desired_direction; // Cannot define cone axis
    }
    
    // Apply cone constraint using ConeConstraintBlock
    double cone_angle_rad = SPHERICAL_JOINT_CONE_ANGLE_RAD;
    ConeConstraintResult constraint_result = ConeConstraintBlock::calculate(
        desired_direction, J_apex_cone, cone_axis, cone_angle_rad);
    
    return constraint_result.projected_direction;
}

double FabrikBackwardBlock::calculate_distance_to_base(const std::vector<Eigen::Vector3d>& joint_positions) {
    if (joint_positions.empty()) {
        return 0.0;
    }
    
    // Distance from end-effector (last joint) to base (first joint)
    Eigen::Vector3d base_position = joint_positions[0];
    Eigen::Vector3d end_position = joint_positions.back();
    
    return (end_position - base_position).norm();
}

} // namespace delta