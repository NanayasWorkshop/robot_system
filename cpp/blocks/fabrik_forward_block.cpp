#include "fabrik_forward_block.hpp"
#include <cmath>
#include <algorithm>

namespace delta {

FabrikForwardResult FabrikForwardBlock::calculate(const std::vector<Eigen::Vector3d>& joint_positions,
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
            throw std::invalid_argument("Segment lengths must match joint positions");
        }
        
        // Perform single forward pass with FIXED segment lengths
        std::vector<Eigen::Vector3d> updated_positions = single_forward_pass(joint_positions, target_position, segment_lengths);
        
        // Calculate distance to target
        double distance_to_target = calculate_distance_to_target(updated_positions, target_position);
        
        // Return same segment lengths (no recalculation)
        std::vector<double> unchanged_lengths = segment_lengths;
        
        return FabrikForwardResult(updated_positions, unchanged_lengths, distance_to_target, calculation_time_ms);
    }
}

std::vector<Eigen::Vector3d> FabrikForwardBlock::single_forward_pass(const std::vector<Eigen::Vector3d>& joint_positions,
                                                                    const Eigen::Vector3d& target_position,
                                                                    const std::vector<double>& segment_lengths) {
    // *** FIX: Create separate arrays for input (NEVER modified) and output (working copy) ***
    const std::vector<Eigen::Vector3d>& original_positions = joint_positions;  // FROZEN reference - never touch!
    std::vector<Eigen::Vector3d> updated_positions = joint_positions;          // Working copy - modify freely
    
    int num_joints = static_cast<int>(updated_positions.size());
    
    if (num_joints == 0) {
        return updated_positions;
    }
    
    // Step 1: Fix base joint at origin
    updated_positions[0] = Eigen::Vector3d(0.0, 0.0, 0.0);
    
    // Step 2: Work forward from joint 1 to end-effector
    for (int i = 1; i < num_joints; ++i) {
        // J_prev: Previous joint already positioned in this forward pass (UPDATED position)
        const Eigen::Vector3d& J_prev = updated_positions[i - 1];
        
        // *** FIX: J_current_original - ALWAYS use ORIGINAL input position ***
        const Eigen::Vector3d& J_current_original = original_positions[i];  // ← NEVER use updated_positions[i]!
        
        // Use FIXED segment length (no recalculation)
        double current_segment_length = segment_lengths[i - 1];
        
        // Calculate desired direction: from J_prev toward J_current_original
        Eigen::Vector3d desired_direction = J_current_original - J_prev;
        
        // Special case: First joint after base (J1) must go straight up in Z+
        Eigen::Vector3d final_direction;
        if (i == 1) {
            final_direction = Eigen::Vector3d(0, 0, 1); // Force straight up
        } else {
            // Apply cone constraint if needed
            final_direction = apply_cone_constraint_if_needed(desired_direction, original_positions, updated_positions, i);
        }
        
        // Place joint at fixed segment length distance
        if (final_direction.norm() > 1e-10) {
            Eigen::Vector3d placement_direction = final_direction.normalized();
            updated_positions[i] = J_prev + placement_direction * current_segment_length;
        } else {
            // Fallback direction if calculation fails
            Eigen::Vector3d fallback_direction;
            if (i + 1 < num_joints) {
                // Point toward next joint (use ORIGINAL position)
                fallback_direction = original_positions[i + 1] - J_prev;
            } else {
                // Last joint - point toward target
                fallback_direction = target_position - J_prev;
            }
            
            if (fallback_direction.norm() < 1e-10) {
                fallback_direction = Eigen::Vector3d(0, 0, 1); // Ultimate fallback
            }
            
            updated_positions[i] = J_prev + fallback_direction.normalized() * current_segment_length;
        }
    }
    
    return updated_positions;
}

Eigen::Vector3d FabrikForwardBlock::apply_cone_constraint_if_needed(const Eigen::Vector3d& desired_direction,
                                                                   const std::vector<Eigen::Vector3d>& original_positions,
                                                                   const std::vector<Eigen::Vector3d>& updated_positions,
                                                                   int joint_index) {
    // Apply cone constraint for spherical joints (forward version)
    // Check if we can apply cone constraint
    // Need joint i-1 (apex) and joint i-2 (direction reference) for forward pass
    bool can_apply_constraint = (joint_index >= 2);
    
    if (!can_apply_constraint || desired_direction.norm() < 1e-10) {
        return desired_direction;
    }
    
    // *** FIX: Use UPDATED position for apex (already placed), ORIGINAL for direction reference ***
    const Eigen::Vector3d& J_apex_cone = updated_positions[joint_index - 1];    // Already placed in this pass
    const Eigen::Vector3d& J_direction_ref = original_positions[joint_index - 2]; // Use ORIGINAL reference
    
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

double FabrikForwardBlock::calculate_distance_to_target(const std::vector<Eigen::Vector3d>& joint_positions,
                                                       const Eigen::Vector3d& target_position) {
    if (joint_positions.empty()) {
        return 0.0;
    }
    
    // Distance from end-effector (last joint) to target
    Eigen::Vector3d end_effector = joint_positions.back();
    
    return (end_effector - target_position).norm();
}

} // namespace delta