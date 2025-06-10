#include "fabrik_forward_block.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace delta {

FabrikForwardResult FabrikForwardBlock::calculate(const std::vector<Eigen::Vector3d>& joint_positions,
                                                 const Eigen::Vector3d& target_position) {
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        // Input validation
        if (joint_positions.empty()) {
            throw std::invalid_argument("Joint positions cannot be empty");
        }
        
        // Initialize recalculated lengths vector
        std::vector<double> recalculated_lengths;
        recalculated_lengths.reserve(joint_positions.size() - 1);
        
        // Perform single forward pass with dynamic segment recalculation
        std::vector<Eigen::Vector3d> updated_positions = single_forward_pass(joint_positions, target_position, recalculated_lengths);
        
        // Calculate distance to target
        double distance_to_target = calculate_distance_to_target(updated_positions, target_position);
        
        return FabrikForwardResult(updated_positions, recalculated_lengths, distance_to_target, calculation_time_ms);
    }
}

std::vector<Eigen::Vector3d> FabrikForwardBlock::single_forward_pass(const std::vector<Eigen::Vector3d>& joint_positions,
                                                                    const Eigen::Vector3d& target_position,
                                                                    std::vector<double>& recalculated_lengths) {
    std::vector<Eigen::Vector3d> updated_positions = joint_positions;
    int num_joints = static_cast<int>(updated_positions.size());
    int total_segments = num_joints - 1;
    
    if (num_joints == 0) {
        return updated_positions;
    }
    
    // Step 1: Fix base joint at origin
    updated_positions[0] = Eigen::Vector3d(0.0, 0.0, 0.0);
    
    // Step 2: Work forward from joint 1 to end-effector
    for (int i = 1; i < num_joints; ++i) {
        // J_prev: Previous joint already positioned in this forward pass
        const Eigen::Vector3d& J_prev = updated_positions[i - 1];
        
        // J_current_original: Where this joint WAS before this pass
        const Eigen::Vector3d& J_current_original = joint_positions[i];
        
        // Calculate new segment length using complete joint chain with SegmentBlock
        double new_segment_length = calculate_new_segment_length_from_complete_chain(
            joint_positions,  // Use original chain for full context
            i - 1,           // Segment index (0-based: i=1 → segment 0, i=2 → segment 1, etc.)
            total_segments   // Total number of segments
        );
        
        // Store the recalculated length
        recalculated_lengths.push_back(new_segment_length);
        
        // Calculate desired direction: from J_prev toward J_current_original
        Eigen::Vector3d desired_direction = J_current_original - J_prev;
        
        // Special case: First joint after base (J1) must go straight up in Z+
        Eigen::Vector3d final_direction;
        if (i == 1) {
            final_direction = Eigen::Vector3d(0, 0, 1); // Force straight up
        } else {
            // Apply cone constraint if needed
            final_direction = apply_cone_constraint_if_needed(desired_direction, updated_positions, i);
        }
        
        // Place joint at new segment length distance
        if (final_direction.norm() > 1e-10) {
            Eigen::Vector3d placement_direction = final_direction.normalized();
            updated_positions[i] = J_prev + placement_direction * new_segment_length;
        } else {
            // Fallback direction if calculation fails
            Eigen::Vector3d fallback_direction;
            if (i + 1 < num_joints) {
                // Point toward next joint
                fallback_direction = joint_positions[i + 1] - J_prev;
            } else {
                // Last joint - point toward target
                fallback_direction = target_position - J_prev;
            }
            
            if (fallback_direction.norm() < 1e-10) {
                fallback_direction = Eigen::Vector3d(0, 0, 1); // Ultimate fallback
            }
            
            updated_positions[i] = J_prev + fallback_direction.normalized() * new_segment_length;
        }
    }
    
    return updated_positions;
}

double FabrikForwardBlock::calculate_new_segment_length_from_complete_chain(const std::vector<Eigen::Vector3d>& complete_joint_chain,
                                                                           int segment_index, int total_segments) {
    try {
        // Use SegmentBlock with complete joint chain context
        SegmentResult segment_result = SegmentBlock::calculate_essential_from_joints(
            complete_joint_chain, segment_index);
        
        if (!segment_result.calculation_successful) {
            // Fallback to minimum segment length
            return convert_prismatic_to_fabrik_length(0.0, segment_index, total_segments);
        }
        
        // Convert prismatic length to FABRIK segment length
        double fabrik_length = convert_prismatic_to_fabrik_length(
            segment_result.prismatic_length, segment_index, total_segments);
        
        return fabrik_length;
        
    } catch (const std::exception& e) {
        // Fallback calculation for errors
        return convert_prismatic_to_fabrik_length(0.0, segment_index, total_segments);
    }
}

double FabrikForwardBlock::convert_prismatic_to_fabrik_length(double prismatic_length, 
                                                            int segment_index, int total_segments) {
    // H→G distance from prismatic length
    double h_to_g_distance = MIN_HEIGHT + 2.0 * MOTOR_LIMIT + prismatic_length;
    
    // Convert to FABRIK segment length using the same pattern as initialization
    if (segment_index == 0) {
        // First FABRIK segment: WORKING_HEIGHT + h_to_g_distance/2
        return WORKING_HEIGHT + h_to_g_distance / 2.0;
    } else if (segment_index == total_segments - 1) {
        // Last FABRIK segment: h_to_g_distance/2 + WORKING_HEIGHT
        return h_to_g_distance / 2.0 + WORKING_HEIGHT;
    } else {
        // Middle FABRIK segments: h_to_g_distance/2 + 2*WORKING_HEIGHT + h_to_g_distance/2
        return h_to_g_distance / 2.0 + 2.0 * WORKING_HEIGHT + h_to_g_distance / 2.0;
    }
}

Eigen::Vector3d FabrikForwardBlock::apply_cone_constraint_if_needed(const Eigen::Vector3d& desired_direction,
                                                                   const std::vector<Eigen::Vector3d>& joint_positions,
                                                                   int joint_index) {
    // Apply cone constraint for spherical joints (forward version)
    // Check if we can apply cone constraint
    // Need joint i-1 (apex) and joint i-2 (direction reference) for forward pass
    bool can_apply_constraint = (joint_index >= 2);
    
    if (!can_apply_constraint || desired_direction.norm() < 1e-10) {
        return desired_direction;
    }
    
    // Cone setup for forward pass
    const Eigen::Vector3d& J_apex_cone = joint_positions[joint_index - 1];    // Previous joint is apex
    const Eigen::Vector3d& J_direction_ref = joint_positions[joint_index - 2]; // Joint before that is reference
    
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