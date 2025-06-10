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

std::vector<Eigen::Vector3d> FabrikForwardBlock::single_forward_pass(const std::vector<Eigen::Vector3d>& original_positions,
                                                                    const Eigen::Vector3d& target_position,
                                                                    std::vector<double>& recalculated_lengths) {
    std::vector<Eigen::Vector3d> updated_positions = original_positions;
    int num_joints = static_cast<int>(updated_positions.size());
    int total_segments = num_joints - 1;
    
    if (num_joints == 0) {
        return updated_positions;
    }
    
    // Step 1: Fix base joint at origin
    updated_positions[0] = Eigen::Vector3d(0.0, 0.0, 0.0);
    
    // Step 2: Work forward from joint 1 to end-effector
    for (int i = 1; i < num_joints; ++i) {
        // Previous joint already positioned in this forward pass
        const Eigen::Vector3d& J_prev_prime = updated_positions[i - 1];
        
        // Original position of current joint before this pass
        const Eigen::Vector3d& J_current_original = original_positions[i];
        
        // Calculate desired direction: from J_prev_prime toward J_current_original
        Eigen::Vector3d desired_direction = J_current_original - J_prev_prime;
        
        // Extract direction pairs for segment calculation
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> direction_pairs = 
            extract_direction_pairs(updated_positions, i);
        
        // Calculate new segment length using SegmentBlock
        Eigen::Vector3d current_direction, previous_direction;
        
        if (!direction_pairs.empty()) {
            current_direction = direction_pairs.back().second;   // Target direction for this segment
            previous_direction = direction_pairs.back().first;   // Reference direction
        } else {
            // Fallback for first segment
            current_direction = desired_direction.normalized();
            previous_direction = Eigen::Vector3d(0, 0, 1); // Z+ default
        }
        
        // Calculate new segment length dynamically
        double new_segment_length = calculate_new_segment_length(current_direction, previous_direction, 
                                                               i - 1, total_segments);
        
        // Store the recalculated length
        recalculated_lengths.push_back(new_segment_length);
        
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
            updated_positions[i] = J_prev_prime + placement_direction * new_segment_length;
        } else {
            // Fallback direction if calculation fails
            Eigen::Vector3d fallback_direction;
            if (i + 1 < num_joints) {
                // Point toward next joint
                fallback_direction = original_positions[i + 1] - J_prev_prime;
            } else {
                // Last joint - point toward target
                fallback_direction = target_position - J_prev_prime;
            }
            
            if (fallback_direction.norm() < 1e-10) {
                fallback_direction = Eigen::Vector3d(0, 0, 1); // Ultimate fallback
            }
            
            updated_positions[i] = J_prev_prime + fallback_direction.normalized() * new_segment_length;
        }
    }
    
    return updated_positions;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> FabrikForwardBlock::extract_direction_pairs(
    const std::vector<Eigen::Vector3d>& joint_positions, int up_to_joint) {
    
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> direction_pairs;
    
    // Extract direction pairs for segments up to current joint
    for (int seg = 0; seg < up_to_joint - 1; seg++) {
        if (seg + 2 < static_cast<int>(joint_positions.size())) {
            // Reference direction: [seg] → [seg+1]
            Eigen::Vector3d ref_direction = (joint_positions[seg + 1] - joint_positions[seg]).normalized();
            
            // Target direction: [seg+1] → [seg+2]
            Eigen::Vector3d target_direction = (joint_positions[seg + 2] - joint_positions[seg + 1]).normalized();
            
            direction_pairs.push_back(std::make_pair(ref_direction, target_direction));
        }
    }
    
    return direction_pairs;
}

double FabrikForwardBlock::calculate_new_segment_length(const Eigen::Vector3d& current_direction,
                                                       const Eigen::Vector3d& previous_direction,
                                                       int segment_index, int total_segments) {
    try {
        // DEBUG: Print input directions
        std::cout << "  [DEBUG] Segment " << segment_index << " calculation:" << std::endl;
        std::cout << "    Current Direction: (" << current_direction.x() << ", " << current_direction.y() << ", " << current_direction.z() << ")" << std::endl;
        std::cout << "    Previous Direction: (" << previous_direction.x() << ", " << previous_direction.y() << ", " << previous_direction.z() << ")" << std::endl;
        
        // Use SegmentBlock to calculate prismatic length with coordinate transformation
        SegmentResult segment_result = SegmentBlock::calculate_essential(current_direction, previous_direction);
        
        // DEBUG: Print SegmentBlock result
        std::cout << "    SegmentBlock Success: " << segment_result.calculation_successful << std::endl;
        std::cout << "    Prismatic Length: " << segment_result.prismatic_length << std::endl;
        
        if (!segment_result.calculation_successful) {
            // Fallback to minimum segment length
            std::cout << "    Using fallback (calculation failed)" << std::endl;
            double fallback = convert_prismatic_to_fabrik_length(0.0, segment_index, total_segments);
            std::cout << "    Final FABRIK Length: " << fallback << std::endl;
            return fallback;
        }
        
        // Convert prismatic length to FABRIK segment length
        double fabrik_length = convert_prismatic_to_fabrik_length(segment_result.prismatic_length, segment_index, total_segments);
        
        // DEBUG: Print conversion result
        std::cout << "    Final FABRIK Length: " << fabrik_length << std::endl;
        
        return fabrik_length;
        
    } catch (const std::exception& e) {
        // DEBUG: Print error
        std::cout << "    ERROR: " << e.what() << std::endl;
        std::cout << "    Using fallback (exception)" << std::endl;
        
        // Fallback calculation for errors
        double fallback = convert_prismatic_to_fabrik_length(0.0, segment_index, total_segments);
        std::cout << "    Final FABRIK Length: " << fallback << std::endl;
        return fallback;
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
    
    // Cone setup for forward pass (different from backward)
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