#include "fabrik_solver_block.hpp"
#include <cmath>
#include <limits>

namespace delta {

FabrikSolverResult FabrikSolverBlock::solve(
    const Eigen::Vector3d& target_position,
    std::optional<std::vector<Eigen::Vector3d>> initial_joint_positions,
    int num_robot_segments,
    double tolerance,
    int max_iterations) {
    
    double solve_time_ms = 0.0;
    
    {
        ScopedTimer timer(solve_time_ms);
        
        try {
            // Step 1: Initialize chain
            FabrikInitializationResult init_result = FabrikInitializationBlock::calculate(
                num_robot_segments, initial_joint_positions);
            
            if (!init_result.validation_successful) {
                throw std::runtime_error("Chain initialization failed");
            }
            
            // Current joint positions - this gets updated each iteration
            std::vector<Eigen::Vector3d> current_joints = init_result.joint_positions;
            
            // Step 2: Calculate initial segment lengths  
            std::vector<double> segment_lengths = calculate_initial_segment_lengths(current_joints);
            
            // Step 3: Convergence loop
            int iteration = 0;
            double current_error = calculate_distance_to_target(current_joints, target_position);
            bool converged = false;
            
            // Check if already at target
            if (current_error <= tolerance) {
                converged = true;
            }
            
            while (iteration < max_iterations && !converged) {
                
                // Backward pass: target → base
                FabrikBackwardResult backward_result = FabrikBackwardBlock::calculate(
                    current_joints, target_position, segment_lengths);
                
                // *** FIX: Update current_joints immediately after backward pass ***
                current_joints = backward_result.updated_joint_positions;
                
                // Forward pass: base → end (using UPDATED joint positions from backward pass)
                FabrikForwardResult forward_result = FabrikForwardBlock::calculate(
                    current_joints, target_position, segment_lengths);
                
                // *** FIX: Update current_joints immediately after forward pass ***
                current_joints = forward_result.updated_joint_positions;
                
                // Check convergence
                current_error = calculate_distance_to_target(current_joints, target_position);
                iteration++;
                
                // Exit if converged
                if (current_error <= tolerance) {
                    converged = true;
                }
                
                // Calculate new segment lengths for next iteration (if not converged)
                if (!converged && iteration < max_iterations) {
                    segment_lengths = calculate_updated_segment_lengths(current_joints);
                }
            }
            
            // FINAL STEP: One more length update after convergence to ensure proper segments
            if (converged || iteration >= max_iterations) {
                segment_lengths = calculate_updated_segment_lengths(current_joints);
            }
            
            // Package results
            Eigen::Vector3d achieved_position = current_joints.back();
            
            return FabrikSolverResult(current_joints, achieved_position, converged, 
                                    current_error, iteration, solve_time_ms);
            
        } catch (const std::exception& e) {
            // Return failed result on any error
            std::vector<Eigen::Vector3d> empty_positions;
            Eigen::Vector3d zero_position(0.0, 0.0, 0.0);
            return FabrikSolverResult(empty_positions, zero_position, false, 
                                    std::numeric_limits<double>::max(), 0, solve_time_ms);
        }
    }
}

std::vector<double> FabrikSolverBlock::calculate_initial_segment_lengths(
    const std::vector<Eigen::Vector3d>& joint_positions) {
    
    std::vector<double> segment_lengths;
    segment_lengths.reserve(joint_positions.size() - 1);
    
    for (size_t i = 0; i < joint_positions.size() - 1; ++i) {
        double length = (joint_positions[i + 1] - joint_positions[i]).norm();
        segment_lengths.push_back(length);
    }
    
    return segment_lengths;
}

double FabrikSolverBlock::calculate_distance_to_target(
    const std::vector<Eigen::Vector3d>& joint_positions,
    const Eigen::Vector3d& target_position) {
    
    if (joint_positions.empty()) {
        return std::numeric_limits<double>::max();
    }
    
    // Distance from end-effector (last joint) to target
    Eigen::Vector3d end_effector = joint_positions.back();
    return (end_effector - target_position).norm();
}

std::vector<double> FabrikSolverBlock::calculate_updated_segment_lengths(
    const std::vector<Eigen::Vector3d>& joint_positions) {
    
    std::vector<double> new_lengths;
    new_lengths.reserve(joint_positions.size() - 1);
    
    int total_segments = static_cast<int>(joint_positions.size()) - 1;
    
    for (int i = 0; i < total_segments; i++) {
        try {
            // Use SegmentBlock for accurate length calculation
            SegmentResult segment_result = SegmentBlock::calculate_essential_from_joints(
                joint_positions, i);
            
            if (segment_result.calculation_successful) {
                // Convert prismatic to FABRIK length
                double fabrik_length = convert_prismatic_to_fabrik_length(
                    segment_result.prismatic_length, i, total_segments);
                new_lengths.push_back(fabrik_length);
            } else {
                // Fallback: use current Euclidean distance
                double euclidean_length = (joint_positions[i+1] - joint_positions[i]).norm();
                new_lengths.push_back(euclidean_length);
            }
        } catch (const std::exception& e) {
            // Fallback on any error
            double euclidean_length = (joint_positions[i+1] - joint_positions[i]).norm();
            new_lengths.push_back(euclidean_length);
        }
    }
    
    return new_lengths;
}

double FabrikSolverBlock::convert_prismatic_to_fabrik_length(double prismatic_length, 
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

} // namespace delta