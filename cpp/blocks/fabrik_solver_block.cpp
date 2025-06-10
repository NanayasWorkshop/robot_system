#include "fabrik_solver_block.hpp"
#include <cmath>

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
            
            // Check if already at target
            if (current_error <= tolerance) {
                return FabrikSolverResult(current_joints, current_joints.back(), 
                                        true, current_error, 0, solve_time_ms);
            }
            
            while (iteration < max_iterations) {
                
                // Backward pass: target → base
                FabrikBackwardResult backward_result = FabrikBackwardBlock::calculate(
                    current_joints, target_position, segment_lengths);
                
                // Forward pass: base → end (recalculates segment lengths)
                FabrikForwardResult forward_result = FabrikForwardBlock::calculate(
                    backward_result.updated_joint_positions, target_position);
                
                // UPDATE: Use forward pass output as new input for next iteration
                current_joints = forward_result.updated_joint_positions;
                segment_lengths = forward_result.recalculated_segment_lengths;
                
                // Check convergence
                current_error = calculate_distance_to_target(current_joints, target_position);
                iteration++;
                
                // Exit if converged
                if (current_error <= tolerance) {
                    break;
                }
            }
            
            // Package results
            bool converged = (current_error <= tolerance);
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

} // namespace delta