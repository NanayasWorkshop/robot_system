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
            
            std::vector<Eigen::Vector3d> joint_positions = init_result.joint_positions;
            
            // Step 2: Calculate initial segment lengths  
            std::vector<double> segment_lengths = calculate_initial_segment_lengths(joint_positions);
            
            // Step 3: Convergence loop (always end on forward pass)
            int iteration = 0;
            double current_error = calculate_distance_to_target(joint_positions, target_position);
            
            // Check if already at target
            if (current_error <= tolerance) {
                return FabrikSolverResult(joint_positions, joint_positions.back(), 
                                        true, current_error, 0, solve_time_ms);
            }
            
            while (iteration < max_iterations) {
                
                // Backward pass (end-effector → target)
                FabrikBackwardResult backward_result = FabrikBackwardBlock::calculate(
                    joint_positions, target_position, segment_lengths);
                
                // FIXED: Update chain state with backward result (like working FABRIK)
                joint_positions = backward_result.updated_joint_positions;
                
                // Forward pass (base → origin, recalculate lengths)
                FabrikForwardResult forward_result = FabrikForwardBlock::calculate(
                    joint_positions, target_position);
                
                // FIXED: Update chain state with forward result (like working FABRIK)
                joint_positions = forward_result.updated_joint_positions;     // Base always at origin after forward
                segment_lengths = forward_result.recalculated_segment_lengths; // CRITICAL: Use updated lengths!
                
                // Check convergence (after forward pass only)
                current_error = calculate_distance_to_target(joint_positions, target_position);
                iteration++;
                
                // Exit if converged
                if (current_error <= tolerance) {
                    break;
                }
            }
            
            // Package results
            bool converged = (current_error <= tolerance);
            Eigen::Vector3d achieved_position = joint_positions.back();
            
            return FabrikSolverResult(joint_positions, achieved_position, converged, 
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