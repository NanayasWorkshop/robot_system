#include "fabrik_solver_block.hpp"
#include <cmath>
#include <numeric>

namespace delta {

FabrikSolverResult FabrikSolverBlock::solve(
    const Eigen::Vector3d& target_position,
    int num_segments,
    double tolerance,
    int max_iterations) {
    
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        // Step 1: Input validation
        if (!validate_solve_inputs(target_position, num_segments, tolerance, max_iterations)) {
            return FabrikSolverResult(calculation_time_ms, 
                "Invalid solver inputs: check target, segments, tolerance, and iteration parameters");
        }
        
        try {
            // Step 2: Initialize joint chain
            FabrikInitResult init_result = FabrikInitializationBlock::create_straight_chain(num_segments);
            
            if (!init_result.initialization_successful) {
                return FabrikSolverResult(calculation_time_ms, 
                    "Initialization failed: " + init_result.error_message);
            }
            
            // Step 3: Optional reachability check
            if (!is_target_potentially_reachable(target_position, init_result.joint_distances)) {
                return FabrikSolverResult(calculation_time_ms, 
                    "Target potentially unreachable: distance exceeds maximum chain reach");
            }
            
            // Step 4: Perform iterative solving
            return solve_iterative(target_position, init_result, tolerance, max_iterations);
            
        } catch (const std::exception& e) {
            return FabrikSolverResult(calculation_time_ms, 
                "FABRIK solving failed: " + std::string(e.what()));
        }
    }
}

bool FabrikSolverBlock::validate_solve_inputs(const Eigen::Vector3d& target,
                                             int num_segments,
                                             double tolerance,
                                             int max_iterations) {
    // Check segments
    if (num_segments <= 0 || num_segments > 1000) {
        return false;
    }
    
    // Check tolerance
    if (tolerance <= 0.0 || tolerance > 1000.0) {
        return false;
    }
    
    // Check iterations
    if (max_iterations <= 0 || max_iterations > 10000) {
        return false;
    }
    
    // Check target for NaN or infinite values
    if (!target.allFinite()) {
        return false;
    }
    
    return true;
}

FabrikSolverResult FabrikSolverBlock::solve_iterative(
    const Eigen::Vector3d& target,
    const FabrikInitResult& init_result,
    double tolerance,
    int max_iterations) {
    
    std::vector<Eigen::Vector3d> current_joints = init_result.initial_joints;
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        for (int iteration = 0; iteration < max_iterations; iteration++) {
            // Check initial convergence (before any iterations)
            double initial_distance = (current_joints.back() - target).norm();
            if (check_convergence(initial_distance, tolerance)) {
                return FabrikSolverResult(current_joints, iteration, initial_distance, true,
                                        calculation_time_ms, init_result);
            }
            
            // Perform one FABRIK iteration
            FabrikIterationResult iter_result = FabrikIterationBlock::iterate(
                current_joints, target, init_result.joint_distances
            );
            
            if (!iter_result.iteration_successful) {
                return FabrikSolverResult(calculation_time_ms, 
                    "Iteration " + std::to_string(iteration + 1) + " failed: " + iter_result.error_message);
            }
            
            // Update current joints
            current_joints = iter_result.updated_joints;
            
            // Check convergence
            if (check_convergence(iter_result.distance_to_target, tolerance)) {
                return FabrikSolverResult(current_joints, iteration + 1, iter_result.distance_to_target, true,
                                        calculation_time_ms, init_result);
            }
        }
        
        // Max iterations reached without convergence
        double final_distance = (current_joints.back() - target).norm();
        return FabrikSolverResult(current_joints, max_iterations, final_distance, false,
                                calculation_time_ms, init_result);
    }
}

bool FabrikSolverBlock::check_convergence(double distance_to_target, double tolerance) {
    return distance_to_target <= tolerance;
}

bool FabrikSolverBlock::is_target_potentially_reachable(const Eigen::Vector3d& target,
                                                       const std::vector<double>& joint_distances) {
    // Calculate maximum possible reach (sum of all joint distances)
    double max_reach = std::accumulate(joint_distances.begin(), joint_distances.end(), 0.0);
    
    // Calculate distance from base (origin) to target
    double target_distance = target.norm();
    
    // Add small buffer for numerical tolerance
    double reach_buffer = 1.1; // 10% buffer
    
    return target_distance <= (max_reach * reach_buffer);
}

} // namespace delta