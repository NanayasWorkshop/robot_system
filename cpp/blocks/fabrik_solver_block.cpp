#include "fabrik_solver_block.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>

namespace delta {

FabrikSolverResult FabrikSolverBlock::solve(
    const Eigen::Vector3d& target_position,
    int num_segments,
    double tolerance,
    int max_iterations) {
    
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        if (!validate_solve_inputs(target_position, num_segments, tolerance, max_iterations)) {
            return FabrikSolverResult(calculation_time_ms, 
                "Invalid solver inputs: check target, segments, tolerance, and iteration parameters");
        }
        
        try {
            FabrikInitResult init_result = FabrikInitializationBlock::create_straight_chain(num_segments);
            
            if (!init_result.initialization_successful) {
                return FabrikSolverResult(calculation_time_ms, 
                    "Initialization failed: " + init_result.error_message);
            }
            
            return solve_iterative(target_position, init_result, tolerance, max_iterations);
            
        } catch (const std::exception& e) {
            return FabrikSolverResult(calculation_time_ms, 
                "FABRIK solving failed: " + std::string(e.what()));
        }
    }
}

FabrikRefinementResult FabrikSolverBlock::solve_with_prismatic_refinement(
    const Eigen::Vector3d& target_position,
    int num_segments,
    double fabrik_tolerance,
    int max_fabrik_iterations,
    double prismatic_tolerance,
    int max_refinement_iterations) {
    
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        if (!validate_refinement_inputs(target_position, num_segments, fabrik_tolerance, 
                                       max_fabrik_iterations, prismatic_tolerance, max_refinement_iterations)) {
            return FabrikRefinementResult(calculation_time_ms, 
                "Invalid refinement inputs: check all parameters");
        }
        
        try {
            FabrikInitResult init_result = FabrikInitializationBlock::create_straight_chain(num_segments);
            
            if (!init_result.initialization_successful) {
                return FabrikRefinementResult(calculation_time_ms, 
                    "Initialization failed: " + init_result.error_message);
            }
            
            return solve_refinement_iterative(target_position, init_result, fabrik_tolerance, 
                                             max_fabrik_iterations, prismatic_tolerance, max_refinement_iterations);
            
        } catch (const std::exception& e) {
            return FabrikRefinementResult(calculation_time_ms, 
                "FABRIK refinement solving failed: " + std::string(e.what()));
        }
    }
}

FabrikRefinementResult FabrikSolverBlock::solve_refinement_iterative(
    const Eigen::Vector3d& target,
    const FabrikInitResult& init_result,
    double fabrik_tolerance,
    int max_fabrik_iterations,
    double prismatic_tolerance,
    int max_refinement_iterations) {
    
    std::vector<Eigen::Vector3d> current_joints = init_result.initial_joints;
    std::vector<double> current_prismatic_lengths;
    std::vector<double> previous_prismatic_lengths;
    
    std::vector<Eigen::Vector3d> direction_vectors = extract_direction_vectors(current_joints);
    
    int fabrik_iterations_used = 0;
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        for (int refinement_iter = 0; refinement_iter < max_refinement_iterations; refinement_iter++) {
            // Solve FABRIK
            FabrikSolverResult fabrik_result = solve_iterative(target, 
                FabrikInitResult(current_joints, init_result.joint_distances, 
                               init_result.num_segments, init_result.num_joints, 0.0),
                fabrik_tolerance, max_fabrik_iterations);
            
            if (!fabrik_result.solving_successful) {
                return FabrikRefinementResult(calculation_time_ms, 
                    "FABRIK solve failed in refinement iteration " + std::to_string(refinement_iter + 1) + 
                    ": " + fabrik_result.error_message);
            }
            
            fabrik_iterations_used = fabrik_result.iterations_used;
            current_joints = fabrik_result.final_joints;
            
            // Extract prismatic lengths - SKIP S0, use segments 1-7
            previous_prismatic_lengths = current_prismatic_lengths;
            current_prismatic_lengths.clear();
            current_prismatic_lengths.reserve(init_result.num_segments);
            
            for (int segment_idx = 1; segment_idx <= init_result.num_segments; segment_idx++) {
                SegmentResult segment_result = SegmentBlock::calculate_essential_from_joints(
                    current_joints, segment_idx);
                
                if (!segment_result.calculation_successful) {
                    return FabrikRefinementResult(calculation_time_ms, 
                        "Segment calculation failed in refinement iteration " + std::to_string(refinement_iter + 1) + 
                        " for segment " + std::to_string(segment_idx) + ": " + segment_result.error_message);
                }
                
                current_prismatic_lengths.push_back(segment_result.prismatic_length);
            }
            
            // Check prismatic convergence
            if (refinement_iter > 0) {
                std::vector<double> prismatic_changes = calculate_prismatic_changes(
                    current_prismatic_lengths, previous_prismatic_lengths);
                
                if (check_prismatic_convergence(prismatic_changes, prismatic_tolerance)) {
                    return FabrikRefinementResult(current_joints, current_prismatic_lengths,
                                                fabrik_iterations_used, refinement_iter + 1,
                                                fabrik_result.final_distance_to_target, 
                                                fabrik_result.converged, true,
                                                calculation_time_ms, init_result, 
                                                (refinement_iter == 0) ? std::vector<double>() : previous_prismatic_lengths);
                }
            }
            
            // Build refined chain for next iteration
            if (refinement_iter < max_refinement_iterations - 1) {
                current_joints = build_refined_chain(direction_vectors, current_prismatic_lengths, init_result.num_segments);
            }
        }
        
        // Max iterations reached
        return FabrikRefinementResult(current_joints, current_prismatic_lengths,
                                    fabrik_iterations_used, max_refinement_iterations,
                                    (current_joints.back() - target).norm(),
                                    false, false, calculation_time_ms, init_result,
                                    previous_prismatic_lengths);
    }
}

std::vector<Eigen::Vector3d> FabrikSolverBlock::build_refined_chain(
    const std::vector<Eigen::Vector3d>& direction_vectors,
    const std::vector<double>& prismatic_lengths,
    int num_segments) {
    
    std::vector<Eigen::Vector3d> refined_joints;
    refined_joints.reserve(num_segments + 2);
    
    refined_joints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    
    for (int i = 0; i < num_segments + 1; i++) {
        Eigen::Vector3d direction = direction_vectors[i];
        double base_length;
        double prismatic_multiplier;
        
        if (i == 0) {
            base_length = WORKING_HEIGHT + MIN_HEIGHT/2.0 + MOTOR_LIMIT;
            prismatic_multiplier = 1.0;
        } else if (i == num_segments) {
            base_length = WORKING_HEIGHT + MIN_HEIGHT/2.0 + MOTOR_LIMIT;
            prismatic_multiplier = 1.0;
        } else {
            base_length = 2.0*WORKING_HEIGHT + MIN_HEIGHT + 2.0*MOTOR_LIMIT;
            prismatic_multiplier = 2.0;
        }
        
        double total_length = base_length + prismatic_multiplier * prismatic_lengths[i];
        Eigen::Vector3d new_joint = refined_joints[i] + direction * total_length;
        refined_joints.push_back(new_joint);
    }
    
    return refined_joints;
}

std::vector<Eigen::Vector3d> FabrikSolverBlock::extract_direction_vectors(
    const std::vector<Eigen::Vector3d>& joint_positions) {
    
    std::vector<Eigen::Vector3d> directions;
    directions.reserve(joint_positions.size() - 1);
    
    for (size_t i = 0; i < joint_positions.size() - 1; i++) {
        Eigen::Vector3d direction = (joint_positions[i + 1] - joint_positions[i]).normalized();
        directions.push_back(direction);
    }
    
    return directions;
}

std::vector<double> FabrikSolverBlock::calculate_prismatic_changes(
    const std::vector<double>& current_prismatic,
    const std::vector<double>& previous_prismatic) {
    
    std::vector<double> changes;
    changes.reserve(current_prismatic.size());
    
    for (size_t i = 0; i < current_prismatic.size(); i++) {
        double change = std::abs(current_prismatic[i] - previous_prismatic[i]);
        changes.push_back(change);
    }
    
    return changes;
}

bool FabrikSolverBlock::check_prismatic_convergence(
    const std::vector<double>& prismatic_changes,
    double tolerance) {
    
    for (double change : prismatic_changes) {
        if (change > tolerance) {
            return false;
        }
    }
    
    return true;
}

bool FabrikSolverBlock::validate_solve_inputs(const Eigen::Vector3d& target,
                                             int num_segments,
                                             double tolerance,
                                             int max_iterations) {
    if (num_segments <= 0 || num_segments > 1000) {
        return false;
    }
    if (tolerance <= 0.0 || tolerance > 1000.0) {
        return false;
    }
    if (max_iterations <= 0 || max_iterations > 10000) {
        return false;
    }
    if (!target.allFinite()) {
        return false;
    }
    return true;
}

bool FabrikSolverBlock::validate_refinement_inputs(const Eigen::Vector3d& target,
                                                  int num_segments,
                                                  double fabrik_tolerance,
                                                  int max_fabrik_iterations,
                                                  double prismatic_tolerance,
                                                  int max_refinement_iterations) {
    
    if (!validate_solve_inputs(target, num_segments, fabrik_tolerance, max_fabrik_iterations)) {
        return false;
    }
    if (prismatic_tolerance <= 0.0 || prismatic_tolerance > 100.0) {
        return false;
    }
    if (max_refinement_iterations <= 0 || max_refinement_iterations > 50) {
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
            double initial_distance = (current_joints.back() - target).norm();
            if (check_convergence(initial_distance, tolerance)) {
                return FabrikSolverResult(current_joints, iteration, initial_distance, true,
                                        calculation_time_ms, init_result);
            }
            
            FabrikIterationResult iter_result = FabrikIterationBlock::iterate(
                current_joints, target, init_result.joint_distances
            );
            
            if (!iter_result.iteration_successful) {
                return FabrikSolverResult(calculation_time_ms, 
                    "Iteration " + std::to_string(iteration + 1) + " failed: " + iter_result.error_message);
            }
            
            current_joints = iter_result.updated_joints;
            
            if (check_convergence(iter_result.distance_to_target, tolerance)) {
                return FabrikSolverResult(current_joints, iteration + 1, iter_result.distance_to_target, true,
                                        calculation_time_ms, init_result);
            }
        }
        
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
    double max_reach = std::accumulate(joint_distances.begin(), joint_distances.end(), 0.0);
    double target_distance = target.norm();
    double reach_buffer = 1.1;
    return target_distance <= (max_reach * reach_buffer);
}

} // namespace delta