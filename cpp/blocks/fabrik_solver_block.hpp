#ifndef DELTA_BLOCKS_FABRIK_SOLVER_BLOCK_HPP
#define DELTA_BLOCKS_FABRIK_SOLVER_BLOCK_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "../core/constants.hpp"
#include "../core/timing.hpp"
#include "fabrik_initialization_block.hpp"
#include "fabrik_iteration_block.hpp"
#include "segment_block.hpp"

namespace delta {

struct FabrikRefinementResult {
    std::vector<Eigen::Vector3d> final_joints;        // Final joint positions
    std::vector<double> final_prismatic_lengths;      // Final prismatic values
    int fabrik_iterations_used;                       // FABRIK iterations in last solve
    int refinement_iterations_used;                   // Number of refinement loops
    double final_distance_to_target;                  // Remaining distance to target
    bool converged;                                   // True if within tolerance
    bool prismatic_converged;                         // True if prismatic changes < tolerance
    double calculation_time_ms;                       // Total solving time
    bool solving_successful;                          // Success flag
    std::string error_message;                        // Error description if failed
    
    // Reference data:
    FabrikInitResult initialization_data;             // Initial setup data
    std::vector<Eigen::Vector3d> initial_joints;      // Starting positions
    std::vector<double> initial_prismatic_lengths;    // Starting prismatic values
    
    FabrikRefinementResult(const std::vector<Eigen::Vector3d>& joints,
                          const std::vector<double>& prismatic,
                          int fabrik_iters, int refinement_iters,
                          double target_distance, bool has_converged, bool prismatic_conv,
                          double time_ms, const FabrikInitResult& init_data,
                          const std::vector<double>& init_prismatic)
        : final_joints(joints), final_prismatic_lengths(prismatic)
        , fabrik_iterations_used(fabrik_iters), refinement_iterations_used(refinement_iters)
        , final_distance_to_target(target_distance), converged(has_converged)
        , prismatic_converged(prismatic_conv), calculation_time_ms(time_ms)
        , solving_successful(true), initialization_data(init_data)
        , initial_joints(init_data.initial_joints), initial_prismatic_lengths(init_prismatic) {}
    
    // Error constructor
    FabrikRefinementResult(double time_ms, const std::string& error)
        : fabrik_iterations_used(0), refinement_iterations_used(0)
        , final_distance_to_target(0.0), converged(false), prismatic_converged(false)
        , calculation_time_ms(time_ms), solving_successful(false), error_message(error) {}
};

struct FabrikSolverResult {
    std::vector<Eigen::Vector3d> final_joints;        // Final joint positions
    int iterations_used;                              // Number of iterations performed
    double final_distance_to_target;                  // Remaining distance to target
    bool converged;                                   // True if within tolerance
    double calculation_time_ms;                       // Total solving time
    bool solving_successful;                          // Success flag
    std::string error_message;                        // Error description if failed
    
    // Reference data (for debugging/analysis):
    FabrikInitResult initialization_data;             // Initial setup data
    std::vector<Eigen::Vector3d> initial_joints;      // Starting positions
    
    FabrikSolverResult(const std::vector<Eigen::Vector3d>& joints,
                      int iterations, double target_distance, bool has_converged,
                      double time_ms, const FabrikInitResult& init_data)
        : final_joints(joints), iterations_used(iterations)
        , final_distance_to_target(target_distance), converged(has_converged)
        , calculation_time_ms(time_ms), solving_successful(true)
        , initialization_data(init_data), initial_joints(init_data.initial_joints) {}
    
    // Error constructor
    FabrikSolverResult(double time_ms, const std::string& error)
        : iterations_used(0), final_distance_to_target(0.0), converged(false)
        , calculation_time_ms(time_ms), solving_successful(false), error_message(error) {}
};

class FabrikSolverBlock {
public:
    // Original interface: basic FABRIK solving
    static FabrikSolverResult solve(
        const Eigen::Vector3d& target_position,
        int num_segments = DEFAULT_ROBOT_SEGMENTS,
        double tolerance = FABRIK_TOLERANCE,
        int max_iterations = FABRIK_MAX_ITERATIONS
    );
    
    // NEW: Enhanced interface with prismatic refinement
    static FabrikRefinementResult solve_with_prismatic_refinement(
        const Eigen::Vector3d& target_position,
        int num_segments = DEFAULT_ROBOT_SEGMENTS,
        double fabrik_tolerance = FABRIK_TOLERANCE,
        int max_fabrik_iterations = FABRIK_MAX_ITERATIONS,
        double prismatic_tolerance = FABRIK_PRISMATIC_TOLERANCE,
        int max_refinement_iterations = FABRIK_MAX_REFINEMENT_ITERATIONS
    );

private:
    // Input validation
    static bool validate_solve_inputs(const Eigen::Vector3d& target,
                                     int num_segments,
                                     double tolerance,
                                     int max_iterations);
    
    // Input validation for refinement
    static bool validate_refinement_inputs(const Eigen::Vector3d& target,
                                          int num_segments,
                                          double fabrik_tolerance,
                                          int max_fabrik_iterations,
                                          double prismatic_tolerance,
                                          int max_refinement_iterations);
    
    // Core solving loop
    static FabrikSolverResult solve_iterative(
        const Eigen::Vector3d& target,
        const FabrikInitResult& init_result,
        double tolerance,
        int max_iterations
    );
    
    // NEW: Core refinement loop
    static FabrikRefinementResult solve_refinement_iterative(
        const Eigen::Vector3d& target,
        const FabrikInitResult& init_result,
        double fabrik_tolerance,
        int max_fabrik_iterations,
        double prismatic_tolerance,
        int max_refinement_iterations
    );
    
    // NEW: Build refined initial chain with prismatic adjustments
    static std::vector<Eigen::Vector3d> build_refined_chain(
        const std::vector<Eigen::Vector3d>& direction_chain,
        const std::vector<double>& prismatic_lengths,
        int num_segments
    );
    
    // NEW: Extract direction vectors from joint chain
    static std::vector<Eigen::Vector3d> extract_direction_vectors(
        const std::vector<Eigen::Vector3d>& joint_positions
    );
    
    // NEW: Calculate prismatic length changes
    static std::vector<double> calculate_prismatic_changes(
        const std::vector<double>& current_prismatic,
        const std::vector<double>& previous_prismatic
    );
    
    // NEW: Check prismatic convergence
    static bool check_prismatic_convergence(
        const std::vector<double>& prismatic_changes,
        double tolerance
    );
    
    // Convergence checking
    static bool check_convergence(double distance_to_target, double tolerance);
    
    // Target reachability estimation (optional validation)
    static bool is_target_potentially_reachable(const Eigen::Vector3d& target,
                                               const std::vector<double>& joint_distances);
};

} // namespace delta

#endif // DELTA_BLOCKS_FABRIK_SOLVER_BLOCK_HPP