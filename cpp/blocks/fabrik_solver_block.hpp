#ifndef DELTA_BLOCKS_FABRIK_SOLVER_BLOCK_HPP
#define DELTA_BLOCKS_FABRIK_SOLVER_BLOCK_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "../core/constants.hpp"
#include "../core/timing.hpp"
#include "fabrik_initialization_block.hpp"
#include "fabrik_iteration_block.hpp"

namespace delta {

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
    // Main interface: complete FABRIK solving
    static FabrikSolverResult solve(
        const Eigen::Vector3d& target_position,
        int num_segments = DEFAULT_ROBOT_SEGMENTS,
        double tolerance = FABRIK_TOLERANCE,
        int max_iterations = FABRIK_MAX_ITERATIONS
    );

private:
    // Input validation
    static bool validate_solve_inputs(const Eigen::Vector3d& target,
                                     int num_segments,
                                     double tolerance,
                                     int max_iterations);
    
    // Core solving loop
    static FabrikSolverResult solve_iterative(
        const Eigen::Vector3d& target,
        const FabrikInitResult& init_result,
        double tolerance,
        int max_iterations
    );
    
    // Convergence checking
    static bool check_convergence(double distance_to_target, double tolerance);
    
    // Target reachability estimation (optional validation)
    static bool is_target_potentially_reachable(const Eigen::Vector3d& target,
                                               const std::vector<double>& joint_distances);
};

} // namespace delta

#endif // DELTA_BLOCKS_FABRIK_SOLVER_BLOCK_HPP