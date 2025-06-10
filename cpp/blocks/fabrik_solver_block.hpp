#ifndef DELTA_BLOCKS_FABRIK_SOLVER_BLOCK_HPP
#define DELTA_BLOCKS_FABRIK_SOLVER_BLOCK_HPP

#include <Eigen/Dense>
#include <vector>
#include <optional>
#include "../core/constants.hpp"
#include "../core/timing.hpp"
#include "fabrik_initialization_block.hpp"
#include "fabrik_backward_block.hpp"
#include "fabrik_forward_block.hpp"
#include "segment_block.hpp"

namespace delta {

struct FabrikSolverResult {
    std::vector<Eigen::Vector3d> final_joint_positions;  // Solved joint chain
    Eigen::Vector3d achieved_position;                   // Actual end-effector position  
    bool converged;                                      // Did we reach target within tolerance?
    double final_error;                                  // Distance from target to achieved
    int total_iterations;                                // How many backward+forward cycles
    double solve_time_ms;                                // Total solving time
    
    FabrikSolverResult(const std::vector<Eigen::Vector3d>& positions, 
                      const Eigen::Vector3d& achieved, bool conv, double error,
                      int iterations, double time_ms)
        : final_joint_positions(positions), achieved_position(achieved)
        , converged(conv), final_error(error), total_iterations(iterations)
        , solve_time_ms(time_ms) {}
};

class FabrikSolverBlock {
public:
    // Main solver interface with default parameters from constants.hpp
    static FabrikSolverResult solve(
        const Eigen::Vector3d& target_position,
        std::optional<std::vector<Eigen::Vector3d>> initial_joint_positions = std::nullopt,
        int num_robot_segments = DEFAULT_ROBOT_SEGMENTS,
        double tolerance = FABRIK_TOLERANCE,
        int max_iterations = FABRIK_MAX_ITERATIONS
    );

private:
    // Calculate initial segment lengths from joint positions
    static std::vector<double> calculate_initial_segment_lengths(
        const std::vector<Eigen::Vector3d>& joint_positions
    );
    
    // Calculate distance from end-effector to target
    static double calculate_distance_to_target(
        const std::vector<Eigen::Vector3d>& joint_positions,
        const Eigen::Vector3d& target_position
    );
    
    // Calculate updated segment lengths using SegmentBlock
    static std::vector<double> calculate_updated_segment_lengths(
        const std::vector<Eigen::Vector3d>& joint_positions
    );
    
    // Convert prismatic length to FABRIK segment length
    static double convert_prismatic_to_fabrik_length(
        double prismatic_length, 
        int segment_index, 
        int total_segments
    );
};

} // namespace delta

#endif // DELTA_BLOCKS_FABRIK_SOLVER_BLOCK_HPP