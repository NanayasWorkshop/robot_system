#ifndef DELTA_BLOCKS_FABRIK_BACKWARD_BLOCK_HPP
#define DELTA_BLOCKS_FABRIK_BACKWARD_BLOCK_HPP

#include <Eigen/Dense>
#include <vector>
#include "../core/constants.hpp"
#include "../core/timing.hpp"
#include "cone_constraint_block.hpp"

namespace delta {

struct FabrikBackwardResult {
    std::vector<Eigen::Vector3d> updated_joint_positions;    // Joint positions after backward pass
    double distance_to_base;                                 // Final distance from end-effector to base
    double calculation_time_ms;                              // Time taken for calculation
    
    FabrikBackwardResult(const std::vector<Eigen::Vector3d>& positions, double dist_to_base, double time_ms)
        : updated_joint_positions(positions), distance_to_base(dist_to_base), calculation_time_ms(time_ms) {}
};

class FabrikBackwardBlock {
public:
    // Main interface: single backward pass from target to base with cone constraints
    static FabrikBackwardResult calculate(const std::vector<Eigen::Vector3d>& joint_positions,
                                         const Eigen::Vector3d& target_position,
                                         const std::vector<double>& segment_lengths);

private:
    // Single backward pass (extracted from your old single_backward_iteration)
    static std::vector<Eigen::Vector3d> single_backward_pass(const std::vector<Eigen::Vector3d>& original_positions,
                                                            const Eigen::Vector3d& target_position,
                                                            const std::vector<double>& segment_lengths);
    
    // Apply cone constraint for spherical joints
    static Eigen::Vector3d apply_cone_constraint_if_needed(const Eigen::Vector3d& desired_direction,
                                                          const std::vector<Eigen::Vector3d>& joint_positions,
                                                          int joint_index);
    
    // Calculate distance from end-effector to base
    static double calculate_distance_to_base(const std::vector<Eigen::Vector3d>& joint_positions);
};

} // namespace delta

#endif // DELTA_BLOCKS_FABRIK_BACKWARD_BLOCK_HPP