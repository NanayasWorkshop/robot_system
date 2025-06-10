// fabrik_forward_block.hpp - Optimized Version

#ifndef DELTA_BLOCKS_FABRIK_FORWARD_BLOCK_HPP
#define DELTA_BLOCKS_FABRIK_FORWARD_BLOCK_HPP

#include <Eigen/Dense>
#include <vector>
#include "../core/constants.hpp"
#include "../core/timing.hpp"
#include "cone_constraint_block.hpp"

namespace delta {

struct FabrikForwardResult {
    std::vector<Eigen::Vector3d> updated_joint_positions;      // Joint positions with base fixed at origin
    std::vector<double> recalculated_segment_lengths;          // Same as input (no recalculation)
    double distance_to_target;                                 // How far end-effector drifted from target
    double calculation_time_ms;                                // Time taken for calculation
    
    FabrikForwardResult(const std::vector<Eigen::Vector3d>& positions, 
                       const std::vector<double>& lengths,
                       double dist_to_target, double time_ms)
        : updated_joint_positions(positions), recalculated_segment_lengths(lengths)
        , distance_to_target(dist_to_target), calculation_time_ms(time_ms) {}
};

class FabrikForwardBlock {
public:
    // Single forward pass from base to end using fixed segment lengths
    static FabrikForwardResult calculate(const std::vector<Eigen::Vector3d>& joint_positions,
                                        const Eigen::Vector3d& target_position,
                                        const std::vector<double>& segment_lengths);

private:
    // Single forward pass implementation
    static std::vector<Eigen::Vector3d> single_forward_pass(const std::vector<Eigen::Vector3d>& joint_positions,
                                                           const Eigen::Vector3d& target_position,
                                                           const std::vector<double>& segment_lengths);
    
    // Apply cone constraint for spherical joints
    static Eigen::Vector3d apply_cone_constraint_if_needed(const Eigen::Vector3d& desired_direction,
                                                          const std::vector<Eigen::Vector3d>& joint_positions,
                                                          int joint_index);
    
    // Calculate distance from end-effector to target
    static double calculate_distance_to_target(const std::vector<Eigen::Vector3d>& joint_positions,
                                              const Eigen::Vector3d& target_position);
};

} // namespace delta

#endif // DELTA_BLOCKS_FABRIK_FORWARD_BLOCK_HPP