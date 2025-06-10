// fabrik_forward_block.hpp - Clean Version

#ifndef DELTA_BLOCKS_FABRIK_FORWARD_BLOCK_HPP
#define DELTA_BLOCKS_FABRIK_FORWARD_BLOCK_HPP

#include <Eigen/Dense>
#include <vector>
#include "../core/constants.hpp"
#include "../core/timing.hpp"
#include "cone_constraint_block.hpp"
#include "segment_block.hpp"

namespace delta {

struct FabrikForwardResult {
    std::vector<Eigen::Vector3d> updated_joint_positions;      // Joint positions with base fixed at origin
    std::vector<double> recalculated_segment_lengths;          // NEW segment lengths for next iteration
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
    // Single forward pass from base to end with dynamic segment recalculation
    static FabrikForwardResult calculate(const std::vector<Eigen::Vector3d>& joint_positions,
                                        const Eigen::Vector3d& target_position);

private:
    // Single forward pass implementation
    static std::vector<Eigen::Vector3d> single_forward_pass(const std::vector<Eigen::Vector3d>& joint_positions,
                                                           const Eigen::Vector3d& target_position,
                                                           std::vector<double>& recalculated_lengths);
    
    // Calculate new segment length using complete joint chain with SegmentBlock
    static double calculate_new_segment_length_from_complete_chain(const std::vector<Eigen::Vector3d>& complete_joint_chain,
                                                                  int segment_index, int total_segments);
    
    // Apply cone constraint for spherical joints
    static Eigen::Vector3d apply_cone_constraint_if_needed(const Eigen::Vector3d& desired_direction,
                                                          const std::vector<Eigen::Vector3d>& joint_positions,
                                                          int joint_index);
    
    // Calculate distance from end-effector to target
    static double calculate_distance_to_target(const std::vector<Eigen::Vector3d>& joint_positions,
                                              const Eigen::Vector3d& target_position);
    
    // Convert prismatic length to FABRIK segment length
    static double convert_prismatic_to_fabrik_length(double prismatic_length, 
                                                    int segment_index, int total_segments);
};

} // namespace delta

#endif // DELTA_BLOCKS_FABRIK_FORWARD_BLOCK_HPP