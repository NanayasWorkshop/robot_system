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
    // Main interface: single forward pass from base to end with dynamic segment recalculation
    static FabrikForwardResult calculate(const std::vector<Eigen::Vector3d>& joint_positions,
                                        const Eigen::Vector3d& target_position);

private:
    // Single forward pass with dynamic segment length recalculation
    static std::vector<Eigen::Vector3d> single_forward_pass(const std::vector<Eigen::Vector3d>& original_positions,
                                                           const Eigen::Vector3d& target_position,
                                                           std::vector<double>& recalculated_lengths);
    
    // Extract direction pairs for segment calculation
    static std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> extract_direction_pairs(
        const std::vector<Eigen::Vector3d>& joint_positions, int up_to_joint);
    
    // Calculate new segment length using SegmentBlock
    static double calculate_new_segment_length(const Eigen::Vector3d& current_direction,
                                              const Eigen::Vector3d& previous_direction,
                                              int segment_index, int total_segments);
    
    // Apply cone constraint for spherical joints (forward version)
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