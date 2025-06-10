#ifndef DELTA_BLOCKS_FABRIK_INITIALIZATION_BLOCK_HPP
#define DELTA_BLOCKS_FABRIK_INITIALIZATION_BLOCK_HPP

#include <Eigen/Dense>
#include <vector>
#include <optional>
#include "../core/constants.hpp"
#include "../core/timing.hpp"

namespace delta {

struct FabrikInitializationResult {
    std::vector<Eigen::Vector3d> joint_positions;    // Initial joint position array
    bool validation_successful;                      // Whether configuration is geometrically valid
    double calculation_time_ms;                      // Time taken for calculation
    
    FabrikInitializationResult(const std::vector<Eigen::Vector3d>& positions, bool valid, double time_ms)
        : joint_positions(positions), validation_successful(valid), calculation_time_ms(time_ms) {}
};

class FabrikInitializationBlock {
public:
    // Main interface: create initial joint positions for FABRIK solving
    static FabrikInitializationResult calculate(int num_robot_segments,
                                               std::optional<std::vector<Eigen::Vector3d>> initial_joint_positions = std::nullopt);
    
    // Convenience method for straight-up initialization
    static FabrikInitializationResult calculate_straight_up(int num_robot_segments = DEFAULT_ROBOT_SEGMENTS);

private:
    // Create straight-up joint positions
    static std::vector<Eigen::Vector3d> create_straight_up_positions(int num_robot_segments);
    
    // Validate joint positions for geometric correctness
    static bool validate_joint_positions(const std::vector<Eigen::Vector3d>& joint_positions, int num_robot_segments);
    
    // Calculate segment length for straight-up configuration
    static double calculate_segment_length();
    
    // Calculate FABRIK segment lengths (different for first, middle, last)
    static std::vector<double> calculate_fabrik_segment_lengths(int num_robot_segments, double base_segment_length);
    
    // Calculate joint position for specific segment and joint index (legacy method)
    static Eigen::Vector3d calculate_joint_position(int segment_index, double segment_length);
};

} // namespace delta

#endif // DELTA_BLOCKS_FABRIK_INITIALIZATION_BLOCK_HPP