#ifndef DELTA_BLOCKS_SEGMENT_BLOCK_HPP
#define DELTA_BLOCKS_SEGMENT_BLOCK_HPP

#include <Eigen/Dense>
#include <optional>
#include <string>
#include <vector>
#include "../core/constants.hpp"
#include "../core/timing.hpp"
#include "fermat_block.hpp"
#include "kinematics_block.hpp"
#include "joint_state_block.hpp"
#include "orientation_block.hpp"

namespace delta {

struct SegmentResult {
    // Essential data (always calculated):
    double prismatic_length;              // Primary output for FABRIK
    double calculation_time_ms;            // Total time including called blocks
    bool calculation_successful;           // Error flag (no exceptions)
    std::string error_message;             // Brief error description if failed
    
    // NEW: S-point conversion data (populated when using joint-based methods)
    std::optional<Eigen::Vector3d> calculated_segment_position;     // The S-point we calculated
    std::optional<Eigen::Vector3d> calculated_direction;            // Direction we derived and used
    std::optional<Eigen::Vector3d> previous_segment_position;       // Previous S-point used for transformation
    
    // Detailed data (complete mode only):
    std::optional<KinematicsResult> kinematics_data;
    std::optional<JointStateResult> joint_state_data;
    std::optional<OrientationResult> orientation_data;
    
    SegmentResult(double prismatic, double time_ms, bool successful, const std::string& error = "")
        : prismatic_length(prismatic), calculation_time_ms(time_ms)
        , calculation_successful(successful), error_message(error) {}
};

class SegmentBlock {
public:
    // FABRIK-optimized path - only essential calculations (direction-based)
    static SegmentResult calculate_essential(
        const Eigen::Vector3d& current_direction,
        const Eigen::Vector3d& previous_direction = Eigen::Vector3d(0, 0, 1)
    );
    
    // Full analysis path - all block calculations (direction-based)
    static SegmentResult calculate_complete(
        const Eigen::Vector3d& current_direction,
        const Eigen::Vector3d& previous_direction = Eigen::Vector3d(0, 0, 1)
    );
    
    // NEW: FABRIK-optimized path with J→S conversion (joint-based)
    static SegmentResult calculate_essential_from_joints(
        const std::vector<Eigen::Vector3d>& joint_positions,
        int segment_index
    );
    
    // NEW: Full analysis path with J→S conversion (joint-based)
    static SegmentResult calculate_complete_from_joints(
        const std::vector<Eigen::Vector3d>& joint_positions,
        int segment_index
    );

private:
    // Coordinate transformation using Rodrigues rotation
    static Eigen::Vector3d apply_coordinate_transformation(
        const Eigen::Vector3d& current_direction,
        const Eigen::Vector3d& previous_direction
    );
    
    // Rodrigues rotation formula implementation
    static Eigen::Vector3d rodrigues_rotation(
        const Eigen::Vector3d& vector,
        const Eigen::Vector3d& axis,
        double angle
    );
    
    // Input validation after transformation
    static bool validate_transformed_direction(const Eigen::Vector3d& transformed_direction);
    
    // Check if transformation is needed (first segment case)
    static bool needs_transformation(const Eigen::Vector3d& previous_direction);
    
    // NEW: Core J→S conversion logic
    static Eigen::Vector3d convert_joint_to_segment_position(
        const std::vector<Eigen::Vector3d>& joint_positions,
        int segment_index
    );
    
    // NEW: Calculate direction vectors for joint-based methods
    static std::pair<Eigen::Vector3d, Eigen::Vector3d> calculate_directions_from_joints(
        const std::vector<Eigen::Vector3d>& joint_positions,
        int segment_index
    );
    
    // NEW: Validate joint positions input
    static bool validate_joint_positions(
        const std::vector<Eigen::Vector3d>& joint_positions,
        int segment_index
    );
};

} // namespace delta

#endif // DELTA_BLOCKS_SEGMENT_BLOCK_HPP