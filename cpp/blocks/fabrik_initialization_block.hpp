#ifndef DELTA_BLOCKS_FABRIK_INITIALIZATION_BLOCK_HPP
#define DELTA_BLOCKS_FABRIK_INITIALIZATION_BLOCK_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "../core/constants.hpp"
#include "../core/timing.hpp"

namespace delta {

struct FabrikInitResult {
    std::vector<Eigen::Vector3d> initial_joints;     // Joint positions array
    std::vector<double> joint_distances;             // Distance between consecutive joints
    int num_segments;                                // Input validation echo
    int num_joints;                                  // Calculated joint count
    double calculation_time_ms;                      // Timing measurement
    bool initialization_successful;                  // Success flag
    std::string error_message;                       // Error description if failed
    
    // Default constructor
    FabrikInitResult() 
        : num_segments(0), num_joints(0), calculation_time_ms(0.0)
        , initialization_successful(false) {}
    
    FabrikInitResult(const std::vector<Eigen::Vector3d>& joints,
                    const std::vector<double>& distances,
                    int segments, int joints_count, double time_ms)
        : initial_joints(joints), joint_distances(distances)
        , num_segments(segments), num_joints(joints_count)
        , calculation_time_ms(time_ms), initialization_successful(true) {}
    
    // Error constructor
    FabrikInitResult(int segments, double time_ms, const std::string& error)
        : num_segments(segments), num_joints(0), calculation_time_ms(time_ms)
        , initialization_successful(false), error_message(error) {}
};

class FabrikInitializationBlock {
public:
    // Main interface: create straight-line joint chain
    static FabrikInitResult create_straight_chain(int num_segments);
    
    // Future expansion placeholder
    // static FabrikInitResult create_bent_chain(int num_segments, 
    //                                           const std::vector<double>& bend_angles);

private:
    // Input validation
    static bool validate_num_segments(int num_segments);
    
    // Joint position calculation helpers
    static Eigen::Vector3d calculate_first_joint_position();
    static Eigen::Vector3d calculate_middle_joint_position(const Eigen::Vector3d& previous_joint);
    static Eigen::Vector3d calculate_last_joint_position(const Eigen::Vector3d& previous_joint);
    
    // Distance calculation helper
    static std::vector<double> calculate_joint_distances(const std::vector<Eigen::Vector3d>& joints);
    
    // Constants for joint spacing calculations
    static constexpr double FIRST_SEGMENT_LENGTH = WORKING_HEIGHT + MIN_HEIGHT/2.0 + MOTOR_LIMIT;
    static constexpr double MIDDLE_SEGMENT_LENGTH = 2.0*WORKING_HEIGHT + MIN_HEIGHT + 2.0*MOTOR_LIMIT;
    static constexpr double LAST_SEGMENT_LENGTH = WORKING_HEIGHT + MIN_HEIGHT/2.0 + MOTOR_LIMIT;
};

} // namespace delta

#endif // DELTA_BLOCKS_FABRIK_INITIALIZATION_BLOCK_HPP