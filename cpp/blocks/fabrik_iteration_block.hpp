#ifndef DELTA_BLOCKS_FABRIK_ITERATION_BLOCK_HPP
#define DELTA_BLOCKS_FABRIK_ITERATION_BLOCK_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "../core/constants.hpp"
#include "../core/timing.hpp"

namespace delta {

struct FabrikIterationResult {
    std::vector<Eigen::Vector3d> updated_joints;          // Joint positions after iteration
    double distance_to_target;                            // Remaining distance to target
    double calculation_time_ms;                           // Timing measurement
    bool iteration_successful;                            // Success flag
    std::string error_message;                            // Error description if failed
    
    FabrikIterationResult(const std::vector<Eigen::Vector3d>& joints,
                         double target_distance, double time_ms)
        : updated_joints(joints), distance_to_target(target_distance)
        , calculation_time_ms(time_ms), iteration_successful(true) {}
    
    // Error constructor
    FabrikIterationResult(double time_ms, const std::string& error)
        : distance_to_target(0.0), calculation_time_ms(time_ms)
        , iteration_successful(false), error_message(error) {}
};

class FabrikIterationBlock {
public:
    // Main interface: perform one complete FABRIK iteration
    static FabrikIterationResult iterate(
        const std::vector<Eigen::Vector3d>& current_joints,
        const Eigen::Vector3d& target_position,
        const std::vector<double>& joint_distances
    );

private:
    // Input validation
    static bool validate_inputs(const std::vector<Eigen::Vector3d>& joints,
                               const std::vector<double>& distances);
    
    // Backward pass: end-effector to base
    static std::vector<Eigen::Vector3d> backward_pass(
        const std::vector<Eigen::Vector3d>& joints,
        const Eigen::Vector3d& target,
        const std::vector<double>& distances,
        double cone_half_angle_rad
    );
    
    // Forward pass: base to end-effector
    static std::vector<Eigen::Vector3d> forward_pass(
        const std::vector<Eigen::Vector3d>& backward_joints,
        const std::vector<Eigen::Vector3d>& original_joints,
        const std::vector<double>& distances,
        double cone_half_angle_rad
    );
    
    // Vector utilities (matching Python implementation)
    static Eigen::Vector3d vector_subtract(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
    static double vector_magnitude(const Eigen::Vector3d& v);
    static double vector_distance(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
    static Eigen::Vector3d normalize_vector(const Eigen::Vector3d& v);
    static double vector_dot(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
    
    // Cone constraint utilities
    static double angle_between_vectors(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);
    static bool is_inside_cone(const Eigen::Vector3d& direction, 
                              const Eigen::Vector3d& cone_axis, 
                              double cone_half_angle_rad);
    static Eigen::Vector3d project_onto_cone(const Eigen::Vector3d& direction,
                                            const Eigen::Vector3d& cone_axis,
                                            double cone_half_angle_rad);
};

} // namespace delta

#endif // DELTA_BLOCKS_FABRIK_ITERATION_BLOCK_HPP