#ifndef DELTA_BLOCKS_CONE_CONSTRAINT_BLOCK_HPP
#define DELTA_BLOCKS_CONE_CONSTRAINT_BLOCK_HPP

#include <Eigen/Dense>
#include "../core/constants.hpp"
#include "../core/timing.hpp"

namespace delta {

struct ConeConstraintResult {
    Eigen::Vector3d projected_direction;    // Direction vector projected onto cone
    bool constraint_applied;                // Whether modification was needed
    double calculation_time_ms;             // Time taken for calculation
    
    ConeConstraintResult(const Eigen::Vector3d& projected, bool applied, double time_ms)
        : projected_direction(projected), constraint_applied(applied), calculation_time_ms(time_ms) {}
};

class ConeConstraintBlock {
public:
    // Main interface: project direction vector onto spherical joint cone constraints
    static ConeConstraintResult calculate(const Eigen::Vector3d& desired_direction,
                                         const Eigen::Vector3d& cone_apex,
                                         const Eigen::Vector3d& cone_axis,
                                         double cone_angle_rad);
    
    // Convenience method using default 120-degree spherical joint constraint
    static ConeConstraintResult calculate_spherical_120(const Eigen::Vector3d& desired_direction,
                                                        const Eigen::Vector3d& cone_apex,
                                                        const Eigen::Vector3d& cone_axis);

private:
    // Core projection algorithm (extracted from your constraint_utils.cpp)
    static Eigen::Vector3d project_direction_onto_cone_surface(const Eigen::Vector3d& desired_direction,
                                                              const Eigen::Vector3d& cone_axis_normalized,
                                                              double cone_half_angle_rad);
    
    // Input validation
    static bool validate_inputs(const Eigen::Vector3d& desired_direction,
                               const Eigen::Vector3d& cone_apex,
                               const Eigen::Vector3d& cone_axis,
                               double cone_angle_rad);
};

} // namespace delta

#endif // DELTA_BLOCKS_CONE_CONSTRAINT_BLOCK_HPP