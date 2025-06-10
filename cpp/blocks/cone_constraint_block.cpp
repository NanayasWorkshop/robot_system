#include "cone_constraint_block.hpp"
#include <cmath>
#include <algorithm>

namespace delta {

ConeConstraintResult ConeConstraintBlock::calculate(const Eigen::Vector3d& desired_direction,
                                                   const Eigen::Vector3d& cone_apex,
                                                   const Eigen::Vector3d& cone_axis,
                                                   double cone_angle_rad) {
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        // Input validation
        if (!validate_inputs(desired_direction, cone_apex, cone_axis, cone_angle_rad)) {
            // Return input unchanged if validation fails
            return ConeConstraintResult(desired_direction, false, calculation_time_ms);
        }
        
        // Check for zero or near-zero direction
        if (desired_direction.norm() < 1e-10) {
            return ConeConstraintResult(desired_direction, false, calculation_time_ms);
        }
        
        // Calculate cone half-angle
        double cone_half_angle_rad = cone_angle_rad / 2.0;
        
        // Normalize cone axis
        Eigen::Vector3d cone_axis_normalized = cone_axis.normalized();
        
        // Project direction onto cone constraints
        Eigen::Vector3d projected = project_direction_onto_cone_surface(
            desired_direction, cone_axis_normalized, cone_half_angle_rad);
        
        // Check if constraint was applied
        bool constraint_applied = (projected - desired_direction).norm() > 1e-10;
        
        return ConeConstraintResult(projected, constraint_applied, calculation_time_ms);
    }
}

ConeConstraintResult ConeConstraintBlock::calculate_spherical_120(const Eigen::Vector3d& desired_direction,
                                                                 const Eigen::Vector3d& cone_apex,
                                                                 const Eigen::Vector3d& cone_axis) {
    return calculate(desired_direction, cone_apex, cone_axis, SPHERICAL_JOINT_CONE_ANGLE_RAD);
}

Eigen::Vector3d ConeConstraintBlock::project_direction_onto_cone_surface(const Eigen::Vector3d& desired_direction,
                                                                        const Eigen::Vector3d& cone_axis_normalized,
                                                                        double cone_half_angle_rad) {
    // Normalize desired direction
    Eigen::Vector3d dir_normalized = desired_direction.normalized();
    
    // Calculate angle between desired direction and cone axis
    double dot_product = std::max(-1.0, std::min(1.0, dir_normalized.dot(cone_axis_normalized)));
    double angle_to_axis = std::acos(dot_product);
    
    // If direction is within cone, use it directly
    if (angle_to_axis <= cone_half_angle_rad) {
        return desired_direction;
    }
    
    // Project direction onto cone surface
    // Find component along cone axis
    Eigen::Vector3d along_axis = cone_axis_normalized * dot_product;
    
    // Find perpendicular component
    Eigen::Vector3d perpendicular = dir_normalized - along_axis;
    
    if (perpendicular.norm() < 1e-10) {
        // Direction is exactly along cone axis - use it
        return desired_direction;
    }
    
    // Create new direction on cone surface
    Eigen::Vector3d perp_normalized = perpendicular.normalized();
    double cos_half_angle = std::cos(cone_half_angle_rad);
    double sin_half_angle = std::sin(cone_half_angle_rad);
    
    // New direction on cone surface closest to desired direction
    Eigen::Vector3d projected_normalized = cone_axis_normalized * cos_half_angle + perp_normalized * sin_half_angle;
    
    // Scale back to original magnitude
    return projected_normalized * desired_direction.norm();
}

bool ConeConstraintBlock::validate_inputs(const Eigen::Vector3d& desired_direction,
                                         const Eigen::Vector3d& cone_apex,
                                         const Eigen::Vector3d& cone_axis,
                                         double cone_angle_rad) {
    // Check cone angle is positive and reasonable
    if (cone_angle_rad <= 0.0 || cone_angle_rad > M_PI) {
        return false;
    }
    
    // Check cone axis is not zero
    if (cone_axis.norm() < 1e-10) {
        return false;
    }
    
    // All other inputs are acceptable (zero direction, any apex position)
    return true;
}

} // namespace delta