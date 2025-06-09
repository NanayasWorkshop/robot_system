#include "kinematics_block.hpp"
#include <cmath>

namespace delta {

KinematicsResult KinematicsBlock::calculate(double x, double y, double z) {
    return calculate(Eigen::Vector3d(x, y, z));
}

KinematicsResult KinematicsBlock::calculate(const Eigen::Vector3d& direction_vector) {
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        // Input validation (same as FermatBlock)
        if (direction_vector.norm() < 1e-10) {
            throw std::invalid_argument("Direction vector cannot be zero");
        }
        if (direction_vector.z() <= 0) {
            throw std::invalid_argument("Direction vector Z component must be positive");
        }
        
        // Step 1: Get fermat data using FermatBlock (internal coupling)
        FermatResult fermat_result = FermatBlock::calculate(direction_vector);
        
        // Step 2: Calculate Point H (fixed joint position)
        Eigen::Vector3d point_H(0.0, 0.0, WORKING_HEIGHT);
        
        // Step 3: Calculate Point G along direction vector
        Eigen::Vector3d point_G = calculate_point_G(direction_vector, fermat_result.fermat_point);
        
        // Step 4: Calculate H→G length
        double HG_length = (point_G - point_H).norm();
        
        // Step 5: Calculate end-effector position using geometric mirroring
        Eigen::Vector3d end_effector = calculate_end_effector_position(point_H, point_G, direction_vector);
        
        return KinematicsResult(point_H, point_G, HG_length, end_effector, calculation_time_ms, fermat_result);
    }
}

Eigen::Vector3d KinematicsBlock::calculate_point_G(const Eigen::Vector3d& direction_vector, 
                                                  const Eigen::Vector3d& fermat_point) {
    // Calculate prismatic joint length from fermat point
    double prismatic_length = 2.0 * fermat_point.z();
    
    // Calculate total vector length from H to G
    double vector_length = MIN_HEIGHT + 2.0 * MOTOR_LIMIT + prismatic_length;
    
    // Point G is along normalized direction vector from Point H
    Eigen::Vector3d point_H(0.0, 0.0, WORKING_HEIGHT);
    Eigen::Vector3d normalized_direction = direction_vector.normalized();
    
    return point_H + normalized_direction * vector_length;
}

Eigen::Vector3d KinematicsBlock::calculate_end_effector_position(const Eigen::Vector3d& point_H, 
                                                               const Eigen::Vector3d& point_G, 
                                                               const Eigen::Vector3d& direction_vector) {
    // Calculate midpoint between H and G
    Eigen::Vector3d midpoint = (point_H + point_G) * 0.5;
    
    // Mirror base point (0,0,0) across plane through midpoint with normal = direction
    Eigen::Vector3d base_point(0.0, 0.0, 0.0);
    Eigen::Vector3d normalized_direction = direction_vector.normalized();
    
    // Mirror formula: P' = P - 2 * ((P - point_on_plane) · normal) * normal
    // Where: P = base_point, point_on_plane = midpoint, normal = normalized_direction
    Eigen::Vector3d to_plane = base_point - midpoint;
    double distance_to_plane = to_plane.dot(normalized_direction);
    Eigen::Vector3d end_effector = base_point - normalized_direction * (2.0 * distance_to_plane);
    
    return end_effector;
}

} // namespace delta