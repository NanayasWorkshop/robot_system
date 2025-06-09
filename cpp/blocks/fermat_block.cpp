#include "fermat_block.hpp"
#include <cmath>

namespace delta {

FermatResult FermatBlock::calculate(double x, double y, double z) {
    return calculate(Eigen::Vector3d(x, y, z));
}

FermatResult FermatBlock::calculate(const Eigen::Vector3d& input_vector) {
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        // Input validation
        if (input_vector.norm() < 1e-10) {
            throw std::invalid_argument("Input vector cannot be zero");
        }
        if (input_vector.z() <= 0) {
            throw std::invalid_argument("Input vector Z component must be positive");
        }
        
        // Normalize the input direction
        Eigen::Vector3d normalized_direction = input_vector.normalized();
        
        // Calculate Fermat point
        Eigen::Vector3d fermat_point = calculate_fermat_point(normalized_direction);
        
        // Get base positions
        Eigen::Vector3d base_A = get_base_position_A();
        Eigen::Vector3d base_B = get_base_position_B();
        Eigen::Vector3d base_C = get_base_position_C();
        
        // Calculate Z positions for each actuator
        double z_A = calculate_z_position(base_A, fermat_point, normalized_direction);
        double z_B = calculate_z_position(base_B, fermat_point, normalized_direction);
        double z_C = calculate_z_position(base_C, fermat_point, normalized_direction);
        
        return FermatResult(z_A, z_B, z_C, fermat_point, calculation_time_ms);
    }
}

Eigen::Vector3d FermatBlock::get_base_position_A() {
    return Eigen::Vector3d(
        ROBOT_RADIUS * std::cos(BASE_A_ANGLE),
        ROBOT_RADIUS * std::sin(BASE_A_ANGLE),
        0.0
    );
}

Eigen::Vector3d FermatBlock::get_base_position_B() {
    return Eigen::Vector3d(
        ROBOT_RADIUS * std::cos(BASE_B_ANGLE),
        ROBOT_RADIUS * std::sin(BASE_B_ANGLE),
        0.0
    );
}

Eigen::Vector3d FermatBlock::get_base_position_C() {
    return Eigen::Vector3d(
        ROBOT_RADIUS * std::cos(BASE_C_ANGLE),
        ROBOT_RADIUS * std::sin(BASE_C_ANGLE),
        0.0
    );
}

Eigen::Vector3d FermatBlock::calculate_fermat_point(const Eigen::Vector3d& normalized_direction) {
    // Get base positions
    Eigen::Vector3d base_A = get_base_position_A();
    Eigen::Vector3d base_B = get_base_position_B();
    Eigen::Vector3d base_C = get_base_position_C();
    
    // Project base positions onto plane perpendicular to direction
    // Plane equation: direction · (point - origin) = 0
    // For simplicity, we'll use the origin as plane point
    
    Eigen::Vector3d proj_A = base_A - normalized_direction * normalized_direction.dot(base_A);
    Eigen::Vector3d proj_B = base_B - normalized_direction * normalized_direction.dot(base_B);
    Eigen::Vector3d proj_C = base_C - normalized_direction * normalized_direction.dot(base_C);
    
    // Calculate centroid of projected points (approximation of Fermat point)
    Eigen::Vector3d fermat_point = (proj_A + proj_B + proj_C) / 3.0;
    
    return fermat_point;
}

double FermatBlock::calculate_z_position(const Eigen::Vector3d& base_position, 
                                       const Eigen::Vector3d& fermat_point, 
                                       const Eigen::Vector3d& normalized_direction) {
    // Calculate vector from base to Fermat point
    Eigen::Vector3d base_to_fermat = fermat_point - base_position;
    
    // Project this vector onto the normalized direction to get Z offset
    double z_offset = base_to_fermat.dot(normalized_direction);
    
    // Z position is base Z (0) plus the offset
    return base_position.z() + z_offset;
}

} // namespace delta