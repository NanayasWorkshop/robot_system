#include "fermat_block.hpp"
#include <cmath>
#include <algorithm>

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
        
        // Calculate true Fermat point using proper mathematical algorithm
        FermatCalculationData calc_data = calculate_true_fermat_point(normalized_direction);
        
        return FermatResult(calc_data.z_A, calc_data.z_B, calc_data.z_C, 
                           calc_data.fermat_point, calculation_time_ms);
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

double FermatBlock::calculate_z_intersection(double base_x, double base_y, 
                                           const Eigen::Vector3d& normal) {
    // Plane equation: normal.x * x + normal.y * y + normal.z * z = 0 (plane through origin)
    // Point on vertical line: (base_x, base_y, z)
    // Solve for z: normal.x * base_x + normal.y * base_y + normal.z * z = 0
    // z = -(normal.x * base_x + normal.y * base_y) / normal.z
    
    if (std::abs(normal.z()) < 1e-10) {
        // Normal is horizontal, no intersection or infinite intersections
        return 0.0;
    }
    
    return -(normal.x() * base_x + normal.y() * base_y) / normal.z();
}

FermatBlock::FermatCalculationData FermatBlock::calculate_true_fermat_point(const Eigen::Vector3d& normalized_direction) {
    FermatCalculationData data;
    
    // Get base positions
    Eigen::Vector3d base_A = get_base_position_A();
    Eigen::Vector3d base_B = get_base_position_B();
    Eigen::Vector3d base_C = get_base_position_C();
    
    // Calculate Z intersections with plane defined by normalized direction
    data.z_A = calculate_z_intersection(base_A.x(), base_A.y(), normalized_direction);
    data.z_B = calculate_z_intersection(base_B.x(), base_B.y(), normalized_direction);
    data.z_C = calculate_z_intersection(base_C.x(), base_C.y(), normalized_direction);
    
    // Create 3D triangle points
    Eigen::Vector3d A_point(base_A.x(), base_A.y(), data.z_A);
    Eigen::Vector3d B_point(base_B.x(), base_B.y(), data.z_B);
    Eigen::Vector3d C_point(base_C.x(), base_C.y(), data.z_C);
    
    // Calculate triangle geometry
    Eigen::Vector3d AB = B_point - A_point;
    Eigen::Vector3d BC = C_point - B_point;
    Eigen::Vector3d CA = A_point - C_point;
    
    // Calculate side lengths
    double side_a = BC.norm();  // side opposite to A
    double side_b = CA.norm();  // side opposite to B
    double side_c = AB.norm();  // side opposite to C
    
    // Calculate angles using dot product
    // Angle at A: between vectors AB and AC (which is -CA)
    Eigen::Vector3d AC = -CA;
    double alpha = std::acos(std::max(-1.0, std::min(1.0, 
        AB.dot(AC) / (AB.norm() * AC.norm()))));
    
    // Angle at B: between vectors BA (which is -AB) and BC
    Eigen::Vector3d BA = -AB;
    double beta = std::acos(std::max(-1.0, std::min(1.0, 
        BA.dot(BC) / (BA.norm() * BC.norm()))));
    
    // Angle at C: between vectors CB (which is -BC) and CA
    Eigen::Vector3d CB = -BC;
    double gamma = std::acos(std::max(-1.0, std::min(1.0, 
        CB.dot(CA) / (CB.norm() * CA.norm()))));
    
    // Calculate Lambda values for true Fermat point
    // λ = side_length / sin(angle + 60°)
    const double sixty_degrees = M_PI / 3.0;
    const double epsilon = 1e-10;
    
    double sin_alpha_plus_60 = std::sin(alpha + sixty_degrees);
    double sin_beta_plus_60 = std::sin(beta + sixty_degrees);
    double sin_gamma_plus_60 = std::sin(gamma + sixty_degrees);
    
    double lambda_A = side_a / std::max(sin_alpha_plus_60, epsilon);
    double lambda_B = side_b / std::max(sin_beta_plus_60, epsilon);
    double lambda_C = side_c / std::max(sin_gamma_plus_60, epsilon);
    
    // Calculate Fermat point as weighted centroid
    double total_lambda = lambda_A + lambda_B + lambda_C;
    
    if (total_lambda < epsilon) {
        // Fallback to simple centroid if lambda calculation fails
        data.fermat_point = (A_point + B_point + C_point) / 3.0;
    } else {
        double fermat_x = (lambda_A * A_point.x() + lambda_B * B_point.x() + lambda_C * C_point.x()) / total_lambda;
        double fermat_y = (lambda_A * A_point.y() + lambda_B * B_point.y() + lambda_C * C_point.y()) / total_lambda;
        double fermat_z = (lambda_A * A_point.z() + lambda_B * B_point.z() + lambda_C * C_point.z()) / total_lambda;
        
        data.fermat_point = Eigen::Vector3d(fermat_x, fermat_y, fermat_z);
    }
    
    return data;
}

} // namespace delta