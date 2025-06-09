#include "joint_state_block.hpp"
#include <cmath>

namespace delta {

JointStateResult JointStateBlock::calculate(double x, double y, double z) {
    return calculate(Eigen::Vector3d(x, y, z));
}

JointStateResult JointStateBlock::calculate(const Eigen::Vector3d& direction_vector) {
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        // Input validation (same as FermatBlock and KinematicsBlock)
        if (direction_vector.norm() < 1e-10) {
            throw std::invalid_argument("Direction vector cannot be zero");
        }
        if (direction_vector.z() <= 0) {
            throw std::invalid_argument("Direction vector Z component must be positive");
        }
        
        // Step 1: Get fermat data using FermatBlock (guaranteed to match direction_vector)
        FermatResult fermat_result = FermatBlock::calculate(direction_vector);
        
        // Step 2: Calculate joint states
        double prismatic = calculate_prismatic_joint(fermat_result.fermat_point);
        double roll = calculate_roll_joint(direction_vector);
        double pitch = calculate_pitch_joint(direction_vector);
        
        return JointStateResult(prismatic, roll, pitch, calculation_time_ms, fermat_result);
    }
}

double JointStateBlock::calculate_prismatic_joint(const Eigen::Vector3d& fermat_point) {
    // Prismatic joint = 2 × Z value of Fermat point
    return 2.0 * fermat_point.z();
}

double JointStateBlock::calculate_roll_joint(const Eigen::Vector3d& direction_vector) {
    // Normalize the direction vector
    Eigen::Vector3d normalized = direction_vector.normalized();
    
    // Roll = -atan2(y, z) - rotation around X-axis
    return -std::atan2(normalized.y(), normalized.z());
}

double JointStateBlock::calculate_pitch_joint(const Eigen::Vector3d& direction_vector) {
    // Normalize the direction vector
    Eigen::Vector3d normalized = direction_vector.normalized();
    
    // Pitch = atan2(x, z) - rotation around Y-axis
    return std::atan2(normalized.x(), normalized.z());
}

} // namespace delta