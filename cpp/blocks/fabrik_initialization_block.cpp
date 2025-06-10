#include "fabrik_initialization_block.hpp"
#include <cmath>

namespace delta {

FabrikInitializationResult FabrikInitializationBlock::calculate(int num_robot_segments,
                                                               std::optional<std::vector<Eigen::Vector3d>> initial_joint_positions) {
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        // Input validation
        if (num_robot_segments <= 0) {
            throw std::invalid_argument("Number of robot segments must be positive");
        }
        if (num_robot_segments > 20) {  // Reasonable upper limit
            throw std::invalid_argument("Number of robot segments too large (max 20)");
        }
        
        std::vector<Eigen::Vector3d> joint_positions;
        bool validation_successful = false;
        
        if (initial_joint_positions.has_value()) {
            // Use provided positions
            joint_positions = initial_joint_positions.value();
            validation_successful = validate_joint_positions(joint_positions, num_robot_segments);
        } else {
            // Create straight-up configuration
            joint_positions = create_straight_up_positions(num_robot_segments);
            validation_successful = true;  // Always valid for straight-up
        }
        
        return FabrikInitializationResult(joint_positions, validation_successful, calculation_time_ms);
    }
}

FabrikInitializationResult FabrikInitializationBlock::calculate_straight_up(int num_robot_segments) {
    return calculate(num_robot_segments, std::nullopt);
}

std::vector<Eigen::Vector3d> FabrikInitializationBlock::create_straight_up_positions(int num_robot_segments) {
    std::vector<Eigen::Vector3d> positions;
    
    // Calculate base segment length for straight-up configuration
    double base_segment_length = calculate_segment_length();
    
    // Expected number of joints: num_robot_segments + 2
    // J0 (base) + J1,J2,...,JN (segment joints) + JN+1 (end-effector)
    int total_joints = num_robot_segments + 2;
    positions.reserve(total_joints);
    
    // J0: Base joint at origin
    positions.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    
    // Calculate FABRIK segment lengths (following your old pattern)
    std::vector<double> fabrik_segment_lengths = calculate_fabrik_segment_lengths(num_robot_segments, base_segment_length);
    
    // Build joints by accumulating FABRIK segment lengths
    double cumulative_z = 0.0;
    
    for (int i = 1; i <= num_robot_segments + 1; i++) {
        cumulative_z += fabrik_segment_lengths[i - 1];  // Add length of segment [i-1] → [i]
        positions.push_back(Eigen::Vector3d(0.0, 0.0, cumulative_z));
    }
    
    return positions;
}

bool FabrikInitializationBlock::validate_joint_positions(const std::vector<Eigen::Vector3d>& joint_positions, 
                                                        int num_robot_segments) {
    // Expected number of joints
    int expected_joints = num_robot_segments + 2;
    if (static_cast<int>(joint_positions.size()) != expected_joints) {
        return false;
    }
    
    // Check that base is at origin
    if (joint_positions[0].norm() > 1e-6) {
        return false;
    }
    
    // Check minimum spacing between consecutive joints
    for (size_t i = 1; i < joint_positions.size(); i++) {
        double distance = (joint_positions[i] - joint_positions[i-1]).norm();
        if (distance < 1.0) {  // Minimum 1mm spacing
            return false;
        }
        if (distance > 2000.0) {  // Maximum 2m spacing (reasonable limit)
            return false;
        }
    }
    
    return true;
}

double FabrikInitializationBlock::calculate_segment_length() {
    // From your existing kinematics: segment length for straight-up configuration
    // Based on MIN_HEIGHT + 2*MOTOR_LIMIT (with zero prismatic length)
    double hypotenuse_distance = MIN_HEIGHT + 2.0 * MOTOR_LIMIT;
    
    // For straight up (angle = 0), segment length = hypotenuse / 2
    return hypotenuse_distance / 2.0;
}

std::vector<double> FabrikInitializationBlock::calculate_fabrik_segment_lengths(int num_robot_segments, double base_segment_length) {
    std::vector<double> fabrik_lengths;
    
    // For straight-up initialization: H→G distance = MIN_HEIGHT + 2*MOTOR_LIMIT (no prismatic extension)
    double h_to_g_distance = MIN_HEIGHT + 2.0 * MOTOR_LIMIT;
    
    // First FABRIK segment: [0] → [1] 
    double first_length = WORKING_HEIGHT + h_to_g_distance / 2.0;
    fabrik_lengths.push_back(first_length);
    
    // Middle FABRIK segments: [1] → [2], [2] → [3], etc.
    for (int i = 1; i < num_robot_segments; i++) {
        double middle_length = h_to_g_distance / 2.0 + 2.0 * WORKING_HEIGHT + h_to_g_distance / 2.0;
        fabrik_lengths.push_back(middle_length);
    }
    
    // Last FABRIK segment: [N] → [N+1]
    double last_length = h_to_g_distance / 2.0 + WORKING_HEIGHT;
    fabrik_lengths.push_back(last_length);
    
    return fabrik_lengths;
}

Eigen::Vector3d FabrikInitializationBlock::calculate_joint_position(int segment_index, double segment_length) {
    // Calculate Z position for segment_index (0-based)
    // Each segment contributes: WORKING_HEIGHT + 2*segment_length + WORKING_HEIGHT
    
    double z_position = 0.0;
    
    // Stack segments vertically
    for (int i = 0; i <= segment_index; i++) {
        if (i == 0) {
            // First segment: from base
            z_position = WORKING_HEIGHT + 2.0 * segment_length + WORKING_HEIGHT;
        } else {
            // Additional segments
            z_position += WORKING_HEIGHT + 2.0 * segment_length + WORKING_HEIGHT;
        }
    }
    
    return Eigen::Vector3d(0.0, 0.0, z_position);
}

} // namespace delta