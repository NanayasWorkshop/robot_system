#include "segment_block.hpp"
#include <cmath>

namespace delta {

SegmentResult SegmentBlock::calculate_essential(const Eigen::Vector3d& current_direction,
                                               const Eigen::Vector3d& previous_direction) {
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        try {
            // Step 1: Apply coordinate transformation if needed
            Eigen::Vector3d transformed_direction;
            if (needs_transformation(previous_direction)) {
                transformed_direction = apply_coordinate_transformation(current_direction, previous_direction);
            } else {
                // First segment - use current_direction directly
                transformed_direction = current_direction;
            }
            
            // Step 2: Validate transformed direction
            if (!validate_transformed_direction(transformed_direction)) {
                return SegmentResult(0.0, calculation_time_ms, false, 
                    "Transformed direction has negative or zero Z component");
            }
            
            // Step 3: Calculate prismatic length using FermatBlock
            FermatResult fermat_result = FermatBlock::calculate(transformed_direction);
            double prismatic_length = 2.0 * fermat_result.fermat_point.z();
            
            return SegmentResult(prismatic_length, calculation_time_ms, true);
            
        } catch (const std::exception& e) {
            return SegmentResult(0.0, calculation_time_ms, false, 
                "Calculation failed: " + std::string(e.what()));
        }
    }
}

SegmentResult SegmentBlock::calculate_complete(const Eigen::Vector3d& current_direction,
                                              const Eigen::Vector3d& previous_direction) {
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        try {
            // Step 1: Apply coordinate transformation if needed
            Eigen::Vector3d transformed_direction;
            if (needs_transformation(previous_direction)) {
                transformed_direction = apply_coordinate_transformation(current_direction, previous_direction);
            } else {
                // First segment - use current_direction directly
                transformed_direction = current_direction;
            }
            
            // Step 2: Validate transformed direction
            if (!validate_transformed_direction(transformed_direction)) {
                return SegmentResult(0.0, calculation_time_ms, false, 
                    "Transformed direction has negative or zero Z component");
            }
            
            // Step 3: Calculate all block results
            KinematicsResult kinematics_result = KinematicsBlock::calculate(transformed_direction);
            JointStateResult joint_state_result = JointStateBlock::calculate(transformed_direction);
            OrientationResult orientation_result = OrientationBlock::calculate(kinematics_result);
            
            // Step 4: Extract prismatic length
            double prismatic_length = joint_state_result.prismatic_joint;
            
            // Step 5: Create complete result
            SegmentResult result(prismatic_length, calculation_time_ms, true);
            result.kinematics_data = kinematics_result;
            result.joint_state_data = joint_state_result;
            result.orientation_data = orientation_result;
            
            return result;
            
        } catch (const std::exception& e) {
            return SegmentResult(0.0, calculation_time_ms, false, 
                "Calculation failed: " + std::string(e.what()));
        }
    }
}

SegmentResult SegmentBlock::calculate_essential_from_joints(const std::vector<Eigen::Vector3d>& joint_positions,
                                                           int segment_index) {
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        try {
            // Validate input
            if (!validate_joint_positions(joint_positions, segment_index)) {
                return SegmentResult(0.0, calculation_time_ms, false, 
                    "Invalid joint positions or segment index");
            }
            
            // Handle S0 (base) special case
            if (segment_index == 0) {
                Eigen::Vector3d base_position(0.0, 0.0, 0.0);
                SegmentResult result(0.0, calculation_time_ms, true);
                result.calculated_segment_position = base_position;
                result.calculated_direction = Eigen::Vector3d(0.0, 0.0, 1.0);
                return result;
            }
            
            // Calculate S-point for segments 1-7
            Eigen::Vector3d current_segment_position = convert_joint_to_segment_position(joint_positions, segment_index);
            auto [current_direction, previous_direction] = calculate_directions_from_joints(joint_positions, segment_index);
            
            // Calculate prismatic
            SegmentResult result = calculate_essential(current_direction, previous_direction);
            
            if (result.calculation_successful) {
                result.calculated_segment_position = current_segment_position;
                result.calculated_direction = current_direction;
            }
            
            return result;
            
        } catch (const std::exception& e) {
            return SegmentResult(0.0, calculation_time_ms, false, 
                "Joint calculation failed: " + std::string(e.what()));
        }
    }
}

SegmentResult SegmentBlock::calculate_complete_from_joints(const std::vector<Eigen::Vector3d>& joint_positions,
                                                          int segment_index) {
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        try {
            // Validate input
            if (!validate_joint_positions(joint_positions, segment_index)) {
                return SegmentResult(0.0, calculation_time_ms, false, 
                    "Invalid joint positions or segment index");
            }
            
            // Handle S0 (base) special case
            if (segment_index == 0) {
                Eigen::Vector3d base_position(0.0, 0.0, 0.0);
                SegmentResult result(0.0, calculation_time_ms, true);
                result.calculated_segment_position = base_position;
                result.calculated_direction = Eigen::Vector3d(0.0, 0.0, 1.0);
                return result;
            }
            
            // Calculate S-point for segments 1-7
            Eigen::Vector3d current_segment_position = convert_joint_to_segment_position(joint_positions, segment_index);
            auto [current_direction, previous_direction] = calculate_directions_from_joints(joint_positions, segment_index);
            
            // Calculate complete
            SegmentResult result = calculate_complete(current_direction, previous_direction);
            
            if (result.calculation_successful) {
                result.calculated_segment_position = current_segment_position;
                result.calculated_direction = current_direction;
            }
            
            return result;
            
        } catch (const std::exception& e) {
            return SegmentResult(0.0, calculation_time_ms, false, 
                "Joint calculation failed: " + std::string(e.what()));
        }
    }
}

Eigen::Vector3d SegmentBlock::apply_coordinate_transformation(const Eigen::Vector3d& current_direction,
                                                             const Eigen::Vector3d& previous_direction) {
    // Normalize directions
    Eigen::Vector3d prev_normalized = previous_direction.normalized();
    Eigen::Vector3d z_axis(0, 0, 1);
    
    // If previous direction is already Z+, no transformation needed
    if ((prev_normalized - z_axis).norm() < 1e-6) {
        return current_direction;
    }
    
    // If previous direction is -Z, simple flip
    if ((prev_normalized + z_axis).norm() < 1e-6) {
        return -current_direction;
    }
    
    // Calculate rotation to align previous_direction with Z+
    Eigen::Vector3d rotation_axis = prev_normalized.cross(z_axis);
    double rotation_axis_length = rotation_axis.norm();
    
    if (rotation_axis_length < 1e-6) {
        return current_direction;
    }
    
    rotation_axis = rotation_axis.normalized();
    double cos_angle = prev_normalized.dot(z_axis);
    double angle = std::acos(std::max(-1.0, std::min(1.0, cos_angle)));
    
    return rodrigues_rotation(current_direction, rotation_axis, angle);
}

Eigen::Vector3d SegmentBlock::rodrigues_rotation(const Eigen::Vector3d& vector,
                                                const Eigen::Vector3d& axis,
                                                double angle) {
    double cos_angle = std::cos(angle);
    double sin_angle = std::sin(angle);
    
    Eigen::Vector3d k_cross_v = axis.cross(vector);
    double k_dot_v = axis.dot(vector);
    
    return vector * cos_angle + k_cross_v * sin_angle + axis * k_dot_v * (1.0 - cos_angle);
}

bool SegmentBlock::validate_transformed_direction(const Eigen::Vector3d& transformed_direction) {
    if (transformed_direction.norm() < 1e-10) {
        return false;
    }
    if (transformed_direction.z() <= 0) {
        return false;
    }
    return true;
}

bool SegmentBlock::needs_transformation(const Eigen::Vector3d& previous_direction) {
    Eigen::Vector3d z_axis(0, 0, 1);
    return (previous_direction - z_axis).norm() > 1e-6;
}

Eigen::Vector3d SegmentBlock::convert_joint_to_segment_position(const std::vector<Eigen::Vector3d>& joint_positions,
                                                               int segment_index) {
    int num_joints = static_cast<int>(joint_positions.size());
    
    if (segment_index == 1) {
        // S1 = J1 + direction(J1→J2) × distance(J1-J0)
        Eigen::Vector3d direction = (joint_positions[2] - joint_positions[1]).normalized();
        double distance = (joint_positions[1] - joint_positions[0]).norm();
        return joint_positions[1] + direction * distance;
    } else {
        // Si = Ji + direction(Ji→Ji+1) × distance(Ji-Si-1)
        Eigen::Vector3d prev_S = convert_joint_to_segment_position(joint_positions, segment_index - 1);
        
        if (segment_index + 1 >= num_joints - 1) {
            // Last segment
            Eigen::Vector3d direction = (joint_positions[num_joints - 1] - joint_positions[segment_index]).normalized();
            double distance = (joint_positions[segment_index] - prev_S).norm();
            return joint_positions[segment_index] + direction * distance;
        } else {
            Eigen::Vector3d direction = (joint_positions[segment_index + 1] - joint_positions[segment_index]).normalized();
            double distance = (joint_positions[segment_index] - prev_S).norm();
            return joint_positions[segment_index] + direction * distance;
        }
    }
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> SegmentBlock::calculate_directions_from_joints(
    const std::vector<Eigen::Vector3d>& joint_positions, int segment_index) {
    
    Eigen::Vector3d current_segment_position = convert_joint_to_segment_position(joint_positions, segment_index);
    Eigen::Vector3d current_direction = current_segment_position.normalized();
    
    Eigen::Vector3d previous_direction;
    if (segment_index == 1) {
        previous_direction = Eigen::Vector3d(0, 0, 1);
    } else {
        Eigen::Vector3d previous_segment_position = convert_joint_to_segment_position(joint_positions, segment_index - 1);
        previous_direction = previous_segment_position.normalized();
    }
    
    return std::make_pair(current_direction, previous_direction);
}

bool SegmentBlock::validate_joint_positions(const std::vector<Eigen::Vector3d>& joint_positions,
                                           int segment_index) {
    int num_joints = static_cast<int>(joint_positions.size());
    
    if (segment_index < 0) {
        return false;
    }
    
    // For 9 joints: support segments 0-7 (S0 + 7 actual segments)
    int max_segment_index = num_joints - 2; // For 9 joints: max = 7
    if (segment_index > max_segment_index) {
        return false;
    }
    
    // S0 always valid with 2+ joints
    if (segment_index == 0) {
        return num_joints >= 2;
    }
    
    // Last segment (segment 7 for 9 joints) needs special handling
    if (segment_index == max_segment_index) {
        return num_joints >= 3; // Need at least 3 joints for last segment
    }
    
    // Normal segments need sufficient joints
    if (num_joints < segment_index + 3) {
        return false;
    }
    
    return true;
}

} // namespace delta