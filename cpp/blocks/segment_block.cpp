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
            // Step 1: Validate joint positions input
            if (!validate_joint_positions(joint_positions, segment_index)) {
                return SegmentResult(0.0, calculation_time_ms, false, 
                    "Invalid joint positions or segment index");
            }
            
            // Step 2: Convert J-points to S-point
            Eigen::Vector3d current_segment_position = convert_joint_to_segment_position(joint_positions, segment_index);
            
            // Step 3: Calculate direction vectors for coordinate transformation
            auto [current_direction, previous_direction] = calculate_directions_from_joints(joint_positions, segment_index);
            
            // Step 4: Call existing essential calculation
            SegmentResult result = calculate_essential(current_direction, previous_direction);
            
            // Step 5: Add J→S conversion data to result
            if (result.calculation_successful) {
                result.calculated_segment_position = current_segment_position;
                result.calculated_direction = current_direction;
                if (segment_index > 0) {
                    result.previous_segment_position = convert_joint_to_segment_position(joint_positions, segment_index - 1);
                }
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
            // Step 1: Validate joint positions input
            if (!validate_joint_positions(joint_positions, segment_index)) {
                return SegmentResult(0.0, calculation_time_ms, false, 
                    "Invalid joint positions or segment index");
            }
            
            // Step 2: Convert J-points to S-point
            Eigen::Vector3d current_segment_position = convert_joint_to_segment_position(joint_positions, segment_index);
            
            // Step 3: Calculate direction vectors for coordinate transformation
            auto [current_direction, previous_direction] = calculate_directions_from_joints(joint_positions, segment_index);
            
            // Step 4: Call existing complete calculation
            SegmentResult result = calculate_complete(current_direction, previous_direction);
            
            // Step 5: Add J→S conversion data to result
            if (result.calculation_successful) {
                result.calculated_segment_position = current_segment_position;
                result.calculated_direction = current_direction;
                if (segment_index > 0) {
                    result.previous_segment_position = convert_joint_to_segment_position(joint_positions, segment_index - 1);
                }
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
        // Vectors are parallel (handled above) or antiparallel
        return current_direction;
    }
    
    rotation_axis = rotation_axis.normalized();
    
    // Calculate rotation angle
    double cos_angle = prev_normalized.dot(z_axis);
    double angle = std::acos(std::max(-1.0, std::min(1.0, cos_angle)));
    
    // Apply rotation to current direction using Rodrigues' rotation formula
    return rodrigues_rotation(current_direction, rotation_axis, angle);
}

Eigen::Vector3d SegmentBlock::rodrigues_rotation(const Eigen::Vector3d& vector,
                                                const Eigen::Vector3d& axis,
                                                double angle) {
    double cos_angle = std::cos(angle);
    double sin_angle = std::sin(angle);
    
    // Rodrigues' rotation formula: v' = v*cos(θ) + (k×v)*sin(θ) + k*(k·v)*(1-cos(θ))
    Eigen::Vector3d k_cross_v = axis.cross(vector);
    double k_dot_v = axis.dot(vector);
    
    return vector * cos_angle + k_cross_v * sin_angle + axis * k_dot_v * (1.0 - cos_angle);
}

bool SegmentBlock::validate_transformed_direction(const Eigen::Vector3d& transformed_direction) {
    // Check for non-zero vector
    if (transformed_direction.norm() < 1e-10) {
        return false;
    }
    
    // Check for positive Z component after transformation
    if (transformed_direction.z() <= 0) {
        return false;
    }
    
    return true;
}

bool SegmentBlock::needs_transformation(const Eigen::Vector3d& previous_direction) {
    // Check if previous_direction is the default Z+ (0, 0, 1)
    Eigen::Vector3d z_axis(0, 0, 1);
    return (previous_direction - z_axis).norm() > 1e-6;
}

Eigen::Vector3d SegmentBlock::convert_joint_to_segment_position(const std::vector<Eigen::Vector3d>& joint_positions,
                                                               int segment_index) {
    int num_joints = static_cast<int>(joint_positions.size());
    
    // Special case: Last segment uses final joint as end-effector directly
    if (segment_index == num_joints - 2) {
        return joint_positions[num_joints - 1];  // Final joint is the segment position
    }
    
    if (segment_index == 0) {
        // S1 = J1 + direction(J1→J2) × distance(J1-J0)
        Eigen::Vector3d direction = (joint_positions[2] - joint_positions[1]).normalized();
        double distance = (joint_positions[1] - joint_positions[0]).norm();
        return joint_positions[1] + direction * distance;
    } else {
        // Si = Ji + direction(Ji→Ji+1) × distance(Ji-Si-1)
        Eigen::Vector3d prev_S = convert_joint_to_segment_position(joint_positions, segment_index - 1);
        Eigen::Vector3d direction = (joint_positions[segment_index + 2] - joint_positions[segment_index + 1]).normalized();
        double distance = (joint_positions[segment_index + 1] - prev_S).norm();
        return joint_positions[segment_index + 1] + direction * distance;
    }
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> SegmentBlock::calculate_directions_from_joints(
    const std::vector<Eigen::Vector3d>& joint_positions, int segment_index) {
    
    // Calculate current segment position
    Eigen::Vector3d current_segment_position = convert_joint_to_segment_position(joint_positions, segment_index);
    
    // Calculate current direction (from origin to current segment)
    Eigen::Vector3d current_direction = current_segment_position.normalized();
    
    // Calculate previous direction
    Eigen::Vector3d previous_direction;
    if (segment_index == 0) {
        // First segment - use default Z+
        previous_direction = Eigen::Vector3d(0, 0, 1);
    } else {
        // Calculate previous segment position and direction
        Eigen::Vector3d previous_segment_position = convert_joint_to_segment_position(joint_positions, segment_index - 1);
        previous_direction = previous_segment_position.normalized();
    }
    
    return std::make_pair(current_direction, previous_direction);
}

bool SegmentBlock::validate_joint_positions(const std::vector<Eigen::Vector3d>& joint_positions,
                                           int segment_index) {
    int num_joints = static_cast<int>(joint_positions.size());
    
    // Check for valid segment index
    if (segment_index < 0) {
        return false;
    }
    
    // Maximum possible segments is num_joints - 1
    if (segment_index >= num_joints - 1) {
        return false;
    }
    
    // Special case: Last segment only needs final joint
    if (segment_index == num_joints - 2) {
        return num_joints >= 2;  // Need at least 2 joints for last segment
    }
    
    // Normal segments need at least segment_index + 3 joints
    int required_joints = segment_index + 3;
    if (num_joints < required_joints) {
        return false;
    }
    
    // Check that joints are not degenerate (same positions)
    for (int i = segment_index; i < segment_index + 3 && i + 1 < num_joints; i++) {
        if ((joint_positions[i + 1] - joint_positions[i]).norm() < 1e-10) {
            return false; // Degenerate joint pair
        }
    }
    
    return true;
}

} // namespace delta