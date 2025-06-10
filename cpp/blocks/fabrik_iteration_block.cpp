#include "fabrik_iteration_block.hpp"
#include <cmath>
#include <algorithm>

namespace delta {

FabrikIterationResult FabrikIterationBlock::iterate(
    const std::vector<Eigen::Vector3d>& current_joints,
    const Eigen::Vector3d& target_position,
    const std::vector<double>& joint_distances) {
    
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        // Step 1: Input validation
        if (!validate_inputs(current_joints, joint_distances)) {
            return FabrikIterationResult(calculation_time_ms, 
                "Invalid inputs: joint count must match distance count");
        }
        
        // Step 2: Calculate cone half-angle from constant
        double cone_half_angle_rad = SPHERICAL_JOINT_CONE_ANGLE_RAD / 2.0;  // 60 degrees (half of 120)
        
        try {
            // Step 3: Backward pass (end-effector to base)
            std::vector<Eigen::Vector3d> backward_joints = backward_pass(
                current_joints, target_position, joint_distances, cone_half_angle_rad
            );
            
            // Step 4: Forward pass (base to end-effector)
            std::vector<Eigen::Vector3d> final_joints = forward_pass(
                backward_joints, current_joints, joint_distances, cone_half_angle_rad
            );
            
            // Step 5: Calculate final distance to target
            double distance_to_target = vector_distance(final_joints.back(), target_position);
            
            return FabrikIterationResult(final_joints, distance_to_target, calculation_time_ms);
            
        } catch (const std::exception& e) {
            return FabrikIterationResult(calculation_time_ms, 
                "FABRIK iteration failed: " + std::string(e.what()));
        }
    }
}

bool FabrikIterationBlock::validate_inputs(const std::vector<Eigen::Vector3d>& joints,
                                          const std::vector<double>& distances) {
    // Check for non-empty arrays
    if (joints.empty() || distances.empty()) {
        return false;
    }
    
    // Check joint-distance relationship: distances = joints - 1
    if (distances.size() != joints.size() - 1) {
        return false;
    }
    
    // Check for positive distances
    for (double dist : distances) {
        if (dist <= 0.0) {
            return false;
        }
    }
    
    return true;
}

std::vector<Eigen::Vector3d> FabrikIterationBlock::backward_pass(
    const std::vector<Eigen::Vector3d>& joints,
    const Eigen::Vector3d& target,
    const std::vector<double>& distances,
    double cone_half_angle_rad) {
    
    std::vector<Eigen::Vector3d> b_joints = joints;
    
    // Step 1: Move end effector to target
    b_joints.back() = target;
    
    // Step 2: Position remaining joints with cone constraints
    for (int i = static_cast<int>(b_joints.size()) - 2; i >= 0; i--) {
        if (i == static_cast<int>(b_joints.size()) - 2) {
            // First joint back (no cone constraint)
            Eigen::Vector3d direction = vector_subtract(joints[i], b_joints[i + 1]);
            Eigen::Vector3d direction_norm = normalize_vector(direction);
            b_joints[i] = b_joints[i + 1] + direction_norm * distances[i];
        } else {
            // Apply cone constraint
            // Cone apex is at the previously positioned joint
            Eigen::Vector3d cone_apex = b_joints[i + 1];
            // Cone axis points from joint two positions forward to apex
            Eigen::Vector3d cone_axis = vector_subtract(cone_apex, b_joints[i + 2]);
            
            // Desired direction
            Eigen::Vector3d desired_direction = vector_subtract(joints[i], cone_apex);
            
            // Apply cone constraint
            Eigen::Vector3d corrected_direction = project_onto_cone(desired_direction, cone_axis, cone_half_angle_rad);
            Eigen::Vector3d corrected_direction_norm = normalize_vector(corrected_direction);
            
            // Position joint at correct distance
            b_joints[i] = cone_apex + corrected_direction_norm * distances[i];
        }
    }
    
    return b_joints;
}

std::vector<Eigen::Vector3d> FabrikIterationBlock::forward_pass(
    const std::vector<Eigen::Vector3d>& backward_joints,
    const std::vector<Eigen::Vector3d>& original_joints,
    const std::vector<double>& distances,
    double cone_half_angle_rad) {
    
    std::vector<Eigen::Vector3d> f_joints = backward_joints;
    
    // Step 1: Fix base position (always at original position)
    f_joints[0] = original_joints[0];
    
    // Step 2: Position remaining joints with cone constraints
    for (size_t i = 1; i < f_joints.size(); i++) {
        if (i == 1) {
            // First joint from base (no cone constraint)
            Eigen::Vector3d direction = vector_subtract(backward_joints[i], f_joints[i - 1]);
            Eigen::Vector3d direction_norm = normalize_vector(direction);
            f_joints[i] = f_joints[i - 1] + direction_norm * distances[i - 1];
        } else {
            // Apply cone constraint
            // Cone apex is at the previously positioned joint
            Eigen::Vector3d cone_apex = f_joints[i - 1];
            // Cone axis points from joint two positions back toward apex
            Eigen::Vector3d cone_axis = vector_subtract(cone_apex, f_joints[i - 2]);
            
            // Desired direction
            Eigen::Vector3d desired_direction = vector_subtract(backward_joints[i], cone_apex);
            
            // Apply cone constraint
            Eigen::Vector3d corrected_direction = project_onto_cone(desired_direction, cone_axis, cone_half_angle_rad);
            Eigen::Vector3d corrected_direction_norm = normalize_vector(corrected_direction);
            
            // Position joint at correct distance
            f_joints[i] = cone_apex + corrected_direction_norm * distances[i - 1];
        }
    }
    
    return f_joints;
}

// Vector utilities (matching Python implementation)
Eigen::Vector3d FabrikIterationBlock::vector_subtract(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return a - b;
}

double FabrikIterationBlock::vector_magnitude(const Eigen::Vector3d& v) {
    return v.norm();
}

double FabrikIterationBlock::vector_distance(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return vector_magnitude(vector_subtract(a, b));
}

Eigen::Vector3d FabrikIterationBlock::normalize_vector(const Eigen::Vector3d& v) {
    double mag = vector_magnitude(v);
    if (mag < 1e-10) {
        return Eigen::Vector3d(0, 0, 0);
    }
    return v / mag;
}

double FabrikIterationBlock::vector_dot(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return a.dot(b);
}

double FabrikIterationBlock::angle_between_vectors(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    Eigen::Vector3d v1_norm = normalize_vector(v1);
    Eigen::Vector3d v2_norm = normalize_vector(v2);
    double dot_product = std::clamp(vector_dot(v1_norm, v2_norm), -1.0, 1.0);
    return std::acos(dot_product);
}

bool FabrikIterationBlock::is_inside_cone(const Eigen::Vector3d& direction, 
                                         const Eigen::Vector3d& cone_axis, 
                                         double cone_half_angle_rad) {
    double angle = angle_between_vectors(direction, cone_axis);
    return angle <= cone_half_angle_rad;
}

Eigen::Vector3d FabrikIterationBlock::project_onto_cone(const Eigen::Vector3d& direction,
                                                       const Eigen::Vector3d& cone_axis,
                                                       double cone_half_angle_rad) {
    if (is_inside_cone(direction, cone_axis, cone_half_angle_rad)) {
        return direction;
    }
    
    // Project direction onto plane perpendicular to cone axis
    Eigen::Vector3d direction_norm = normalize_vector(direction);
    Eigen::Vector3d cone_axis_norm = normalize_vector(cone_axis);
    
    // Component along cone axis
    Eigen::Vector3d axis_component = vector_dot(direction_norm, cone_axis_norm) * cone_axis_norm;
    
    // Component perpendicular to cone axis
    Eigen::Vector3d perp_component = direction_norm - axis_component;
    Eigen::Vector3d perp_component_norm = normalize_vector(perp_component);
    
    // Calculate the corrected direction on cone surface
    Eigen::Vector3d corrected_direction = (std::cos(cone_half_angle_rad) * cone_axis_norm + 
                                          std::sin(cone_half_angle_rad) * perp_component_norm);
    
    return corrected_direction;
}

} // namespace delta