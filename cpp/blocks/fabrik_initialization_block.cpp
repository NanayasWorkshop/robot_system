#include "fabrik_initialization_block.hpp"
#include <cmath>

namespace delta {

FabrikInitResult FabrikInitializationBlock::create_straight_chain(int num_segments) {
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        // Step 1: Input validation
        if (!validate_num_segments(num_segments)) {
            return FabrikInitResult(num_segments, calculation_time_ms, 
                "Invalid number of segments: must be positive integer");
        }
        
        // Step 2: Calculate number of joints
        int num_joints = num_segments + 2;
        
        // Step 3: Initialize joint positions array
        std::vector<Eigen::Vector3d> joints;
        joints.reserve(num_joints);
        
        // Step 4: Calculate joint positions
        try {
            // J0 - Base joint (always at origin)
            joints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            
            // J1 - First joint
            joints.push_back(calculate_first_joint_position());
            
            // J2 to J(n-1) - Middle joints
            for (int i = 2; i < num_joints - 1; i++) {
                joints.push_back(calculate_middle_joint_position(joints[i-1]));
            }
            
            // J_last - Final joint (end-effector)
            joints.push_back(calculate_last_joint_position(joints[num_joints-2]));
            
        } catch (const std::exception& e) {
            return FabrikInitResult(num_segments, calculation_time_ms, 
                "Joint calculation failed: " + std::string(e.what()));
        }
        
        // Step 5: Calculate joint distances
        std::vector<double> joint_distances = calculate_joint_distances(joints);
        
        // Step 6: Validate result consistency
        if (joints.size() != static_cast<size_t>(num_joints)) {
            return FabrikInitResult(num_segments, calculation_time_ms, 
                "Internal error: joint count mismatch");
        }
        
        if (joint_distances.size() != static_cast<size_t>(num_segments + 1)) {
            return FabrikInitResult(num_segments, calculation_time_ms, 
                "Internal error: distance count mismatch");
        }
        
        return FabrikInitResult(joints, joint_distances, num_segments, num_joints, calculation_time_ms);
    }
}

bool FabrikInitializationBlock::validate_num_segments(int num_segments) {
    // Check for positive number
    if (num_segments <= 0) {
        return false;
    }
    
    // Check for reasonable upper limit (prevent memory issues)
    if (num_segments > 1000) {
        return false;
    }
    
    return true;
}

Eigen::Vector3d FabrikInitializationBlock::calculate_first_joint_position() {
    // J1 = J0 + (WORKING_HEIGHT + MIN_HEIGHT/2 + MOTOR_LIMIT) * [0,0,1]
    double z_offset = FIRST_SEGMENT_LENGTH;
    return Eigen::Vector3d(0.0, 0.0, z_offset);
}

Eigen::Vector3d FabrikInitializationBlock::calculate_middle_joint_position(const Eigen::Vector3d& previous_joint) {
    // Ji = J(i-1) + (2*WORKING_HEIGHT + MIN_HEIGHT + 2*MOTOR_LIMIT) * [0,0,1]
    double z_offset = MIDDLE_SEGMENT_LENGTH;
    return Eigen::Vector3d(
        previous_joint.x(),
        previous_joint.y(), 
        previous_joint.z() + z_offset
    );
}

Eigen::Vector3d FabrikInitializationBlock::calculate_last_joint_position(const Eigen::Vector3d& previous_joint) {
    // J_last = J(n-1) + (WORKING_HEIGHT + MIN_HEIGHT/2 + MOTOR_LIMIT) * [0,0,1]
    double z_offset = LAST_SEGMENT_LENGTH;
    return Eigen::Vector3d(
        previous_joint.x(),
        previous_joint.y(),
        previous_joint.z() + z_offset
    );
}

std::vector<double> FabrikInitializationBlock::calculate_joint_distances(const std::vector<Eigen::Vector3d>& joints) {
    std::vector<double> distances;
    distances.reserve(joints.size() - 1);
    
    // Calculate distance between each consecutive pair of joints
    for (size_t i = 0; i < joints.size() - 1; i++) {
        Eigen::Vector3d diff = joints[i+1] - joints[i];
        double distance = diff.norm();
        distances.push_back(distance);
    }
    
    return distances;
}

} // namespace delta