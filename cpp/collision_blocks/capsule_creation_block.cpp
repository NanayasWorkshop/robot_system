#include "capsule_creation_block.hpp"
#include <cmath>

namespace delta {

CapsuleChainResult CapsuleCreationBlock::create_capsule_chain(
    const std::vector<Eigen::Vector3d>& s_points,
    double robot_radius) {
    
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        try {
            // Step 1: Input validation
            if (!validate_s_points(s_points)) {
                return CapsuleChainResult(calculation_time_ms, 
                    "Invalid S-points: need at least 2 points");
            }
            
            if (!validate_radius(robot_radius)) {
                return CapsuleChainResult(calculation_time_ms, 
                    "Invalid robot radius: must be positive");
            }
            
            // Step 2: Create capsules between consecutive S-points
            std::vector<CapsuleData> capsules;
            capsules.reserve(s_points.size() - 1);
            double total_length = 0.0;
            
            for (size_t i = 0; i < s_points.size() - 1; i++) {
                CapsuleData capsule;
                capsule.start_point = s_points[i];
                capsule.end_point = s_points[i + 1];
                capsule.radius = robot_radius;
                capsule.length = (capsule.end_point - capsule.start_point).norm();
                
                capsules.push_back(capsule);
                total_length += capsule.length;
            }
            
            return CapsuleChainResult(capsules, total_length, calculation_time_ms, true);
            
        } catch (const std::exception& e) {
            return CapsuleChainResult(calculation_time_ms, 
                "Capsule creation failed: " + std::string(e.what()));
        }
    }
}

CapsuleChainResult CapsuleCreationBlock::update_capsule_positions(
    const std::vector<CapsuleData>& existing_capsules,
    const std::vector<Eigen::Vector3d>& new_s_points) {
    
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        try {
            // Step 1: Input validation
            if (!validate_capsule_update(existing_capsules, new_s_points)) {
                return CapsuleChainResult(calculation_time_ms, 
                    "Invalid update: S-points count must match existing capsules");
            }
            
            // Step 2: Update capsule positions, preserve radius
            std::vector<CapsuleData> updated_capsules;
            updated_capsules.reserve(existing_capsules.size());
            double total_length = 0.0;
            
            for (size_t i = 0; i < existing_capsules.size(); i++) {
                CapsuleData updated_capsule;
                updated_capsule.start_point = new_s_points[i];
                updated_capsule.end_point = new_s_points[i + 1];
                updated_capsule.radius = existing_capsules[i].radius;  // Preserve original radius
                updated_capsule.length = (updated_capsule.end_point - updated_capsule.start_point).norm();
                
                updated_capsules.push_back(updated_capsule);
                total_length += updated_capsule.length;
            }
            
            return CapsuleChainResult(updated_capsules, total_length, calculation_time_ms, true);
            
        } catch (const std::exception& e) {
            return CapsuleChainResult(calculation_time_ms, 
                "Capsule update failed: " + std::string(e.what()));
        }
    }
}

bool CapsuleCreationBlock::validate_s_points(const std::vector<Eigen::Vector3d>& s_points) {
    if (s_points.size() < 2) {
        return false;
    }
    
    // Check for finite values
    for (const auto& point : s_points) {
        if (!point.allFinite()) {
            return false;
        }
    }
    
    // Check for non-degenerate segments (no duplicate consecutive points)
    for (size_t i = 0; i < s_points.size() - 1; i++) {
        double distance = (s_points[i + 1] - s_points[i]).norm();
        if (distance < 1e-6) {
            return false; // Degenerate segment
        }
    }
    
    return true;
}

bool CapsuleCreationBlock::validate_radius(double robot_radius) {
    return (robot_radius > 0.0) && std::isfinite(robot_radius);
}

bool CapsuleCreationBlock::validate_capsule_update(const std::vector<CapsuleData>& existing_capsules,
                                                   const std::vector<Eigen::Vector3d>& new_s_points) {
    // Need exactly one more S-point than existing capsules
    if (new_s_points.size() != existing_capsules.size() + 1) {
        return false;
    }
    
    return validate_s_points(new_s_points);
}

} // namespace delta