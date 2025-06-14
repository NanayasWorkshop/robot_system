#include "star_collision_bridge.hpp"
#include <iostream>
#include <iomanip>

namespace delta {

// STAR joint names for debugging and reference
const std::vector<std::string> STARCollisionBridge::STAR_JOINT_NAMES = {
    "pelvis", "left_hip", "right_hip", "spine1", "left_knee", "right_knee",
    "spine2", "left_ankle", "right_ankle", "spine3", "left_foot", "right_foot",
    "neck", "left_collar", "right_collar", "head", "left_shoulder", "right_shoulder",
    "left_elbow", "right_elbow", "left_wrist", "right_wrist", "left_hand", "right_hand"
};

// =============================================================================
// SIMPLIFIED MAIN BRIDGE INTERFACE
// =============================================================================

BridgeResult<bool> STARCollisionBridge::update_collision_from_star(
    CollisionDetectionEngine& engine,
    const std::vector<Eigen::Vector3d>& star_joints,
    bool debug_logging) {
    
    double total_time_ms = 0.0;
    ScopedTimer timer(total_time_ms);
    
    // Validate input
    if (!engine.is_initialized()) {
        return BridgeResult<bool>("Collision detection engine not initialized", total_time_ms);
    }
    
    if (!validate_star_joints(star_joints)) {
        return BridgeResult<bool>("Invalid STAR joint positions", total_time_ms);
    }
    
    debug_print("STAR bridge: Processing " + std::to_string(star_joints.size()) + " joints (direct pass-through)", debug_logging);
    
    // SIMPLIFIED: No coordinate transformation - collision system operates in STAR space
    std::cout << "STAR bridge: " << star_joints.size() 
              << " joints validated and ready (no transformation needed)" << std::endl;
    
    return BridgeResult<bool>(true, total_time_ms);
}

BridgeResult<bool> STARCollisionBridge::update_layer_manager_from_star(
    LayerManager& layer_manager,
    const std::vector<Eigen::Vector3d>& star_joints,
    bool debug_logging) {
    
    double update_time_ms = 0.0;
    ScopedTimer timer(update_time_ms);
    
    if (!validate_star_joints(star_joints)) {
        return BridgeResult<bool>("Invalid STAR joint positions", update_time_ms);
    }
    
    // Update layer manager directly with STAR data (no transformation)
    bool update_success = layer_manager.update_human_pose(star_joints);
    if (!update_success) {
        return BridgeResult<bool>("LayerManager pose update failed", update_time_ms);
    }
    
    debug_print("LayerManager updated with STAR pose (direct)", debug_logging);
    return BridgeResult<bool>(true, update_time_ms);
}

// =============================================================================
// VALIDATION AND UTILITY METHODS
// =============================================================================

bool STARCollisionBridge::validate_star_joints(const std::vector<Eigen::Vector3d>& star_joints) {
    // Check joint count
    if (static_cast<int>(star_joints.size()) != EXPECTED_JOINT_COUNT) {
        std::cerr << "❌ STAR bridge: Expected " << EXPECTED_JOINT_COUNT 
                  << " joints, got " << star_joints.size() << std::endl;
        return false;
    }
    
    // Check for finite values
    for (size_t i = 0; i < star_joints.size(); ++i) {
        if (!star_joints[i].allFinite()) {
            std::cerr << "❌ STAR bridge: Joint " << i;
            if (i < STAR_JOINT_NAMES.size()) {
                std::cerr << " (" << STAR_JOINT_NAMES[i] << ")";
            }
            std::cerr << " contains non-finite values" << std::endl;
            return false;
        }
    }
    
    // Check for reasonable STAR coordinate ranges (meters, Y-up)
    for (size_t i = 0; i < star_joints.size(); ++i) {
        const auto& joint = star_joints[i];
        
        // STAR coordinates should be in reasonable human ranges (meters)
        if (std::abs(joint.x()) > 5.0 || std::abs(joint.y()) > 3.0 || std::abs(joint.z()) > 5.0) {
            std::cerr << "⚠️  STAR bridge: Joint " << i;
            if (i < STAR_JOINT_NAMES.size()) {
                std::cerr << " (" << STAR_JOINT_NAMES[i] << ")";
            }
            std::cerr << " has unusual coordinates: (" 
                      << joint.x() << ", " << joint.y() << ", " << joint.z() << ")" << std::endl;
            // Continue anyway - might be a different scale or pose
        }
    }
    
    return true;
}

void STARCollisionBridge::debug_print(const std::string& message, bool debug_logging) {
    if (debug_logging) {
        std::cout << "  " << message << std::endl;
    }
}

std::string STARCollisionBridge::get_coordinate_info(const std::vector<Eigen::Vector3d>& joints) {
    if (joints.empty()) {
        return "No joints";
    }
    
    // Calculate bounding box
    Eigen::Vector3d min_coords = joints[0];
    Eigen::Vector3d max_coords = joints[0];
    
    for (const auto& joint : joints) {
        for (int i = 0; i < 3; ++i) {
            min_coords[i] = std::min(min_coords[i], joint[i]);
            max_coords[i] = std::max(max_coords[i], joint[i]);
        }
    }
    
    // Calculate ranges
    Eigen::Vector3d ranges = max_coords - min_coords;
    
    std::ostringstream info;
    info << std::fixed << std::setprecision(2);
    info << "Range X:" << ranges.x() << " Y:" << ranges.y() << " Z:" << ranges.z();
    info << " Min:(" << min_coords.x() << "," << min_coords.y() << "," << min_coords.z() << ")";
    info << " Max:(" << max_coords.x() << "," << max_coords.y() << "," << max_coords.z() << ")";
    
    return info.str();
}

} // namespace delta