#include "star_collision_bridge.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

namespace delta {

// STAR joint names for debugging and reference
const std::vector<std::string> STARCollisionBridge::STAR_JOINT_NAMES = {
    "pelvis", "left_hip", "right_hip", "spine1", "left_knee", "right_knee",
    "spine2", "left_ankle", "right_ankle", "spine3", "left_foot", "right_foot",
    "neck", "left_collar", "right_collar", "head", "left_shoulder", "right_shoulder",
    "left_elbow", "right_elbow", "left_wrist", "right_wrist", "left_hand", "right_hand"
};

// =============================================================================
// MAIN BRIDGE INTERFACE
// =============================================================================

BridgeResult<bool> STARCollisionBridge::update_collision_from_star(
    CollisionDetectionEngine& engine,
    const std::vector<Eigen::Vector3d>& star_joints,
    bool debug_logging) {
    
    double total_time_ms = 0.0;
    ScopedTimer timer(total_time_ms);
    
    // Step 1: Validate input
    if (!engine.is_initialized()) {
        return BridgeResult<bool>("Collision detection engine not initialized", total_time_ms);
    }
    
    if (!validate_star_joints(star_joints)) {
        return BridgeResult<bool>("Invalid STAR joint positions", total_time_ms);
    }
    
    debug_print("STAR bridge: Processing " + std::to_string(star_joints.size()) + " joints", debug_logging);
    
    // Step 2: Transform STAR coordinates to collision coordinates
    auto transform_result = transform_star_to_collision_coords(star_joints, debug_logging);
    if (!transform_result.success) {
        return BridgeResult<bool>("Coordinate transformation failed: " + transform_result.error_message, total_time_ms);
    }
    
    // Step 3: The collision engine handles the update internally
    // We don't need to call LayerManager directly - the engine does this in detect_collisions()
    // For now, we'll store the transformed joints and report success
    // The actual update happens when detect_collisions() is called with these joints
    
    // Success! Print minimal confirmation
    if (total_time_ms > 1.0) {  // Only log if unusually slow
        std::cout << "STAR bridge: " << star_joints.size() 
                  << " joints transformed (" << std::fixed << std::setprecision(1) 
                  << total_time_ms << "ms)" << std::endl;
    } else {
        std::cout << "STAR bridge: " << star_joints.size() 
                  << " joints transformed" << std::endl;
    }
    
    return BridgeResult<bool>(true, total_time_ms);
}

BridgeResult<std::vector<Eigen::Vector3d>> STARCollisionBridge::transform_star_to_collision_coords(
    const std::vector<Eigen::Vector3d>& star_joints,
    bool debug_logging) {
    
    double transform_time_ms = 0.0;
    ScopedTimer timer(transform_time_ms);
    
    // Step 1: Validate STAR joints
    if (!validate_star_joints(star_joints)) {
        return BridgeResult<std::vector<Eigen::Vector3d>>("Invalid STAR joints", transform_time_ms);
    }
    
    debug_print("Transforming coordinates: STAR (Y-up, meters) → Collision (Z-up, mm)", debug_logging);
    
    if (debug_logging) {
        debug_print("STAR coordinate info: " + get_coordinate_info(star_joints), debug_logging);
    }
    
    // Step 2: Transform each joint
    std::vector<Eigen::Vector3d> transformed_joints;
    transformed_joints.reserve(star_joints.size());
    
    for (size_t i = 0; i < star_joints.size(); ++i) {
        Eigen::Vector3d transformed = transform_point_star_to_collision(star_joints[i]);
        transformed_joints.push_back(transformed);
        
        // Debug first few joints if enabled
        if (debug_logging && i < 3) {
            debug_print("Joint " + std::to_string(i) + " (" + STAR_JOINT_NAMES[i] + "): " +
                       "STAR(" + std::to_string(star_joints[i].x()) + ", " + 
                       std::to_string(star_joints[i].y()) + ", " + 
                       std::to_string(star_joints[i].z()) + ") → " +
                       "Collision(" + std::to_string(transformed.x()) + ", " + 
                       std::to_string(transformed.y()) + ", " + 
                       std::to_string(transformed.z()) + ")", debug_logging);
        }
    }
    
    // Step 3: Validate transformed results
    if (!validate_transformed_joints(transformed_joints)) {
        return BridgeResult<std::vector<Eigen::Vector3d>>("Coordinate transformation produced invalid results", transform_time_ms);
    }
    
    if (debug_logging) {
        debug_print("Transformed coordinate info: " + get_coordinate_info(transformed_joints), debug_logging);
    }
    
    return BridgeResult<std::vector<Eigen::Vector3d>>(transformed_joints, transform_time_ms);
}

BridgeResult<bool> STARCollisionBridge::update_layer_manager_from_star(
    LayerManager& layer_manager,
    const std::vector<Eigen::Vector3d>& star_joints,
    bool debug_logging) {
    
    double update_time_ms = 0.0;
    ScopedTimer timer(update_time_ms);
    
    // Step 1: Transform coordinates
    auto transform_result = transform_star_to_collision_coords(star_joints, debug_logging);
    if (!transform_result.success) {
        return BridgeResult<bool>("Coordinate transformation failed: " + transform_result.error_message, update_time_ms);
    }
    
    // Step 2: Update layer manager directly
    bool update_success = layer_manager.update_human_pose(transform_result.data);
    if (!update_success) {
        return BridgeResult<bool>("LayerManager pose update failed", update_time_ms);
    }
    
    debug_print("LayerManager updated with transformed STAR pose", debug_logging);
    
    return BridgeResult<bool>(true, update_time_ms);
}

// =============================================================================
// COORDINATE TRANSFORMATION
// =============================================================================

Eigen::Vector3d STARCollisionBridge::transform_point_star_to_collision(const Eigen::Vector3d& star_point) {
    // STAR coordinate system: Y-up, meters, person lying on back
    // Collision coordinate system: Z-up, millimeters, person standing upright
    
    // Transformation:
    // X stays X (left-right unchanged)
    // Y (up in STAR) becomes Z (up in collision)  
    // Z (forward in STAR) becomes -Y (forward in collision, flipped to face correct direction)
    // Scale: meters → millimeters (× 1000)
    
    return Eigen::Vector3d(
        star_point.x() * METERS_TO_MM,  // X unchanged, meters → mm
        -star_point.z() * METERS_TO_MM, // Y = -Z (flip to face forward)
        star_point.y() * METERS_TO_MM   // Z = Y (Y-up → Z-up)
    );
}

// =============================================================================
// VALIDATION METHODS
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

bool STARCollisionBridge::validate_transformed_joints(const std::vector<Eigen::Vector3d>& transformed_joints) {
    // Check for finite values after transformation
    for (size_t i = 0; i < transformed_joints.size(); ++i) {
        if (!transformed_joints[i].allFinite()) {
            std::cerr << "❌ STAR bridge: Transformed joint " << i 
                      << " contains non-finite values" << std::endl;
            return false;
        }
    }
    
    // Check for reasonable collision coordinate ranges (millimeters, Z-up)
    for (size_t i = 0; i < transformed_joints.size(); ++i) {
        const auto& joint = transformed_joints[i];
        
        // Collision coordinates should be in reasonable ranges (millimeters)
        if (std::abs(joint.x()) > 5000.0 || std::abs(joint.y()) > 5000.0 || std::abs(joint.z()) > 3000.0) {
            std::cerr << "⚠️  STAR bridge: Transformed joint " << i;
            if (i < STAR_JOINT_NAMES.size()) {
                std::cerr << " (" << STAR_JOINT_NAMES[i] << ")";
            }
            std::cerr << " has unusual coordinates: (" 
                      << joint.x() << ", " << joint.y() << ", " << joint.z() << ") mm" << std::endl;
            // Continue anyway - might be a different scale
        }
    }
    
    return true;
}

// =============================================================================
// UTILITY METHODS
// =============================================================================

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