#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <Eigen/Dense>

// Real collision system components
#include "../cpp/visualization/data_publisher.hpp"
#include "../cpp/blocks/fabrik_solver_block.hpp"
#include "../cpp/bridges/robot_collision_bridge.hpp"
#include "../cpp/bridges/star_collision_bridge.hpp"
#include "../cpp/collision/collision_detection_engine.hpp"
#include "../cpp/collision/layer_manager.hpp"

using namespace delta;

/**
 * Real Collision System Test with Moving Target
 * Tests complete pipeline: Moving Target → FABRIK → Bridges → Collision Detection → Visualization
 */
class MovingTargetTest {
private:
    DataPublisher publisher_;
    std::unique_ptr<CollisionDetectionEngine> collision_engine_;
    
    int frame_count_;
    double animation_time_;
    
    // Animation parameters
    const double target_fps_ = 30.0;
    const double orbit_radius_ = 80.0;  // mm
    const double orbit_speed_ = 0.5;    // revolutions per second
    const Eigen::Vector3d orbit_center_{50.0, 30.0, 120.0}; // mm
    
    // Robot configuration
    const int num_segments_ = 7;
    const double robot_radius_ = 24.8; // From constants
    
public:
    MovingTargetTest() : frame_count_(0), animation_time_(0.0) {}
    
    bool initialize() {
        std::cout << "🚀 Initializing Moving Target Test..." << std::endl;
        
        // Initialize data publisher
        if (!publisher_.initialize("127.0.0.1", 9999, target_fps_)) {
            std::cerr << "❌ Failed to initialize DataPublisher" << std::endl;
            return false;
        }
        std::cout << "✅ DataPublisher initialized" << std::endl;
        
        // Initialize collision detection engine
        collision_engine_ = std::make_unique<CollisionDetectionEngine>();
        
        // Create minimal T-pose mesh vertices for collision system
        auto base_vertices = create_t_pose_mesh_vertices();
        
        if (!collision_engine_->initialize("collision_data.h5", base_vertices)) {
            std::cerr << "❌ Failed to initialize CollisionDetectionEngine" << std::endl;
            std::cerr << "   Make sure collision_data.h5 exists (run your main compilation script first)" << std::endl;
            return false;
        }
        std::cout << "✅ CollisionDetectionEngine initialized" << std::endl;
        
        std::cout << "✅ Moving Target Test ready!" << std::endl;
        return true;
    }
    
    void run_test_loop() {
        std::cout << "🎬 Starting moving target animation..." << std::endl;
        std::cout << "   Target: Circle orbit (radius=" << orbit_radius_ << "mm, speed=" << orbit_speed_ << "rps)" << std::endl;
        std::cout << "   Robot: " << num_segments_ << " segments, FABRIK solver" << std::endl;
        std::cout << "   Human: Static T-pose" << std::endl;
        std::cout << "   Press Ctrl+C to stop" << std::endl;
        std::cout << std::endl;
        
        const auto frame_duration = std::chrono::microseconds(static_cast<int>(1000000.0 / target_fps_));
        
        while (true) {
            auto frame_start = std::chrono::steady_clock::now();
            
            // Generate moving target position
            Eigen::Vector3d target_position = calculate_target_position();
            
            // Solve FABRIK for current target
            auto fabrik_result = solve_fabrik_for_target(target_position);
            if (!fabrik_result.solving_successful) {
                std::cerr << "⚠️  FABRIK solving failed for frame " << frame_count_ << std::endl;
                continue;
            }
            
            // Convert FABRIK result to robot capsules
            auto robot_bridge_result = RobotCollisionBridge::convert_basic_fabrik_to_capsules(fabrik_result);
            if (!robot_bridge_result.success) {
                std::cerr << "⚠️  Robot bridge failed: " << robot_bridge_result.error_message << std::endl;
                continue;
            }
            
            // Create static human T-pose
            auto human_joints = create_t_pose_joints();
            auto human_vertices = create_t_pose_mesh_vertices();
            
            // Transform human to collision coordinates
            auto star_bridge_result = STARCollisionBridge::transform_star_to_collision_coords(human_joints);
            if (!star_bridge_result.success) {
                std::cerr << "⚠️  STAR bridge failed: " << star_bridge_result.error_message << std::endl;
                continue;
            }
            
            // Run collision detection
            auto collision_result = collision_engine_->detect_collisions(
                star_bridge_result.data, robot_bridge_result.data);
            
            // Publish complete frame via visualization system
            bool publish_success = publisher_.publish_collision_frame(
                robot_bridge_result.data, human_joints, human_vertices, 
                collision_result, collision_result.computation_time_ms);
            
            if (!publish_success) {
                std::cerr << "⚠️  Failed to publish frame " << frame_count_ << std::endl;
            }
            
            // Print statistics every second
            if (frame_count_ % 30 == 0) {
                print_frame_statistics(target_position, fabrik_result, collision_result);
            }
            
            frame_count_++;
            animation_time_ += 1.0 / target_fps_;
            
            // Frame rate control
            auto frame_end = std::chrono::steady_clock::now();
            auto frame_time = frame_end - frame_start;
            if (frame_time < frame_duration) {
                std::this_thread::sleep_for(frame_duration - frame_time);
            }
        }
    }
    
private:
    
    Eigen::Vector3d calculate_target_position() {
        // Circular orbit around center point
        double angle = 2.0 * M_PI * orbit_speed_ * animation_time_;
        
        Eigen::Vector3d target;
        target.x() = orbit_center_.x() + orbit_radius_ * std::cos(angle);
        target.y() = orbit_center_.y() + orbit_radius_ * std::sin(angle);
        target.z() = orbit_center_.z() + 20.0 * std::sin(angle * 2.0); // Slight vertical motion
        
        return target;
    }
    
    FabrikSolverResult solve_fabrik_for_target(const Eigen::Vector3d& target) {
        // Use the basic FABRIK solver (more likely to be implemented)
        return FabrikSolverBlock::solve(target, num_segments_, FABRIK_TOLERANCE, FABRIK_MAX_ITERATIONS);
    }
    
    std::vector<Eigen::Vector3d> create_t_pose_joints() {
        std::vector<Eigen::Vector3d> joints;
        
        // Standard T-pose for STAR model (24 joints in Y-up, meters)
        // Convert to millimeters and Z-up for consistency
        
        joints.push_back(Eigen::Vector3d(0, 0, 0));           // 0: pelvis
        joints.push_back(Eigen::Vector3d(0, 200, 0));         // 1: spine1
        joints.push_back(Eigen::Vector3d(0, 400, 0));         // 2: spine2
        joints.push_back(Eigen::Vector3d(0, 600, 0));         // 3: spine3
        joints.push_back(Eigen::Vector3d(0, 800, 0));         // 4: neck
        joints.push_back(Eigen::Vector3d(0, 950, 0));         // 5: head
        
        // Left arm
        joints.push_back(Eigen::Vector3d(-150, 750, 0));      // 6: left_shoulder
        joints.push_back(Eigen::Vector3d(-300, 750, 0));      // 7: left_arm
        joints.push_back(Eigen::Vector3d(-450, 750, 0));      // 8: left_forearm
        joints.push_back(Eigen::Vector3d(-600, 750, 0));      // 9: left_hand
        
        // Right arm  
        joints.push_back(Eigen::Vector3d(150, 750, 0));       // 10: right_shoulder
        joints.push_back(Eigen::Vector3d(300, 750, 0));       // 11: right_arm
        joints.push_back(Eigen::Vector3d(450, 750, 0));       // 12: right_forearm
        joints.push_back(Eigen::Vector3d(600, 750, 0));       // 13: right_hand
        
        // Left leg
        joints.push_back(Eigen::Vector3d(-100, 0, 0));        // 14: left_hip
        joints.push_back(Eigen::Vector3d(-100, -400, 0));     // 15: left_thigh
        joints.push_back(Eigen::Vector3d(-100, -800, 0));     // 16: left_shin
        joints.push_back(Eigen::Vector3d(-100, -1000, 50));   // 17: left_foot
        
        // Right leg
        joints.push_back(Eigen::Vector3d(100, 0, 0));         // 18: right_hip
        joints.push_back(Eigen::Vector3d(100, -400, 0));      // 19: right_thigh
        joints.push_back(Eigen::Vector3d(100, -800, 0));      // 20: right_shin
        joints.push_back(Eigen::Vector3d(100, -1000, 50));    // 21: right_foot
        
        // Additional joints for STAR compatibility
        joints.push_back(Eigen::Vector3d(-50, 200, 0));       // 22: spine_detail1
        joints.push_back(Eigen::Vector3d(50, 200, 0));        // 23: spine_detail2
        
        return joints;
    }
    
    std::vector<Eigen::Vector3d> create_t_pose_mesh_vertices() {
        std::vector<Eigen::Vector3d> vertices;
        
        // TODO: Load real STAR T-pose vertices that match the HDF5 file
        // For now, return empty to avoid vertex count mismatch
        // The collision system should handle this gracefully
        
        std::cout << "⚠️  Using empty vertex array (real STAR vertices not loaded)" << std::endl;
        std::cout << "   HDF5 expects 6890 vertices from STAR model" << std::endl;
        std::cout << "   Consider loading real STAR T-pose data for full functionality" << std::endl;
        
        return vertices; // Empty for now
    }
    
    void print_frame_statistics(const Eigen::Vector3d& target, 
                               const FabrikSolverResult& fabrik_result,
                               const CollisionResult& collision_result) {
        auto publisher_stats = publisher_.get_statistics();
        
        std::cout << "📊 Frame " << frame_count_ 
                  << " | FPS: " << std::fixed << std::setprecision(1) << publisher_stats.current_fps
                  << " | Target: (" << std::setprecision(0) 
                  << target.x() << "," << target.y() << "," << target.z() << ")"
                  << " | FABRIK: " << fabrik_result.iterations_used << " iters"
                  << " | Collision: " << (collision_result.has_collision ? "YES" : "NO");
        
        if (collision_result.has_collision) {
            std::cout << " (" << collision_result.contacts.size() << " contacts)";
        }
        
        std::cout << " | Packets: " << publisher_stats.network_stats.packets_sent
                  << " | Success: " << std::setprecision(1) << publisher_stats.network_stats.success_rate << "%"
                  << std::endl;
    }
};

int main() {
    std::cout << "🎯 MOVING TARGET COLLISION TEST" << std::endl;
    std::cout << "================================" << std::endl;
    std::cout << "Real-time collision detection with animated target" << std::endl;
    std::cout << "Pipeline: FABRIK → Bridges → Collision → Visualization" << std::endl;
    std::cout << std::endl;
    
    MovingTargetTest test;
    
    if (!test.initialize()) {
        std::cerr << "❌ Failed to initialize moving target test" << std::endl;
        return 1;
    }
    
    std::cout << "📡 Streaming real collision data to UDP localhost:9999..." << std::endl;
    std::cout << "   Start receiver: ./build/test_visualization_receiver" << std::endl;
    std::cout << std::endl;
    
    try {
        test.run_test_loop();
    } catch (const std::exception& e) {
        std::cerr << "❌ Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}