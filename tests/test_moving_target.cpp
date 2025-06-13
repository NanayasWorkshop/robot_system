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

// NEW: STAR vertex loader
#include "../cpp/utils/vertex_loader.hpp"

using namespace delta;

/**
 * Real Collision System Test with Moving Target (Updated with STAR Vertices)
 * Tests complete pipeline: Moving Target → FABRIK → Bridges → Collision Detection → Visualization
 */
class MovingTargetTest {
private:
    DataPublisher publisher_;
    std::unique_ptr<CollisionDetectionEngine> collision_engine_;
    
    // STAR vertex data
    std::vector<Eigen::Vector3d> star_vertices_;
    std::vector<Eigen::Vector3d> star_joints_;
    
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
        
        // NEW: Load real STAR vertices
        if (!load_star_vertices()) {
            std::cerr << "❌ Failed to load STAR vertices" << std::endl;
            return false;
        }
        
        // Initialize collision detection engine with real vertices
        collision_engine_ = std::make_unique<CollisionDetectionEngine>();
        
        if (!collision_engine_->initialize("collision_data.h5", star_vertices_)) {
            std::cerr << "❌ Failed to initialize CollisionDetectionEngine" << std::endl;
            std::cerr << "   Make sure collision_data.h5 exists and matches STAR vertex count" << std::endl;
            return false;
        }
        std::cout << "✅ CollisionDetectionEngine initialized with real STAR vertices" << std::endl;
        
        std::cout << "✅ Moving Target Test ready!" << std::endl;
        return true;
    }
    
    void run_test_loop() {
        std::cout << "🎬 Starting moving target animation..." << std::endl;
        std::cout << "   Target: Circle orbit (radius=" << orbit_radius_ << "mm, speed=" << orbit_speed_ << "rps)" << std::endl;
        std::cout << "   Robot: " << num_segments_ << " segments, FABRIK solver" << std::endl;
        std::cout << "   Human: Real STAR T-pose (" << star_vertices_.size() << " vertices)" << std::endl;
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
            
            // Use real STAR data (T-pose for now, vertices already loaded)
            // NOTE: For now we keep T-pose, but this is where dynamic STAR calls would go
            
            // Transform human to collision coordinates
            auto star_bridge_result = STARCollisionBridge::transform_star_to_collision_coords(star_joints_);
            if (!star_bridge_result.success) {
                std::cerr << "⚠️  STAR bridge failed: " << star_bridge_result.error_message << std::endl;
                continue;
            }
            
            // Run collision detection with real STAR vertices
            auto collision_result = collision_engine_->detect_collisions(
                star_bridge_result.data, robot_bridge_result.data);
            
            // Publish complete frame via visualization system
            bool publish_success = publisher_.publish_collision_frame(
                robot_bridge_result.data, star_joints_, star_vertices_, 
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
    
    // NEW: Load real STAR vertices using VertexLoader
    bool load_star_vertices() {
        std::cout << "🔄 Loading STAR vertices..." << std::endl;
        
        // Load vertices using the VertexLoader utility
        star_vertices_ = VertexLoader::load_or_generate_star_vertices(
            "star_vertices.bin",    // Binary file path
            "get_star_vertices.py"  // Python script path
        );
        
        if (star_vertices_.empty()) {
            std::cerr << "❌ Failed to load STAR vertices" << std::endl;
            return false;
        }
        
        // Validate vertex count
        if (!VertexLoader::validate_vertices(star_vertices_, 6890)) {
            std::cerr << "❌ STAR vertex validation failed" << std::endl;
            return false;
        }
        
        // Print vertex statistics
        std::cout << VertexLoader::get_vertex_statistics(star_vertices_) << std::endl;
        
        // Create corresponding T-pose joints (simplified for now)
        star_joints_ = create_star_t_pose_joints();
        
        std::cout << "✅ STAR vertices loaded successfully" << std::endl;
        std::cout << "   Vertices: " << star_vertices_.size() << std::endl;
        std::cout << "   Joints: " << star_joints_.size() << std::endl;
        
        return true;
    }
    
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
    
    // UPDATED: Create proper STAR joint positions that match the vertex data
    std::vector<Eigen::Vector3d> create_star_t_pose_joints() {
        std::vector<Eigen::Vector3d> joints;
        
        // Standard T-pose for STAR model (24 joints)
        // These should correspond to the actual STAR joint positions
        // STAR coordinate system: Y-up, meters
        
        // NOTE: These are approximate T-pose positions
        // For full accuracy, these should come from STAR joint regressor
        
        joints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));           // 0: pelvis
        joints.push_back(Eigen::Vector3d(-0.1, 0.0, 0.0));         // 1: left_hip
        joints.push_back(Eigen::Vector3d(0.1, 0.0, 0.0));          // 2: right_hip
        joints.push_back(Eigen::Vector3d(0.0, 0.2, 0.0));          // 3: spine1
        joints.push_back(Eigen::Vector3d(-0.1, -0.4, 0.0));        // 4: left_knee
        joints.push_back(Eigen::Vector3d(0.1, -0.4, 0.0));         // 5: right_knee
        joints.push_back(Eigen::Vector3d(0.0, 0.4, 0.0));          // 6: spine2
        joints.push_back(Eigen::Vector3d(-0.1, -0.8, 0.0));        // 7: left_ankle
        joints.push_back(Eigen::Vector3d(0.1, -0.8, 0.0));         // 8: right_ankle
        joints.push_back(Eigen::Vector3d(0.0, 0.6, 0.0));          // 9: spine3
        joints.push_back(Eigen::Vector3d(-0.1, -1.0, 0.05));       // 10: left_foot
        joints.push_back(Eigen::Vector3d(0.1, -1.0, 0.05));        // 11: right_foot
        joints.push_back(Eigen::Vector3d(0.0, 0.8, 0.0));          // 12: neck
        joints.push_back(Eigen::Vector3d(-0.15, 0.75, 0.0));       // 13: left_collar
        joints.push_back(Eigen::Vector3d(0.15, 0.75, 0.0));        // 14: right_collar
        joints.push_back(Eigen::Vector3d(0.0, 0.95, 0.0));         // 15: head
        joints.push_back(Eigen::Vector3d(-0.3, 0.75, 0.0));        // 16: left_shoulder
        joints.push_back(Eigen::Vector3d(0.3, 0.75, 0.0));         // 17: right_shoulder
        joints.push_back(Eigen::Vector3d(-0.45, 0.75, 0.0));       // 18: left_elbow
        joints.push_back(Eigen::Vector3d(0.45, 0.75, 0.0));        // 19: right_elbow
        joints.push_back(Eigen::Vector3d(-0.6, 0.75, 0.0));        // 20: left_wrist
        joints.push_back(Eigen::Vector3d(0.6, 0.75, 0.0));         // 21: right_wrist
        joints.push_back(Eigen::Vector3d(-0.65, 0.75, 0.0));       // 22: left_hand
        joints.push_back(Eigen::Vector3d(0.65, 0.75, 0.0));        // 23: right_hand
        
        return joints;
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
                  << " | STAR: " << star_vertices_.size() << " verts"
                  << std::endl;
    }
};

int main() {
    std::cout << "🎯 MOVING TARGET COLLISION TEST (WITH REAL STAR VERTICES)" << std::endl;
    std::cout << "==========================================================" << std::endl;
    std::cout << "Real-time collision detection with animated target" << std::endl;
    std::cout << "Pipeline: FABRIK → Bridges → Collision → Visualization" << std::endl;
    std::cout << "STAR Integration: Real 6890 vertices from T-pose model" << std::endl;
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