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

// STAR daemon client (replaces vertex loader)
#include "../cpp/utils/star_daemon_client.hpp"

using namespace delta;

/**
 * Real Collision System Test with Moving Target (UPDATED: Robot→STAR Transform)
 * Tests complete pipeline: Moving Target → FABRIK → Robot Transform → STAR Collision → Visualization
 * NEW APPROACH: Transform robot to STAR coordinates instead of STAR to collision coordinates
 */
class MovingTargetTest {
private:
    DataPublisher publisher_;
    std::unique_ptr<CollisionDetectionEngine> collision_engine_;
    std::unique_ptr<STARDaemonClient> star_client_;
    
    // Current mesh data (updated each frame) - in STAR coordinates
    std::vector<Eigen::Vector3d> current_vertices_;
    std::vector<Eigen::Vector3d> current_joints_;
    
    int frame_count_;
    double animation_time_;
    
    // Animation parameters (robot coordinates - Z-up, mm)
    const double target_fps_ = 30.0;
    const double orbit_radius_ = 160.0;  // mm
    const double orbit_speed_ = 0.05;    // revolutions per second
    const Eigen::Vector3d orbit_center_{50.0, 30.0, 320.0}; // mm (robot coordinates)
    
    // Robot configuration
    const int num_segments_ = 7;
    const double robot_radius_ = 24.8; // From constants
    
    // Robot positioning in STAR coordinate space (meters)
    const Eigen::Vector3d robot_offset_{0.5, 0.0, 0.3}; // X=0.5m right, Y=0.0m, Z=0.3m forward
    
public:
    MovingTargetTest() : frame_count_(0), animation_time_(0.0) {}
    
    bool initialize() {
        std::cout << "🚀 Initializing Moving Target Test (Robot→STAR Transform)..." << std::endl;
        
        // Initialize data publisher
        if (!publisher_.initialize("127.0.0.1", 9999, target_fps_)) {
            std::cerr << "❌ Failed to initialize DataPublisher" << std::endl;
            return false;
        }
        std::cout << "✅ DataPublisher initialized" << std::endl;
        
        // Connect to STAR daemon
        if (!connect_to_star_daemon()) {
            std::cerr << "❌ Failed to connect to STAR daemon" << std::endl;
            return false;
        }
        
        // Initialize collision detection engine with STAR vertices (no transformation!)
        collision_engine_ = std::make_unique<CollisionDetectionEngine>();
        
        if (!collision_engine_->initialize("collision_data.h5", current_vertices_)) {
            std::cerr << "❌ Failed to initialize CollisionDetectionEngine" << std::endl;
            std::cerr << "   Make sure collision_data.h5 exists and matches STAR vertex count" << std::endl;
            return false;
        }
        std::cout << "✅ CollisionDetectionEngine initialized with STAR vertices (Y-up, meters)" << std::endl;
        
        std::cout << "✅ Moving Target Test ready (Robot→STAR coordinate transformation)!" << std::endl;
        return true;
    }
    
    void run_test_loop() {
        std::cout << "🎬 Starting moving target animation (Robot→STAR transform)..." << std::endl;
        std::cout << "   Target: Circle orbit in robot coordinates (radius=" << orbit_radius_ << "mm, speed=" << orbit_speed_ << "rps)" << std::endl;
        std::cout << "   Robot: " << num_segments_ << " segments, FABRIK solver → Transform to STAR coordinates with offset(" 
                  << robot_offset_.x() << ", " << robot_offset_.y() << ", " << robot_offset_.z() << ")m" << std::endl;
        std::cout << "   Human: Real STAR with animated mesh (" << current_vertices_.size() << " vertices, native coordinates)" << std::endl;
        std::cout << "   Collision: Both robot and human in STAR coordinate space (Y-up, meters)" << std::endl;
        std::cout << "   Press Ctrl+C to stop" << std::endl;
        std::cout << std::endl;
        
        const auto frame_duration = std::chrono::microseconds(static_cast<int>(1000000.0 / target_fps_));
        
        while (true) {
            auto frame_start = std::chrono::steady_clock::now();
            
            // Generate moving target position (robot coordinates: Z-up, mm)
            Eigen::Vector3d target_position = calculate_target_position();
            
            // Solve FABRIK for current target (robot coordinates)
            auto fabrik_result = solve_fabrik_for_target(target_position);
            if (!fabrik_result.solving_successful) {
                std::cerr << "⚠️  FABRIK solving failed for frame " << frame_count_ << std::endl;
                continue;
            }
            
            // NEW: Convert FABRIK result to robot capsules AND transform to STAR coordinates WITH OFFSET
            auto robot_capsules_star = RobotCollisionBridge::convert_basic_fabrik_to_capsules_star_coords(
                fabrik_result, robot_radius_, robot_offset_);
            if (!robot_capsules_star.success) {
                std::cerr << "⚠️  Robot→STAR transformation failed: " << robot_capsules_star.error_message << std::endl;
                continue;
            }
            
            // Generate animated STAR pose and get deformed mesh (STAR coordinates)
            auto star_result = get_animated_star_mesh(animation_time_);
            if (!star_result.success) {
                std::cerr << "⚠️  STAR mesh generation failed: " << star_result.error_message << std::endl;
                continue;
            }
            
            // Update current mesh data (STAR coordinates)
            current_vertices_ = star_result.vertices;
            current_joints_ = star_result.joints;
            
            // NEW: Collision detection with both robot and human in STAR coordinates!
            // No coordinate transformation needed - both are in Y-up meters
            auto collision_result = collision_engine_->detect_collisions(
                current_joints_, robot_capsules_star.data);
            
            // Publish complete frame via visualization system
            bool publish_success = publisher_.publish_collision_frame(
                robot_capsules_star.data, current_joints_, current_vertices_, 
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
    
    // Connect to STAR daemon and get initial mesh (STAR coordinates)
    bool connect_to_star_daemon() {
        std::cout << "🔄 Connecting to STAR daemon..." << std::endl;
        
        star_client_ = create_star_client();
        if (!star_client_) {
            std::cerr << "❌ Failed to connect to STAR daemon" << std::endl;
            std::cerr << "   Make sure daemon is running: python3 python/star_daemon.py" << std::endl;
            return false;
        }
        
        std::cout << "✅ Connected to STAR daemon" << std::endl;
        
        // Get initial T-pose mesh (STAR coordinates: Y-up, meters)
        auto initial_result = star_client_->get_neutral_pose();
        if (!initial_result.success) {
            std::cerr << "❌ Failed to get initial mesh: " << initial_result.error_message << std::endl;
            return false;
        }
        
        current_vertices_ = initial_result.vertices;
        current_joints_ = initial_result.joints;
        
        std::cout << "✅ Initial STAR mesh loaded: " << current_vertices_.size() 
                  << " vertices, " << current_joints_.size() << " joints (STAR coordinates)" << std::endl;
        
        return true;
    }
    
    Eigen::Vector3d calculate_target_position() {
        // Circular orbit around center point (robot coordinates: Z-up, mm)
        double angle = 2.0 * M_PI * orbit_speed_ * animation_time_;
        
        Eigen::Vector3d target;
        target.x() = orbit_center_.x() + orbit_radius_ * std::cos(angle);
        target.y() = orbit_center_.y() + orbit_radius_ * std::sin(angle);
        target.z() = orbit_center_.z() + 20.0 * std::sin(angle * 6.0); // Slight vertical motion
        
        return target;
    }
    
    FabrikSolverResult solve_fabrik_for_target(const Eigen::Vector3d& target) {
        // Use the basic FABRIK solver (robot coordinates: Z-up, mm)
        return FabrikSolverBlock::solve(target, num_segments_, FABRIK_TOLERANCE, FABRIK_MAX_ITERATIONS);
    }
    
    // Get animated STAR mesh using daemon (STAR coordinates)
    STARDaemonClient::MeshResult get_animated_star_mesh(double time) {
        // Generate pose parameters for animation
        auto pose_params = generate_animated_pose_params(time, 0.3);  // 30% amplitude for subtle movement
        
        // Request deformed mesh from daemon (returns STAR coordinates: Y-up, meters)
        return star_client_->get_deformed_mesh(pose_params);
    }
    
    void print_frame_statistics(const Eigen::Vector3d& target, 
                               const FabrikSolverResult& fabrik_result,
                               const CollisionResult& collision_result) {
        auto publisher_stats = publisher_.get_statistics();
        
        std::cout << "📊 Frame " << frame_count_ 
                  << " | FPS: " << std::fixed << std::setprecision(1) << publisher_stats.current_fps
                  << " | Target(robot): (" << std::setprecision(0) 
                  << target.x() << "," << target.y() << "," << target.z() << ")mm"
                  << " | FABRIK: " << fabrik_result.iterations_used << " iters"
                  << " | Collision: " << (collision_result.has_collision ? "YES" : "NO");
        
        if (collision_result.has_collision) {
            std::cout << " (" << collision_result.contacts.size() << " contacts)";
        }
        
        std::cout << " | Transform: Robot→STAR"
                  << " | Packets: " << publisher_stats.network_stats.packets_sent
                  << " | Success: " << std::setprecision(1) << publisher_stats.network_stats.success_rate << "%"
                  << " | STAR: " << current_vertices_.size() << " verts"
                  << " | Unified coords: Y-up meters"
                  << std::endl;
    }
};

int main() {
    std::cout << "🎯 MOVING TARGET COLLISION TEST (ROBOT→STAR TRANSFORM)" << std::endl;
    std::cout << "========================================================" << std::endl;
    std::cout << "Real-time collision detection with coordinate transformation optimization" << std::endl;
    std::cout << "Pipeline: FABRIK (robot coords) → Robot→STAR Transform → Unified Collision → Visualization" << std::endl;
    std::cout << "Optimization: Transform 7 robot capsules instead of 6890 STAR vertices" << std::endl;
    std::cout << "Collision Space: STAR coordinates (Y-up, meters)" << std::endl;
    std::cout << std::endl;
    
    MovingTargetTest test;
    
    if (!test.initialize()) {
        std::cerr << "❌ Failed to initialize moving target test" << std::endl;
        return 1;
    }
    
    std::cout << "📡 Streaming collision data (STAR coords) to UDP localhost:9999..." << std::endl;
    std::cout << "   Start receiver: ./build/test_visualization_receiver" << std::endl;
    std::cout << "   Performance: ~1000x fewer coordinate transformations per frame!" << std::endl;
    std::cout << std::endl;
    
    try {
        test.run_test_loop();
    } catch (const std::exception& e) {
        std::cerr << "❌ Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}