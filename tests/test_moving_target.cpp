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

// NEW: STAR daemon client (replaces vertex loader)
#include "../cpp/utils/star_daemon_client.hpp"

using namespace delta;

/**
 * Real Collision System Test with Moving Target (Updated with STAR Vertices)
 * Tests complete pipeline: Moving Target → FABRIK → Bridges → Collision Detection → Visualization
 */
class MovingTargetTest {
private:
    DataPublisher publisher_;
    std::unique_ptr<CollisionDetectionEngine> collision_engine_;
    std::unique_ptr<STARDaemonClient> star_client_;
    
    // Current mesh data (updated each frame)
    std::vector<Eigen::Vector3d> current_vertices_;
    std::vector<Eigen::Vector3d> current_joints_;
    
    int frame_count_;
    double animation_time_;
    
    // Animation parameters
    const double target_fps_ = 30.0;
    const double orbit_radius_ = 160.0;  // mm
    const double orbit_speed_ = 0.05;    // revolutions per second
    const Eigen::Vector3d orbit_center_{50.0, 30.0, 320.0}; // mm
    
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
        
        // NEW: Connect to STAR daemon
        if (!connect_to_star_daemon()) {
            std::cerr << "❌ Failed to connect to STAR daemon" << std::endl;
            return false;
        }
        
        // Initialize collision detection engine with initial vertices
        collision_engine_ = std::make_unique<CollisionDetectionEngine>();
        
        if (!collision_engine_->initialize("collision_data.h5", current_vertices_)) {
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
        std::cout << "   Human: Real STAR with animated mesh (" << current_vertices_.size() << " vertices)" << std::endl;
        std::cout << "   Animation: Subtle arm/leg movement for collision testing" << std::endl;
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
            
            // NEW: Generate animated STAR pose and get deformed mesh
            auto star_result = get_animated_star_mesh(animation_time_);
            if (!star_result.success) {
                std::cerr << "⚠️  STAR mesh generation failed: " << star_result.error_message << std::endl;
                continue;
            }
            
            // Update current mesh data
            current_vertices_ = star_result.vertices;
            current_joints_ = star_result.joints;
            
            // Transform human to collision coordinates
            auto star_bridge_result = STARCollisionBridge::transform_star_to_collision_coords(current_joints_);
            if (!star_bridge_result.success) {
                std::cerr << "⚠️  STAR bridge failed: " << star_bridge_result.error_message << std::endl;
                continue;
            }
            
            // Run collision detection with real STAR vertices
            auto collision_result = collision_engine_->detect_collisions(
                star_bridge_result.data, robot_bridge_result.data);
            
            // Publish complete frame via visualization system
            bool publish_success = publisher_.publish_collision_frame(
                robot_bridge_result.data, current_joints_, current_vertices_, 
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
    
    // NEW: Connect to STAR daemon and get initial mesh
    bool connect_to_star_daemon() {
        std::cout << "🔄 Connecting to STAR daemon..." << std::endl;
        
        star_client_ = create_star_client();
        if (!star_client_) {
            std::cerr << "❌ Failed to connect to STAR daemon" << std::endl;
            std::cerr << "   Make sure daemon is running: python3 python/star_daemon.py" << std::endl;
            return false;
        }
        
        std::cout << "✅ Connected to STAR daemon" << std::endl;
        
        // Get initial T-pose mesh
        auto initial_result = star_client_->get_neutral_pose();
        if (!initial_result.success) {
            std::cerr << "❌ Failed to get initial mesh: " << initial_result.error_message << std::endl;
            return false;
        }
        
        current_vertices_ = initial_result.vertices;
        current_joints_ = initial_result.joints;
        
        std::cout << "✅ Initial STAR mesh loaded: " << current_vertices_.size() 
                  << " vertices, " << current_joints_.size() << " joints" << std::endl;
        
        return true;
    }
    
    Eigen::Vector3d calculate_target_position() {
        // Circular orbit around center point
        double angle = 2.0 * M_PI * orbit_speed_ * animation_time_;
        
        Eigen::Vector3d target;
        target.x() = orbit_center_.x() + orbit_radius_ * std::cos(angle);
        target.y() = orbit_center_.y() + orbit_radius_ * std::sin(angle);
        target.z() = orbit_center_.z() + 20.0 * std::sin(angle * 6.0); // Slight vertical motion
        
        return target;
    }
    
    FabrikSolverResult solve_fabrik_for_target(const Eigen::Vector3d& target) {
        // Use the basic FABRIK solver (more likely to be implemented)
        return FabrikSolverBlock::solve(target, num_segments_, FABRIK_TOLERANCE, FABRIK_MAX_ITERATIONS);
    }
    
    // NEW: Get animated STAR mesh using daemon
    STARDaemonClient::MeshResult get_animated_star_mesh(double time) {
        // Generate pose parameters for animation
        auto pose_params = generate_animated_pose_params(time, 0.3);  // 30% amplitude for subtle movement
        
        // Request deformed mesh from daemon
        return star_client_->get_deformed_mesh(pose_params);
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
        
        std::cout                   << " | Packets: " << publisher_stats.network_stats.packets_sent
                  << " | Success: " << std::setprecision(1) << publisher_stats.network_stats.success_rate << "%"
                  << " | STAR: " << current_vertices_.size() << " verts"
                  << " | Animated Mesh"
                  << std::endl;
    }
};

int main() {
    std::cout << "🎯 MOVING TARGET COLLISION TEST (WITH ANIMATED STAR MESH)" << std::endl;
    std::cout << "==========================================================" << std::endl;
    std::cout << "Real-time collision detection with animated target + deformed human mesh" << std::endl;
    std::cout << "Pipeline: FABRIK → Bridges → STAR Daemon → Collision → Visualization" << std::endl;
    std::cout << "STAR Integration: Real-time mesh deformation via daemon" << std::endl;
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