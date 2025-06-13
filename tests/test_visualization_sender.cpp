#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <Eigen/Dense>

#include "../cpp/visualization/data_publisher.hpp"
#include "../cpp/collision_blocks/capsule_creation_block.hpp"
#include "../cpp/collision/collision_types.hpp"

using namespace delta;

/**
 * Test Visualization Sender
 * Creates fake collision data and streams it via DataPublisher
 */
class TestVisualizationSender {
private:
    DataPublisher publisher_;
    int frame_count_;
    double animation_time_;
    
public:
    TestVisualizationSender() : frame_count_(0), animation_time_(0.0) {}
    
    bool initialize() {
        std::cout << "🚀 Initializing Test Visualization Sender..." << std::endl;
        
        if (!publisher_.initialize("127.0.0.1", 9999, 30.0)) {
            std::cerr << "❌ Failed to initialize DataPublisher" << std::endl;
            return false;
        }
        
        std::cout << "✅ DataPublisher initialized (30 FPS → localhost:9999)" << std::endl;
        return true;
    }
    
    void run_test_loop() {
        std::cout << "🎬 Starting test animation loop..." << std::endl;
        std::cout << "   Press Ctrl+C to stop" << std::endl;
        
        const double target_fps = 30.0;
        const auto frame_duration = std::chrono::microseconds(static_cast<int>(1000000.0 / target_fps));
        
        while (true) {
            auto frame_start = std::chrono::steady_clock::now();
            
            // Generate frame data
            auto robot_capsules = create_test_robot_capsules();
            auto human_joints = create_test_human_pose();
            auto human_vertices = create_test_human_mesh();
            auto collision_result = create_test_collision_result();
            
            // Publish complete frame
            double computation_time = 0.5; // Fake computation time
            bool success = publisher_.publish_collision_frame(
                robot_capsules, human_joints, human_vertices, collision_result, computation_time);
            
            if (!success) {
                std::cerr << "❌ Failed to publish frame " << frame_count_ << std::endl;
            } else if (frame_count_ % 30 == 0) {
                // Print stats every second
                auto stats = publisher_.get_statistics();
                std::cout << "📊 Frame " << frame_count_ 
                          << " | FPS: " << std::fixed << std::setprecision(1) << stats.current_fps
                          << " | Packets: " << stats.network_stats.packets_sent 
                          << " | Success: " << std::setprecision(1) << stats.network_stats.success_rate << "%"
                          << std::endl;
            }
            
            frame_count_++;
            animation_time_ += 1.0 / target_fps;
            
            // Frame rate control
            auto frame_end = std::chrono::steady_clock::now();
            auto frame_time = frame_end - frame_start;
            if (frame_time < frame_duration) {
                std::this_thread::sleep_for(frame_duration - frame_time);
            }
        }
    }
    
private:
    
    std::vector<CapsuleData> create_test_robot_capsules() {
        std::vector<CapsuleData> capsules;
        
        // Create a 7-segment robot arm that moves in a circle
        const int num_segments = 7;
        const double segment_length = 50.0; // mm
        const double robot_radius = 24.8;   // From constants
        
        // Animate the target position in a circle
        double target_x = 100.0 * std::cos(animation_time_ * 0.5);
        double target_y = 100.0 * std::sin(animation_time_ * 0.5);
        double target_z = 150.0 + 50.0 * std::sin(animation_time_ * 1.0);
        
        // Create simple joint chain pointing toward target
        std::vector<Eigen::Vector3d> joints;
        joints.push_back(Eigen::Vector3d(0, 0, 0)); // Base
        
        for (int i = 1; i <= num_segments; ++i) {
            double progress = static_cast<double>(i) / num_segments;
            Eigen::Vector3d joint(
                target_x * progress,
                target_y * progress,
                target_z * progress
            );
            joints.push_back(joint);
        }
        
        // Convert joints to capsules
        for (size_t i = 0; i < joints.size() - 1; ++i) {
            CapsuleData capsule;
            capsule.start_point = joints[i];
            capsule.end_point = joints[i + 1];
            capsule.radius = robot_radius;
            capsule.length = (capsule.end_point - capsule.start_point).norm();
            capsules.push_back(capsule);
        }
        
        return capsules;
    }
    
    std::vector<Eigen::Vector3d> create_test_human_pose() {
        std::vector<Eigen::Vector3d> joints;
        
        // Create 24 STAR joints in T-pose with slight animation
        const double sway = 10.0 * std::sin(animation_time_ * 2.0); // Small sway animation
        
        // STAR joint hierarchy (simplified T-pose)
        joints.push_back(Eigen::Vector3d(0, 0, 0));           // 0: pelvis
        joints.push_back(Eigen::Vector3d(0, 0, 200));         // 1: spine
        joints.push_back(Eigen::Vector3d(0, 0, 400));         // 2: spine1
        joints.push_back(Eigen::Vector3d(0, 0, 600));         // 3: spine2
        joints.push_back(Eigen::Vector3d(0, 0, 800));         // 4: neck
        joints.push_back(Eigen::Vector3d(sway, 0, 950));      // 5: head
        
        // Left arm
        joints.push_back(Eigen::Vector3d(-150, 0, 750));      // 6: left_shoulder
        joints.push_back(Eigen::Vector3d(-300, 0, 750));      // 7: left_arm
        joints.push_back(Eigen::Vector3d(-450, 0, 750));      // 8: left_forearm
        joints.push_back(Eigen::Vector3d(-600, 0, 750));      // 9: left_hand
        
        // Right arm  
        joints.push_back(Eigen::Vector3d(150, 0, 750));       // 10: right_shoulder
        joints.push_back(Eigen::Vector3d(300, 0, 750));       // 11: right_arm
        joints.push_back(Eigen::Vector3d(450, 0, 750));       // 12: right_forearm
        joints.push_back(Eigen::Vector3d(600, 0, 750));       // 13: right_hand
        
        // Left leg
        joints.push_back(Eigen::Vector3d(-100, 0, 0));        // 14: left_hip
        joints.push_back(Eigen::Vector3d(-100, 0, -400));     // 15: left_thigh
        joints.push_back(Eigen::Vector3d(-100, 0, -800));     // 16: left_shin
        joints.push_back(Eigen::Vector3d(-100, 50, -1000));   // 17: left_foot
        
        // Right leg
        joints.push_back(Eigen::Vector3d(100, 0, 0));         // 18: right_hip
        joints.push_back(Eigen::Vector3d(100, 0, -400));      // 19: right_thigh
        joints.push_back(Eigen::Vector3d(100, 0, -800));      // 20: right_shin
        joints.push_back(Eigen::Vector3d(100, 50, -1000));    // 21: right_foot
        
        // Additional joints for STAR (24 total)
        joints.push_back(Eigen::Vector3d(-50, 0, 200));       // 22: spine_detail1
        joints.push_back(Eigen::Vector3d(50, 0, 200));        // 23: spine_detail2
        
        return joints;
    }
    
    std::vector<Eigen::Vector3d> create_test_human_mesh() {
        std::vector<Eigen::Vector3d> vertices;
        
        // Create a simple human-like mesh (very basic for testing)
        // Just a few hundred vertices around the skeleton
        
        const int vertices_per_joint = 8;
        auto joints = create_test_human_pose();
        
        for (const auto& joint : joints) {
            // Create vertices around each joint
            for (int i = 0; i < vertices_per_joint; ++i) {
                double angle = 2.0 * M_PI * i / vertices_per_joint;
                double radius = 30.0 + 10.0 * std::sin(animation_time_ * 3.0); // Slight breathing animation
                
                Eigen::Vector3d vertex(
                    joint.x() + radius * std::cos(angle),
                    joint.y() + radius * std::sin(angle),
                    joint.z() + 10.0 * std::sin(angle * 2.0)
                );
                vertices.push_back(vertex);
            }
        }
        
        return vertices;
    }
    
    CollisionResult create_test_collision_result() {
        CollisionResult result;
        
        // Simulate collision detection
        bool should_have_collision = std::sin(animation_time_ * 1.5) > 0.3;
        result.has_collision = should_have_collision;
        result.computation_time_ms = 0.5 + 0.2 * std::sin(animation_time_ * 4.0);
        
        if (should_have_collision) {
            // Create some test collision contacts
            CollisionContact contact1;
            contact1.contact_point = Eigen::Vector3d(
                50.0 + 20.0 * std::cos(animation_time_),
                20.0 + 10.0 * std::sin(animation_time_),
                100.0
            );
            contact1.surface_normal = Eigen::Vector3d(1, 0, 0);
            contact1.penetration_depth = 5.0 + 2.0 * std::sin(animation_time_ * 2.0);
            contact1.robot_capsule_index = 3;
            
            CollisionContact contact2;
            contact2.contact_point = Eigen::Vector3d(-30.0, 40.0, 120.0);
            contact2.surface_normal = Eigen::Vector3d(0, 1, 0);
            contact2.penetration_depth = 2.5;
            contact2.robot_capsule_index = 5;
            
            result.contacts.push_back(contact1);
            result.contacts.push_back(contact2);
            result.max_penetration_depth = std::max(contact1.penetration_depth, contact2.penetration_depth);
        }
        
        // Add some test statistics
        result.layer3_tests = 9;
        result.layer2_activations = should_have_collision ? 15 : 3;
        result.layer1_activations = should_have_collision ? 25 : 5;
        result.layer0_activations = should_have_collision ? 8 : 0;
        
        return result;
    }
};

int main() {
    std::cout << "🧪 TEST VISUALIZATION SENDER" << std::endl;
    std::cout << "============================" << std::endl;
    
    TestVisualizationSender sender;
    
    if (!sender.initialize()) {
        std::cerr << "❌ Failed to initialize sender" << std::endl;
        return 1;
    }
    
    std::cout << "📡 Streaming test data to UDP localhost:9999..." << std::endl;
    std::cout << "   Robot: Animated 7-segment arm moving in circle" << std::endl;
    std::cout << "   Human: T-pose with 24 joints + mesh vertices" << std::endl;
    std::cout << "   Collisions: Simulated contacts (animated)" << std::endl;
    std::cout << std::endl;
    
    try {
        sender.run_test_loop();
    } catch (const std::exception& e) {
        std::cerr << "❌ Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}