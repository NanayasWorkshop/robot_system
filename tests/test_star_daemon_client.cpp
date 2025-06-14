#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <limits>
#include "../cpp/utils/star_daemon_client.hpp"

using namespace delta;

/**
 * STAR Daemon Client Test
 * Tests C++ client communication with Python STAR daemon
 */
class STARDaemonClientTest {
private:
    std::unique_ptr<STARDaemonClient> client_;
    
public:
    STARDaemonClientTest() = default;
    
    bool run_tests() {
        std::cout << "🧪 STAR DAEMON CLIENT TEST" << std::endl;
        std::cout << "=" << std::string(50, '=') << std::endl;
        
        // Test 1: Connection
        if (!test_connection()) {
            return false;
        }
        
        // Test 2: Neutral pose
        if (!test_neutral_pose()) {
            return false;
        }
        
        // Test 3: Animated poses
        if (!test_animated_poses()) {
            return false;
        }
        
        // Test 4: Performance
        if (!test_performance()) {
            return false;
        }
        
        // Test 5: Error handling
        if (!test_error_handling()) {
            return false;
        }
        
        std::cout << "\n" << std::string(50, '=') << std::endl;
        std::cout << "🎉 ALL TESTS PASSED!" << std::endl;
        print_final_statistics();
        
        return true;
    }
    
private:
    
    bool test_connection() {
        std::cout << "\n1. Testing connection..." << std::endl;
        
        client_ = create_star_client();
        if (!client_) {
            std::cout << "   ❌ Failed to create and connect client" << std::endl;
            std::cout << "   Make sure STAR daemon is running: python3 python/star_daemon.py" << std::endl;
            return false;
        }
        
        std::cout << "   ✅ Connected successfully" << std::endl;
        
        // Test ping
        if (!client_->ping_daemon(1000)) {
            std::cout << "   ❌ Daemon ping failed" << std::endl;
            return false;
        }
        
        std::cout << "   ✅ Daemon ping successful" << std::endl;
        return true;
    }
    
    bool test_neutral_pose() {
        std::cout << "\n2. Testing neutral pose..." << std::endl;
        
        auto result = client_->get_neutral_pose();
        
        if (!result.success) {
            std::cout << "   ❌ Neutral pose failed: " << result.error_message << std::endl;
            return false;
        }
        
        // Validate data
        if (result.vertices.size() != 6890) {
            std::cout << "   ❌ Wrong vertex count: " << result.vertices.size() << " (expected 6890)" << std::endl;
            return false;
        }
        
        if (result.joints.size() != 24) {
            std::cout << "   ❌ Wrong joint count: " << result.joints.size() << " (expected 24)" << std::endl;
            return false;
        }
        
        // Calculate bounds
        auto vertex_bounds = calculate_bounds(result.vertices);
        auto joint_bounds = calculate_bounds(result.joints);
        
        std::cout << "   ✅ Success: " << result.vertices.size() << " vertices, " 
                  << result.joints.size() << " joints" << std::endl;
        std::cout << "   ⏱️  Computation time: " << std::fixed << std::setprecision(1) 
                  << result.computation_time_ms << "ms" << std::endl;
        std::cout << "   📐 Vertex bounds: X[" << std::setprecision(3) 
                  << vertex_bounds.min_x << ", " << vertex_bounds.max_x << "] "
                  << "Y[" << vertex_bounds.min_y << ", " << vertex_bounds.max_y << "] "
                  << "Z[" << vertex_bounds.min_z << ", " << vertex_bounds.max_z << "]" << std::endl;
        
        return true;
    }
    
    bool test_animated_poses() {
        std::cout << "\n3. Testing animated poses..." << std::endl;
        
        // Get neutral pose for comparison
        auto neutral_result = client_->get_neutral_pose();
        if (!neutral_result.success) {
            std::cout << "   ❌ Failed to get neutral pose for comparison" << std::endl;
            return false;
        }
        
        // Test 1: Spine bend
        std::cout << "   Testing spine bend..." << std::endl;
        auto spine_params = generate_animated_pose_params(0.25, 0.5);  // 1/4 cycle, 50% amplitude
        auto spine_result = client_->get_deformed_mesh(spine_params);
        
        if (!spine_result.success) {
            std::cout << "   ❌ Spine bend failed: " << spine_result.error_message << std::endl;
            return false;
        }
        
        double vertex_movement = calculate_average_movement(neutral_result.vertices, spine_result.vertices);
        std::cout << "     ✅ Spine bend: " << std::setprecision(4) << vertex_movement 
                  << "m avg vertex movement" << std::endl;
        
        if (vertex_movement < 0.001) {
            std::cout << "     ⚠️  Warning: Very small movement detected" << std::endl;
        }
        
        // Test 2: Arm movement
        std::cout << "   Testing arm movement..." << std::endl;
        auto arm_params = generate_animated_pose_params(0.5, 0.3);  // 1/2 cycle, 30% amplitude
        auto arm_result = client_->get_deformed_mesh(arm_params);
        
        if (!arm_result.success) {
            std::cout << "   ❌ Arm movement failed: " << arm_result.error_message << std::endl;
            return false;
        }
        
        double arm_movement = calculate_average_movement(neutral_result.vertices, arm_result.vertices);
        std::cout << "     ✅ Arm movement: " << std::setprecision(4) << arm_movement 
                  << "m avg vertex movement" << std::endl;
        
        return true;
    }
    
    bool test_performance() {
        std::cout << "\n4. Testing performance..." << std::endl;
        
        const int num_requests = 20;
        std::vector<double> times;
        times.reserve(num_requests);
        
        std::cout << "   Running " << num_requests << " requests..." << std::endl;
        
        for (int i = 0; i < num_requests; ++i) {
            // Generate slightly different poses
            double time_offset = static_cast<double>(i) / num_requests;
            auto pose_params = generate_animated_pose_params(time_offset, 0.2);
            
            auto result = client_->get_deformed_mesh(pose_params);
            
            if (!result.success) {
                std::cout << "   ❌ Request " << (i + 1) << " failed: " << result.error_message << std::endl;
                return false;
            }
            
            times.push_back(result.computation_time_ms);
            
            if ((i + 1) % 5 == 0) {
                std::cout << "     Request " << (i + 1) << "/" << num_requests 
                          << ": " << std::setprecision(1) << result.computation_time_ms << "ms" << std::endl;
            }
        }
        
        // Calculate statistics
        double total_time = 0.0;
        double min_time = times[0];
        double max_time = times[0];
        
        for (double time : times) {
            total_time += time;
            min_time = std::min(min_time, time);
            max_time = std::max(max_time, time);
        }
        
        double avg_time = total_time / times.size();
        
        std::cout << "   ✅ Performance results:" << std::endl;
        std::cout << "     Average: " << std::setprecision(1) << avg_time << "ms" << std::endl;
        std::cout << "     Range: " << min_time << "ms - " << max_time << "ms" << std::endl;
        std::cout << "     Total: " << total_time << "ms for " << num_requests << " requests" << std::endl;
        
        if (avg_time < 10.0) {
            std::cout << "     ✅ Excellent performance (<10ms average)" << std::endl;
        } else if (avg_time < 20.0) {
            std::cout << "     ✅ Good performance (<20ms average)" << std::endl;
        } else {
            std::cout << "     ⚠️  Slow performance (>20ms average)" << std::endl;
        }
        
        return true;
    }
    
    bool test_error_handling() {
        std::cout << "\n5. Testing error handling..." << std::endl;
        
        // Test invalid pose parameters
        std::cout << "   Testing invalid pose size..." << std::endl;
        std::vector<float> invalid_pose(50, 0.0f);  // Wrong size
        
        auto result = client_->get_deformed_mesh(invalid_pose);
        
        if (result.success) {
            std::cout << "   ⚠️  Warning: Invalid pose was accepted (should have failed)" << std::endl;
        } else {
            std::cout << "   ✅ Invalid pose correctly rejected: " << result.error_message << std::endl;
        }
        
        // Test with NaN values
        std::cout << "   Testing NaN values..." << std::endl;
        std::vector<float> nan_pose(72, 0.0f);
        nan_pose[0] = std::numeric_limits<float>::quiet_NaN();
        
        auto nan_result = client_->get_deformed_mesh(nan_pose);
        
        if (nan_result.success) {
            std::cout << "   ⚠️  Warning: NaN pose was accepted (should have failed)" << std::endl;
        } else {
            std::cout << "   ✅ NaN pose correctly rejected: " << nan_result.error_message << std::endl;
        }
        
        return true;
    }
    
    struct Bounds {
        double min_x, max_x, min_y, max_y, min_z, max_z;
    };
    
    Bounds calculate_bounds(const std::vector<Eigen::Vector3d>& points) {
        Bounds bounds{};
        
        if (points.empty()) {
            return bounds;
        }
        
        bounds.min_x = bounds.max_x = points[0].x();
        bounds.min_y = bounds.max_y = points[0].y();
        bounds.min_z = bounds.max_z = points[0].z();
        
        for (const auto& point : points) {
            bounds.min_x = std::min(bounds.min_x, point.x());
            bounds.max_x = std::max(bounds.max_x, point.x());
            bounds.min_y = std::min(bounds.min_y, point.y());
            bounds.max_y = std::max(bounds.max_y, point.y());
            bounds.min_z = std::min(bounds.min_z, point.z());
            bounds.max_z = std::max(bounds.max_z, point.z());
        }
        
        return bounds;
    }
    
    double calculate_average_movement(const std::vector<Eigen::Vector3d>& points1,
                                    const std::vector<Eigen::Vector3d>& points2) {
        if (points1.size() != points2.size()) {
            return 0.0;
        }
        
        double total_movement = 0.0;
        
        for (size_t i = 0; i < points1.size(); ++i) {
            double distance = (points2[i] - points1[i]).norm();
            total_movement += distance;
        }
        
        return total_movement / points1.size();
    }
    
    void print_final_statistics() {
        if (!client_) {
            return;
        }
        
        auto stats = client_->get_statistics();
        
        std::cout << "\n📊 Final Statistics:" << std::endl;
        std::cout << "  Total requests: " << stats.total_requests << std::endl;
        std::cout << "  Successful: " << stats.successful_requests << std::endl;
        std::cout << "  Failed: " << stats.failed_requests << std::endl;
        std::cout << "  Success rate: " << std::setprecision(1) << stats.success_rate_percent << "%" << std::endl;
        std::cout << "  Average time: " << std::setprecision(1) << stats.avg_request_time_ms << "ms" << std::endl;
        std::cout << "  Last request: " << stats.last_request_time_ms << "ms" << std::endl;
    }
};

int main() {
    std::cout << "🎯 STAR DAEMON CLIENT TEST" << std::endl;
    std::cout << "Tests C++ client communication with Python STAR daemon" << std::endl;
    std::cout << "Make sure STAR daemon is running: python3 python/star_daemon.py" << std::endl;
    std::cout << std::endl;
    
    STARDaemonClientTest test;
    
    bool success = test.run_tests();
    
    if (!success) {
        std::cout << "\n❌ Tests failed!" << std::endl;
        std::cout << "Check that:" << std::endl;
        std::cout << "  1. STAR daemon is running: python3 python/star_daemon.py" << std::endl;
        std::cout << "  2. Socket is accessible: /tmp/star_daemon.sock" << std::endl;
        std::cout << "  3. No firewall blocking Unix sockets" << std::endl;
        return 1;
    }
    
    return 0;
}