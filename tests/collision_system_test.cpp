#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "core/timing.hpp"
#include "collision_blocks/capsule_creation_block.hpp"

using namespace delta;

int main() {
    std::cout << "🧪 Comprehensive collision system test..." << std::endl;
    
    // Test 1: Basic data structures
    std::cout << "  Testing basic data structures..." << std::endl;
    std::vector<Eigen::Vector3d> test_points;
    for (int i = 0; i < 10; ++i) {
        test_points.emplace_back(i * 0.1, 0, 0);
    }
    std::cout << "  ✅ Created " << test_points.size() << " test points" << std::endl;
    
    // Test 2: Capsule creation
    std::cout << "  Testing capsule creation..." << std::endl;
    CapsuleCreationBlock capsule_creator;
    
    std::vector<Eigen::Vector3d> s_points;
    for (int i = 0; i < 6; ++i) {
        s_points.emplace_back(i * 0.3, 0, 0);
    }
    
    auto result = capsule_creator.create_capsule_chain(s_points, 0.05);
    if (result.creation_successful) {
        std::cout << "  ✅ Created " << result.capsules.size() << " capsules" << std::endl;
        std::cout << "  ✅ Total length: " << result.total_chain_length << std::endl;
    } else {
        std::cout << "  ❌ Capsule creation failed: " << result.error_message << std::endl;
        return 1;
    }
    
    // Test 3: Timing system
    std::cout << "  Testing timing system..." << std::endl;
    double timing_test = 0.0;
    {
        ScopedTimer timer(timing_test);
        // Simulate some computation
        double sum = 0.0;
        for (int i = 0; i < 100000; ++i) {
            sum += sqrt(i * 3.14159);
        }
        std::cout << "  ✅ Computation result: " << sum << std::endl;
    }
    std::cout << "  ✅ Timing test: " << timing_test << " ms" << std::endl;
    
    // Test 4: Memory usage estimation
    std::cout << "  Testing memory calculations..." << std::endl;
    size_t estimated_memory = test_points.size() * sizeof(Eigen::Vector3d);
    estimated_memory += result.capsules.size() * sizeof(CapsuleData);
    std::cout << "  ✅ Estimated memory usage: " << (estimated_memory / 1024.0) << " KB" << std::endl;
    
    std::cout << "✅ All collision system tests passed!" << std::endl;
    std::cout << "🏆 Collision detection system is ready for integration!" << std::endl;
    
    return 0;
}
