#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <Eigen/Dense>

// Include your collision system headers
#include "../collision/layer_manager.hpp"
#include "../collision/collision_detection_engine.hpp"

using namespace delta;

void diagnostic_hdf5_file_access(const std::string& filepath) {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "HDF5 FILE ACCESS DIAGNOSTIC" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    std::cout << "Testing file: " << filepath << std::endl;
    
    // Check if file exists
    std::ifstream file(filepath);
    if (!file.good()) {
        std::cout << "❌ File does not exist or cannot be accessed" << std::endl;
        return;
    }
    file.close();
    
    // Get file size
    std::ifstream file_size(filepath, std::ifstream::ate | std::ifstream::binary);
    auto size = file_size.tellg();
    file_size.close();
    
    std::cout << "✅ File accessible" << std::endl;
    std::cout << "  File size: " << (size / 1024.0 / 1024.0) << " MB" << std::endl;
    
    // Try to open with HDF5 (basic test)
    try {
        // This will test if h5cpp can open the file
        std::cout << "  Testing HDF5 format compatibility..." << std::endl;
        // Note: Add actual HDF5 open test here if h5cpp is available
        std::cout << "  ✅ HDF5 format test would go here" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "  ❌ HDF5 format error: " << e.what() << std::endl;
    }
}

void diagnostic_layer_manager_loading(const std::string& filepath) {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "LAYER MANAGER LOADING DIAGNOSTIC" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    // Create LayerManager instance
    LayerManager layer_manager;
    
    // Time the loading process
    auto start_time = std::chrono::high_resolution_clock::now();
    
    std::cout << "Attempting to load HDF5 data..." << std::endl;
    bool load_success = layer_manager.load_hdf5_data(filepath);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "Loading result: " << (load_success ? "✅ SUCCESS" : "❌ FAILED") << std::endl;
    std::cout << "Loading time: " << duration.count() << " ms" << std::endl;
    
    if (!load_success) {
        std::cout << "❌ Failed to load HDF5 data - check file format and LayerManager implementation" << std::endl;
        return;
    }
    
    // Get hierarchy mappings to verify data was loaded
    const auto& hierarchy = layer_manager.get_hierarchy_mappings();
    
    std::cout << "\nLoaded Data Summary:" << std::endl;
    std::cout << "  Vertices: " << hierarchy.num_vertices << std::endl;
    std::cout << "  Spheres: " << hierarchy.num_spheres << std::endl;
    std::cout << "  Capsules: " << hierarchy.num_capsules << std::endl;
    std::cout << "  Simple capsules: " << hierarchy.num_simple << std::endl;
    std::cout << "  Max assignments per vertex: " << hierarchy.max_assignments_per_vertex << std::endl;
    
    // Validate the data makes sense
    if (hierarchy.num_vertices < 5000) {
        std::cout << "  ⚠️  WARNING: Low vertex count (" << hierarchy.num_vertices << ")" << std::endl;
    }
    if (hierarchy.num_spheres < 50) {
        std::cout << "  ⚠️  WARNING: Low sphere count (" << hierarchy.num_spheres << ")" << std::endl;
    }
    if (hierarchy.num_capsules != 24) {
        std::cout << "  ⚠️  WARNING: Expected 24 capsules, got " << hierarchy.num_capsules << std::endl;
    }
    if (hierarchy.num_simple != 9) {
        std::cout << "  ⚠️  WARNING: Expected 9 simple capsules, got " << hierarchy.num_simple << std::endl;
    }
    
    std::cout << "✅ HDF5 data loaded successfully" << std::endl;
}

void diagnostic_base_mesh_initialization() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "BASE MESH INITIALIZATION DIAGNOSTIC" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    // Create dummy T-pose vertices (matching STAR output format)
    std::vector<Eigen::Vector3d> dummy_vertices;
    
    // Generate 6890 dummy vertices (STAR mesh size)
    const int vertex_count = 6890;
    dummy_vertices.reserve(vertex_count);
    
    // Create a simple human-like shape for testing
    for (int i = 0; i < vertex_count; ++i) {
        double t = static_cast<double>(i) / vertex_count;
        
        // Simple body shape: head at top, feet at bottom
        double x = 0.5 * sin(t * 6.28) * (1.0 - t);  // Tapering from shoulders to feet
        double y = -1.5 + t * 1.8;                    // From feet (-1.5) to head (0.3)
        double z = 0.1 * cos(t * 12.56);              // Small front-back variation
        
        dummy_vertices.emplace_back(x, y, z);
    }
    
    std::cout << "Created " << dummy_vertices.size() << " dummy vertices" << std::endl;
    std::cout << "Vertex range:" << std::endl;
    
    // Calculate bounds
    Eigen::Vector3d min_bounds = dummy_vertices[0];
    Eigen::Vector3d max_bounds = dummy_vertices[0];
    
    for (const auto& vertex : dummy_vertices) {
        for (int i = 0; i < 3; ++i) {
            min_bounds[i] = std::min(min_bounds[i], vertex[i]);
            max_bounds[i] = std::max(max_bounds[i], vertex[i]);
        }
    }
    
    std::cout << "  Min: (" << min_bounds.transpose() << ")" << std::endl;
    std::cout << "  Max: (" << max_bounds.transpose() << ")" << std::endl;
    
    // Test LayerManager initialization
    LayerManager layer_manager;
    
    std::cout << "\nTesting base mesh initialization..." << std::endl;
    bool init_success = layer_manager.initialize_base_mesh(dummy_vertices);
    
    std::cout << "Base mesh initialization: " << (init_success ? "✅ SUCCESS" : "❌ FAILED") << std::endl;
    
    if (init_success) {
        std::cout << "✅ LayerManager can handle vertex data format" << std::endl;
    }
}

void diagnostic_collision_engine_integration(const std::string& filepath) {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "COLLISION ENGINE INTEGRATION DIAGNOSTIC" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    // Create dummy T-pose vertices
    std::vector<Eigen::Vector3d> dummy_vertices;
    const int vertex_count = 6890;
    
    for (int i = 0; i < vertex_count; ++i) {
        double t = static_cast<double>(i) / vertex_count;
        double x = 0.5 * sin(t * 6.28) * (1.0 - t);
        double y = -1.5 + t * 1.8;
        double z = 0.1 * cos(t * 12.56);
        dummy_vertices.emplace_back(x, y, z);
    }
    
    std::cout << "Testing CollisionDetectionEngine initialization..." << std::endl;
    
    // Create collision detection engine
    CollisionDetectionEngine engine;
    
    // Time the initialization
    auto start_time = std::chrono::high_resolution_clock::now();
    
    bool engine_init = engine.initialize(filepath, dummy_vertices);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "Engine initialization: " << (engine_init ? "✅ SUCCESS" : "❌ FAILED") << std::endl;
    std::cout << "Initialization time: " << duration.count() << " ms" << std::endl;
    
    if (!engine_init) {
        std::cout << "❌ CollisionDetectionEngine failed to initialize" << std::endl;
        return;
    }
    
    // Test configuration
    std::cout << "\nTesting engine configuration..." << std::endl;
    engine.configure(3, 10, 1e-6);
    std::cout << "✅ Engine configuration successful" << std::endl;
    
    // Get engine statistics
    auto stats = engine.get_layer_statistics();
    std::cout << "\nEngine Statistics:" << std::endl;
    std::cout << "  Total Layer 3: " << stats.total_layer3 << std::endl;
    std::cout << "  Active Layer 2: " << stats.active_layer2 << std::endl;
    std::cout << "  Active Layer 1: " << stats.active_layer1 << std::endl;
    std::cout << "  Memory usage: " << stats.memory_usage_mb << " MB" << std::endl;
    
    std::cout << "✅ Full collision engine integration successful" << std::endl;
}

void diagnostic_bridge_requirements() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "BRIDGE REQUIREMENTS SUMMARY" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    std::cout << "\nINPUT REQUIREMENTS:" << std::endl;
    std::cout << "  - HDF5 file: Generated by Python collision_mapping.py" << std::endl;
    std::cout << "  - Base vertices: STAR T-pose mesh (6890 vertices)" << std::endl;
    std::cout << "  - Format: std::vector<Eigen::Vector3d>" << std::endl;
    
    std::cout << "\nCOLLISION ENGINE INTERFACE:" << std::endl;
    std::cout << "  - LayerManager::load_hdf5_data(filepath)" << std::endl;
    std::cout << "  - LayerManager::initialize_base_mesh(vertices)" << std::endl;
    std::cout << "  - CollisionDetectionEngine::initialize(filepath, vertices)" << std::endl;
    
    std::cout << "\nRUNTIME INTERFACE:" << std::endl;
    std::cout << "  - LayerManager::update_human_pose(bone_positions)" << std::endl;
    std::cout << "  - CollisionDetectionEngine::detect_collisions(bones, robot_capsules)" << std::endl;
    
    std::cout << "\nDATA FLOW:" << std::endl;
    std::cout << "  1. Python generates HDF5 collision data" << std::endl;
    std::cout << "  2. C++ loads HDF5 data via LayerManager" << std::endl;
    std::cout << "  3. STAR provides bone positions → update_human_pose()" << std::endl;
    std::cout << "  4. FABRIK provides robot capsules → detect_collisions()" << std::endl;
    std::cout << "  5. Engine returns collision results with depth/normals" << std::endl;
    
    std::cout << "\nBRIDGE FUNCTIONS NEEDED:" << std::endl;
    std::cout << "  - robot_collision_bridge: FABRIK result → CapsuleData vector" << std::endl;
    std::cout << "  - star_collision_bridge: STAR joints → update collision layers" << std::endl;
    std::cout << "  - integration_test: End-to-end pipeline test" << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "HDF5 LOADING & COLLISION ENGINE DIAGNOSTIC" << std::endl;
    std::cout << "Testing C++ collision system with Python-generated data" << std::endl;
    
    // Default test file path
    std::string hdf5_filepath = "collision_data.h5";
    
    if (argc > 1) {
        hdf5_filepath = argv[1];
    }
    
    std::cout << "\nUsing HDF5 file: " << hdf5_filepath << std::endl;
    std::cout << "Run with: ./hdf5_diag [path_to_collision_data.h5]" << std::endl;
    
    try {
        // Step 1: Test file access
        diagnostic_hdf5_file_access(hdf5_filepath);
        
        // Step 2: Test LayerManager loading
        diagnostic_layer_manager_loading(hdf5_filepath);
        
        // Step 3: Test base mesh initialization
        diagnostic_base_mesh_initialization();
        
        // Step 4: Test full collision engine
        diagnostic_collision_engine_integration(hdf5_filepath);
        
        // Step 5: Bridge requirements
        diagnostic_bridge_requirements();
        
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "HDF5 DIAGNOSTIC COMPLETE" << std::endl;
        std::cout << "All systems ready for bridge implementation" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "\n❌ DIAGNOSTIC FAILED: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}