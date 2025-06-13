#include "vertex_loader.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <sys/stat.h>

namespace delta {

// =============================================================================
// MAIN LOADING FUNCTIONS
// =============================================================================

std::vector<Eigen::Vector3d> VertexLoader::load_vertices_from_binary(const std::string& filepath) {
    std::cout << "🔄 Loading vertices from binary file: " << filepath << std::endl;
    
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "❌ Could not open binary file: " << filepath << std::endl;
        return {};
    }
    
    // Read header
    uint32_t vertex_count;
    if (!read_binary_header(file, vertex_count)) {
        std::cerr << "❌ Failed to read binary file header" << std::endl;
        return {};
    }
    
    std::cout << "   Vertex count in file: " << vertex_count << std::endl;
    
    // Read vertex data
    auto vertices = read_binary_vertices(file, vertex_count);
    
    if (vertices.empty()) {
        std::cerr << "❌ Failed to read vertex data" << std::endl;
        return {};
    }
    
    std::cout << "✅ Successfully loaded " << vertices.size() << " vertices from binary file" << std::endl;
    return vertices;
}

std::vector<Eigen::Vector3d> VertexLoader::load_vertices_from_text(const std::string& filepath) {
    std::cout << "🔄 Loading vertices from text file: " << filepath << std::endl;
    
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "❌ Could not open text file: " << filepath << std::endl;
        return {};
    }
    
    std::vector<Eigen::Vector3d> vertices;
    std::string line;
    size_t line_number = 0;
    
    while (std::getline(file, line)) {
        line_number++;
        
        // Skip comments and empty lines
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        // Parse vertex coordinates
        std::istringstream iss(line);
        double x, y, z;
        
        if (!(iss >> x >> y >> z)) {
            std::cerr << "⚠️  Warning: Could not parse line " << line_number << ": " << line << std::endl;
            continue;
        }
        
        vertices.emplace_back(x, y, z);
        
        // Progress reporting for large files
        if (vertices.size() % VertexLoaderConstants::PROGRESS_UPDATE_INTERVAL == 0) {
            print_progress(vertices.size(), VertexLoaderConstants::STAR_VERTEX_COUNT, "Loading");
        }
    }
    
    std::cout << "✅ Successfully loaded " << vertices.size() << " vertices from text file" << std::endl;
    return vertices;
}

bool VertexLoader::generate_star_vertices(const std::string& python_script_path,
                                         const std::string& output_binary_path) {
    std::cout << "🔄 Generating STAR vertices using Python script..." << std::endl;
    std::cout << "   Script: " << python_script_path << std::endl;
    std::cout << "   Output: " << output_binary_path << std::endl;
    
    // Check if Python script exists
    if (!file_exists_and_readable(python_script_path)) {
        std::cerr << "❌ Python script not found: " << python_script_path << std::endl;
        return false;
    }
    
    // Build command
    std::string command = "python3 " + python_script_path;
    
    std::cout << "   Executing: " << command << std::endl;
    std::cout << "   This may take a few seconds..." << std::endl;
    
    // Execute Python script
    bool success = execute_command(command);
    
    if (!success) {
        std::cerr << "❌ Python script execution failed" << std::endl;
        std::cerr << "   Make sure STAR model is installed and configured correctly" << std::endl;
        return false;
    }
    
    // Check if output file was created
    if (!file_exists_and_readable(output_binary_path)) {
        std::cerr << "❌ Output file not created: " << output_binary_path << std::endl;
        return false;
    }
    
    std::cout << "✅ STAR vertex generation successful" << std::endl;
    return true;
}

std::vector<Eigen::Vector3d> VertexLoader::load_or_generate_star_vertices(
    const std::string& binary_filepath,
    const std::string& python_script_path) {
    
    std::cout << "🔄 Loading or generating STAR vertices..." << std::endl;
    
    // Try to load existing binary file first
    if (file_exists_and_readable(binary_filepath)) {
        std::cout << "   Found existing binary file: " << binary_filepath << std::endl;
        auto vertices = load_vertices_from_binary(binary_filepath);
        
        if (!vertices.empty() && validate_vertices(vertices)) {
            std::cout << "✅ Successfully loaded existing vertices" << std::endl;
            return vertices;
        } else {
            std::cout << "⚠️  Existing file invalid, regenerating..." << std::endl;
        }
    } else {
        std::cout << "   Binary file not found, generating..." << std::endl;
    }
    
    // Generate new vertices
    if (!generate_star_vertices(python_script_path, binary_filepath)) {
        std::cerr << "❌ Failed to generate STAR vertices" << std::endl;
        return {};
    }
    
    // Load the newly generated file
    auto vertices = load_vertices_from_binary(binary_filepath);
    
    if (vertices.empty() || !validate_vertices(vertices)) {
        std::cerr << "❌ Generated vertices are invalid" << std::endl;
        return {};
    }
    
    std::cout << "✅ Successfully generated and loaded STAR vertices" << std::endl;
    return vertices;
}

// =============================================================================
// VALIDATION AND UTILITIES
// =============================================================================

bool VertexLoader::validate_vertices(const std::vector<Eigen::Vector3d>& vertices, size_t expected_count) {
    if (vertices.empty()) {
        std::cerr << "❌ Vertex validation failed: empty vertex array" << std::endl;
        return false;
    }
    
    if (vertices.size() != expected_count) {
        std::cerr << "❌ Vertex validation failed: expected " << expected_count 
                  << " vertices, got " << vertices.size() << std::endl;
        return false;
    }
    
    // Check for invalid coordinates (NaN, infinite)
    size_t invalid_count = 0;
    for (size_t i = 0; i < vertices.size(); ++i) {
        const auto& v = vertices[i];
        if (!std::isfinite(v.x()) || !std::isfinite(v.y()) || !std::isfinite(v.z())) {
            invalid_count++;
            if (invalid_count <= 5) {  // Only print first few invalid vertices
                std::cerr << "⚠️  Invalid vertex " << i << ": (" << v.x() << ", " << v.y() << ", " << v.z() << ")" << std::endl;
            }
        }
    }
    
    if (invalid_count > 0) {
        std::cerr << "❌ Vertex validation failed: " << invalid_count << " invalid coordinates" << std::endl;
        return false;
    }
    
    std::cout << "✅ Vertex validation passed: " << vertices.size() << " valid vertices" << std::endl;
    return true;
}

std::string VertexLoader::get_vertex_statistics(const std::vector<Eigen::Vector3d>& vertices) {
    if (vertices.empty()) {
        return "No vertices";
    }
    
    // Calculate bounds
    Eigen::Vector3d min_bounds = vertices[0];
    Eigen::Vector3d max_bounds = vertices[0];
    
    for (const auto& vertex : vertices) {
        min_bounds = min_bounds.cwiseMin(vertex);
        max_bounds = max_bounds.cwiseMax(vertex);
    }
    
    Eigen::Vector3d size = max_bounds - min_bounds;
    Eigen::Vector3d center = (min_bounds + max_bounds) * 0.5;
    
    std::ostringstream stats;
    stats << std::fixed << std::setprecision(3);
    stats << "Vertex Statistics:\n";
    stats << "  Count: " << vertices.size() << "\n";
    stats << "  Bounds: X[" << min_bounds.x() << ", " << max_bounds.x() << "] ";
    stats << "Y[" << min_bounds.y() << ", " << max_bounds.y() << "] ";
    stats << "Z[" << min_bounds.z() << ", " << max_bounds.z() << "]\n";
    stats << "  Size: (" << size.x() << ", " << size.y() << ", " << size.z() << ")\n";
    stats << "  Center: (" << center.x() << ", " << center.y() << ", " << center.z() << ")";
    
    return stats.str();
}

bool VertexLoader::file_exists_and_readable(const std::string& filepath) {
    struct stat buffer;
    return (stat(filepath.c_str(), &buffer) == 0);
}

// =============================================================================
// CONVERSION UTILITIES
// =============================================================================

std::vector<Eigen::Vector3d> VertexLoader::convert_coordinate_system(
    const std::vector<Eigen::Vector3d>& vertices,
    double scale,
    bool swap_yz) {
    
    if (vertices.empty()) {
        return {};
    }
    
    std::cout << "🔄 Converting coordinate system..." << std::endl;
    std::cout << "   Scale factor: " << scale << std::endl;
    std::cout << "   Swap Y/Z: " << (swap_yz ? "yes" : "no") << std::endl;
    
    std::vector<Eigen::Vector3d> converted_vertices;
    converted_vertices.reserve(vertices.size());
    
    for (const auto& vertex : vertices) {
        Eigen::Vector3d converted;
        
        if (swap_yz) {
            converted.x() = vertex.x() * scale;
            converted.y() = vertex.z() * scale;  // Z -> Y
            converted.z() = vertex.y() * scale;  // Y -> Z
        } else {
            converted = vertex * scale;
        }
        
        converted_vertices.push_back(converted);
    }
    
    std::cout << "✅ Coordinate conversion complete" << std::endl;
    return converted_vertices;
}

bool VertexLoader::save_vertices_to_binary(const std::vector<Eigen::Vector3d>& vertices,
                                          const std::string& filepath) {
    std::cout << "🔄 Saving " << vertices.size() << " vertices to binary file: " << filepath << std::endl;
    
    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "❌ Could not create binary file: " << filepath << std::endl;
        return false;
    }
    
    // Write header: vertex count
    uint32_t vertex_count = static_cast<uint32_t>(vertices.size());
    file.write(reinterpret_cast<const char*>(&vertex_count), sizeof(vertex_count));
    
    // Write vertex data
    for (const auto& vertex : vertices) {
        double coords[3] = {vertex.x(), vertex.y(), vertex.z()};
        file.write(reinterpret_cast<const char*>(coords), sizeof(coords));
    }
    
    if (!file.good()) {
        std::cerr << "❌ Error writing to binary file" << std::endl;
        return false;
    }
    
    std::cout << "✅ Binary file saved successfully" << std::endl;
    return true;
}

// =============================================================================
// INTERNAL HELPER FUNCTIONS
// =============================================================================

bool VertexLoader::read_binary_header(std::ifstream& file, uint32_t& vertex_count) {
    file.read(reinterpret_cast<char*>(&vertex_count), sizeof(vertex_count));
    
    if (!file.good()) {
        std::cerr << "❌ Failed to read vertex count from binary file" << std::endl;
        return false;
    }
    
    // Sanity check
    if (vertex_count == 0 || vertex_count > 100000) {
        std::cerr << "❌ Invalid vertex count: " << vertex_count << std::endl;
        return false;
    }
    
    return true;
}

std::vector<Eigen::Vector3d> VertexLoader::read_binary_vertices(std::ifstream& file, uint32_t vertex_count) {
    std::vector<Eigen::Vector3d> vertices;
    vertices.reserve(vertex_count);
    
    for (uint32_t i = 0; i < vertex_count; ++i) {
        double coords[3];
        file.read(reinterpret_cast<char*>(coords), sizeof(coords));
        
        if (!file.good()) {
            std::cerr << "❌ Failed to read vertex " << i << " from binary file" << std::endl;
            return {};
        }
        
        vertices.emplace_back(coords[0], coords[1], coords[2]);
        
        // Progress reporting
        if ((i + 1) % VertexLoaderConstants::PROGRESS_UPDATE_INTERVAL == 0) {
            print_progress(i + 1, vertex_count, "Reading");
        }
    }
    
    return vertices;
}

bool VertexLoader::execute_command(const std::string& command) {
    int result = std::system(command.c_str());
    
    // On Unix systems, std::system returns the exit status
    // Exit status 0 means success
    return (result == 0);
}

void VertexLoader::print_progress(size_t current, size_t total, const std::string& operation_name) {
    if (total == 0) return;
    
    double percentage = (static_cast<double>(current) / total) * 100.0;
    std::cout << "   " << operation_name << " progress: " << current << "/" << total 
              << " (" << std::fixed << std::setprecision(1) << percentage << "%)" << std::endl;
}

} // namespace delta