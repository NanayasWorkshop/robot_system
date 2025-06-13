#pragma once

#include <vector>
#include <string>
#include <Eigen/Dense>

namespace delta {

/**
 * Vertex Loader Utility
 * Loads STAR mesh vertices from binary files for collision detection
 */
class VertexLoader {
public:
    // =============================================================================
    // LOADING FUNCTIONS
    // =============================================================================
    
    /**
     * Load vertices from binary file
     * 
     * @param filepath Path to binary vertex file
     * @return Vector of vertex positions, empty if failed
     */
    static std::vector<Eigen::Vector3d> load_vertices_from_binary(const std::string& filepath);
    
    /**
     * Load vertices from text file (for debugging)
     * 
     * @param filepath Path to text vertex file
     * @return Vector of vertex positions, empty if failed
     */
    static std::vector<Eigen::Vector3d> load_vertices_from_text(const std::string& filepath);
    
    /**
     * Generate STAR vertices by calling Python script
     * 
     * @param python_script_path Path to get_star_vertices.py
     * @param output_binary_path Path where binary file will be saved
     * @return true if generation successful
     */
    static bool generate_star_vertices(const std::string& python_script_path = "get_star_vertices.py",
                                      const std::string& output_binary_path = "star_vertices.bin");
    
    /**
     * Load STAR vertices with automatic generation if needed
     * Main interface - tries to load existing file, generates if missing
     * 
     * @param binary_filepath Path to binary vertex file
     * @param python_script_path Path to Python generation script
     * @return Vector of vertex positions, empty if failed
     */
    static std::vector<Eigen::Vector3d> load_or_generate_star_vertices(
        const std::string& binary_filepath = "star_vertices.bin",
        const std::string& python_script_path = "get_star_vertices.py");
    
    // =============================================================================
    // VALIDATION AND UTILITIES
    // =============================================================================
    
    /**
     * Validate loaded vertices
     * 
     * @param vertices Vertex array to validate
     * @param expected_count Expected number of vertices (6890 for STAR)
     * @return true if validation passed
     */
    static bool validate_vertices(const std::vector<Eigen::Vector3d>& vertices, 
                                 size_t expected_count = 6890);
    
    /**
     * Get vertex statistics for debugging
     * 
     * @param vertices Vertex array
     * @return Statistics string
     */
    static std::string get_vertex_statistics(const std::vector<Eigen::Vector3d>& vertices);
    
    /**
     * Check if binary file exists and is readable
     * 
     * @param filepath Path to check
     * @return true if file exists and is readable
     */
    static bool file_exists_and_readable(const std::string& filepath);
    
    // =============================================================================
    // CONVERSION UTILITIES
    // =============================================================================
    
    /**
     * Convert vertices to different coordinate system if needed
     * STAR uses Y-up meters, collision system might use Z-up millimeters
     * 
     * @param vertices Input vertices
     * @param scale Scale factor (1000.0 to convert meters to millimeters)
     * @param swap_yz Whether to swap Y and Z axes
     * @return Converted vertices
     */
    static std::vector<Eigen::Vector3d> convert_coordinate_system(
        const std::vector<Eigen::Vector3d>& vertices,
        double scale = 1.0,
        bool swap_yz = false);
    
    /**
     * Save vertices to binary file (for debugging/caching)
     * 
     * @param vertices Vertex array to save
     * @param filepath Output file path
     * @return true if save successful
     */
    static bool save_vertices_to_binary(const std::vector<Eigen::Vector3d>& vertices,
                                       const std::string& filepath);

private:
    // =============================================================================
    // INTERNAL HELPER FUNCTIONS
    // =============================================================================
    
    /**
     * Read binary file header
     * 
     * @param file File stream
     * @param vertex_count Output: number of vertices
     * @return true if header read successfully
     */
    static bool read_binary_header(std::ifstream& file, uint32_t& vertex_count);
    
    /**
     * Read vertex data from binary file
     * 
     * @param file File stream
     * @param vertex_count Number of vertices to read
     * @return Vector of vertices, empty if failed
     */
    static std::vector<Eigen::Vector3d> read_binary_vertices(std::ifstream& file, uint32_t vertex_count);
    
    /**
     * Execute system command and capture output
     * 
     * @param command Command to execute
     * @return true if command executed successfully (exit code 0)
     */
    static bool execute_command(const std::string& command);
    
    /**
     * Print loading progress for large files
     * 
     * @param current Current progress
     * @param total Total items
     * @param operation_name Name of operation for display
     */
    static void print_progress(size_t current, size_t total, const std::string& operation_name);
};

// =============================================================================
// CONSTANTS
// =============================================================================

namespace VertexLoaderConstants {
    constexpr size_t STAR_VERTEX_COUNT = 6890;
    constexpr double METERS_TO_MILLIMETERS = 1000.0;
    constexpr size_t PROGRESS_UPDATE_INTERVAL = 1000;  // Print progress every N vertices
}

} // namespace delta