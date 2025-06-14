#pragma once

#include <vector>
#include <string>
#include <memory>
#include <Eigen/Dense>
#include <chrono>

namespace delta {

/**
 * STAR Daemon Client
 * Communicates with Python STAR daemon for real-time mesh deformation
 */
class STARDaemonClient {
public:
    /**
     * Result structure for mesh deformation requests
     */
    struct MeshResult {
        std::vector<Eigen::Vector3d> vertices;  // Deformed mesh vertices (6890)
        std::vector<Eigen::Vector3d> joints;    // Joint positions (24)
        double computation_time_ms;             // Time taken for deformation
        bool success;                           // Success flag
        std::string error_message;              // Error description if failed
        
        MeshResult() : computation_time_ms(0.0), success(false) {}
        
        // Success constructor
        MeshResult(const std::vector<Eigen::Vector3d>& verts, 
                   const std::vector<Eigen::Vector3d>& joint_pos, 
                   double time_ms)
            : vertices(verts), joints(joint_pos), computation_time_ms(time_ms), success(true) {}
        
        // Error constructor
        MeshResult(const std::string& error, double time_ms)
            : computation_time_ms(time_ms), success(false), error_message(error) {}
    };
    
    /**
     * Performance statistics
     */
    struct ClientStats {
        uint64_t total_requests;
        uint64_t successful_requests;
        uint64_t failed_requests;
        double total_request_time_ms;
        double avg_request_time_ms;
        double last_request_time_ms;
        double success_rate_percent;
        
        ClientStats() : total_requests(0), successful_requests(0), failed_requests(0),
                       total_request_time_ms(0.0), avg_request_time_ms(0.0),
                       last_request_time_ms(0.0), success_rate_percent(0.0) {}
    };

private:
    // Connection state
    int socket_fd_;
    std::string socket_path_;
    bool is_connected_;
    
    // Performance tracking
    mutable ClientStats stats_;
    
    // Constants
    static constexpr int EXPECTED_VERTEX_COUNT = 6890;
    static constexpr int EXPECTED_JOINT_COUNT = 24;
    static constexpr int POSE_PARAM_COUNT = 72;
    static constexpr size_t VERTEX_DATA_SIZE = EXPECTED_VERTEX_COUNT * 3 * sizeof(float);
    static constexpr size_t JOINT_DATA_SIZE = EXPECTED_JOINT_COUNT * 3 * sizeof(float);
    static constexpr size_t POSE_DATA_SIZE = POSE_PARAM_COUNT * sizeof(float);

public:
    STARDaemonClient();
    ~STARDaemonClient();
    
    // =============================================================================
    // CONNECTION MANAGEMENT
    // =============================================================================
    
    /**
     * Connect to STAR daemon
     * @param socket_path Path to Unix domain socket (default: /tmp/star_daemon.sock)
     * @return true if connection successful
     */
    bool connect(const std::string& socket_path = "/tmp/star_daemon.sock");
    
    /**
     * Disconnect from daemon
     */
    void disconnect();
    
    /**
     * Check if connected to daemon
     * @return true if connected
     */
    bool is_connected() const { return is_connected_; }
    
    /**
     * Test connection with neutral pose request
     * @return true if daemon responds correctly
     */
    bool test_connection();
    
    // =============================================================================
    // MESH DEFORMATION INTERFACE
    // =============================================================================
    
    /**
     * Request deformed mesh for given pose parameters
     * Main interface for real-time mesh deformation
     * 
     * @param pose_params STAR pose parameters (72 floats)
     * @return MeshResult with vertices, joints, and timing info
     */
    MeshResult get_deformed_mesh(const std::vector<float>& pose_params);
    
    /**
     * Request deformed mesh with Eigen vector interface
     * Convenience wrapper for Eigen users
     * 
     * @param pose_params STAR pose parameters as Eigen vector
     * @return MeshResult with vertices, joints, and timing info
     */
    MeshResult get_deformed_mesh(const Eigen::VectorXf& pose_params);
    
    /**
     * Get neutral T-pose mesh
     * Convenience function for T-pose (zero pose parameters)
     * 
     * @return MeshResult with T-pose mesh
     */
    MeshResult get_neutral_pose();
    
    // =============================================================================
    // PERFORMANCE AND DIAGNOSTICS
    // =============================================================================
    
    /**
     * Get performance statistics
     * @return Current client statistics
     */
    ClientStats get_statistics() const;
    
    /**
     * Reset performance statistics
     */
    void reset_statistics();
    
    /**
     * Get debug information string
     * @return Formatted debug info
     */
    std::string get_debug_info() const;
    
    /**
     * Check if daemon is responsive
     * @param timeout_ms Timeout in milliseconds (default: 1000)
     * @return true if daemon responds within timeout
     */
    bool ping_daemon(int timeout_ms = 1000);

private:
    // =============================================================================
    // INTERNAL COMMUNICATION
    // =============================================================================
    
    /**
     * Send pose parameters to daemon
     * @param pose_params Pose parameters (72 floats)
     * @return true if sent successfully
     */
    bool send_pose_parameters(const std::vector<float>& pose_params);
    
    /**
     * Receive mesh data from daemon
     * @param vertices Output: vertex positions
     * @param joints Output: joint positions
     * @return true if received successfully
     */
    bool receive_mesh_data(std::vector<Eigen::Vector3d>& vertices, 
                          std::vector<Eigen::Vector3d>& joints);
    
    /**
     * Send data with size header
     * @param data Data to send
     * @param size Size of data
     * @return true if sent successfully
     */
    bool send_data(const void* data, size_t size);
    
    /**
     * Receive exact amount of data
     * @param buffer Output buffer
     * @param size Number of bytes to receive
     * @return true if received successfully
     */
    bool receive_exact(void* buffer, size_t size);
    
    /**
     * Receive data with size header
     * @param buffer Output buffer
     * @param max_size Maximum buffer size
     * @param actual_size Output: actual size received
     * @return true if received successfully
     */
    bool receive_data_with_header(void* buffer, size_t max_size, size_t& actual_size);
    
    // =============================================================================
    // VALIDATION AND CONVERSION
    // =============================================================================
    
    /**
     * Validate pose parameters
     * @param pose_params Pose parameters to validate
     * @return true if valid
     */
    bool validate_pose_parameters(const std::vector<float>& pose_params) const;
    
    /**
     * Convert float array to Eigen vector positions
     * @param float_data Input float array (x,y,z,x,y,z,...)
     * @param count Number of 3D points
     * @return Vector of Eigen::Vector3d
     */
    std::vector<Eigen::Vector3d> convert_float_array_to_vectors(const float* float_data, int count) const;
    
    /**
     * Update statistics after request
     * @param success Whether request was successful
     * @param time_ms Time taken for request
     */
    void update_statistics(bool success, double time_ms) const;
    
    /**
     * Log error message with context
     * @param operation Operation that failed
     * @param error_msg Error message
     */
    void log_error(const std::string& operation, const std::string& error_msg) const;
    
    /**
     * Set socket timeout
     * @param timeout_ms Timeout in milliseconds
     * @return true if set successfully
     */
    bool set_socket_timeout(int timeout_ms);
};

// =============================================================================
// CONVENIENCE FUNCTIONS
// =============================================================================

/**
 * Create and connect STAR daemon client
 * @param socket_path Socket path (optional)
 * @return Connected client or nullptr if failed
 */
std::unique_ptr<STARDaemonClient> create_star_client(const std::string& socket_path = "/tmp/star_daemon.sock");

/**
 * Generate smooth pose parameters for animation
 * Helper function for creating animated poses
 * 
 * @param time Animation time in seconds
 * @param amplitude_scale Scale factor for movement amplitude (0.0-1.0)
 * @return 72-dimensional pose parameter vector
 */
std::vector<float> generate_animated_pose_params(double time, double amplitude_scale = 0.3);

} // namespace delta