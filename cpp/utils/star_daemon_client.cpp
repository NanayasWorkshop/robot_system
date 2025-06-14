#include "star_daemon_client.hpp"
#include "../core/timing.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <cmath>
#include <memory>
#include <limits>

namespace delta {

STARDaemonClient::STARDaemonClient() 
    : socket_fd_(-1), is_connected_(false) {
}

STARDaemonClient::~STARDaemonClient() {
    disconnect();
}

// =============================================================================
// CONNECTION MANAGEMENT
// =============================================================================

bool STARDaemonClient::connect(const std::string& socket_path) {
    socket_path_ = socket_path;
    
    // Create socket
    socket_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (socket_fd_ == -1) {
        log_error("socket creation", "Failed to create Unix domain socket");
        return false;
    }
    
    // Set up address
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, socket_path.c_str(), sizeof(addr.sun_path) - 1);
    
    // Connect to daemon
    if (::connect(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
        log_error("connection", "Failed to connect to STAR daemon at " + socket_path);
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    
    // Set socket timeout
    if (!set_socket_timeout(5000)) {  // 5 second timeout
        log_error("socket timeout", "Failed to set socket timeout");
        disconnect();
        return false;
    }
    
    is_connected_ = true;
    
    // Test connection
    if (!test_connection()) {
        log_error("connection test", "Daemon connection test failed");
        disconnect();
        return false;
    }
    
    std::cout << "✅ STARDaemonClient connected to " << socket_path << std::endl;
    return true;
}

void STARDaemonClient::disconnect() {
    if (socket_fd_ != -1) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    is_connected_ = false;
}

bool STARDaemonClient::test_connection() {
    if (!is_connected_) {
        return false;
    }
    
    // Test with neutral pose
    auto result = get_neutral_pose();
    
    if (!result.success) {
        log_error("connection test", result.error_message);
        return false;
    }
    
    // Validate result
    if (result.vertices.size() != EXPECTED_VERTEX_COUNT || 
        result.joints.size() != EXPECTED_JOINT_COUNT) {
        log_error("connection test", 
                 "Unexpected data size: " + std::to_string(result.vertices.size()) + 
                 " vertices, " + std::to_string(result.joints.size()) + " joints");
        return false;
    }
    
    std::cout << "✅ STAR daemon test successful: " << result.vertices.size() 
              << " vertices, " << result.joints.size() << " joints, " 
              << result.computation_time_ms << "ms" << std::endl;
    
    return true;
}

// =============================================================================
// MESH DEFORMATION INTERFACE
// =============================================================================

STARDaemonClient::MeshResult STARDaemonClient::get_deformed_mesh(const std::vector<float>& pose_params) {
    double total_time = 0.0;
    ScopedTimer timer(total_time);
    
    if (!is_connected_) {
        auto result = MeshResult("Not connected to daemon", total_time);
        update_statistics(false, total_time);
        return result;
    }
    
    if (!validate_pose_parameters(pose_params)) {
        auto result = MeshResult("Invalid pose parameters", total_time);
        update_statistics(false, total_time);
        return result;
    }
    
    // Send pose parameters
    if (!send_pose_parameters(pose_params)) {
        auto result = MeshResult("Failed to send pose parameters", total_time);
        update_statistics(false, total_time);
        return result;
    }
    
    // Receive mesh data
    std::vector<Eigen::Vector3d> vertices, joints;
    if (!receive_mesh_data(vertices, joints)) {
        auto result = MeshResult("Failed to receive mesh data", total_time);
        update_statistics(false, total_time);
        return result;
    }
    
    // Validate received data
    if (vertices.size() != EXPECTED_VERTEX_COUNT || joints.size() != EXPECTED_JOINT_COUNT) {
        auto result = MeshResult("Received invalid data size", total_time);
        update_statistics(false, total_time);
        return result;
    }
    
    auto result = MeshResult(vertices, joints, total_time);
    update_statistics(true, total_time);
    return result;
}

STARDaemonClient::MeshResult STARDaemonClient::get_deformed_mesh(const Eigen::VectorXf& pose_params) {
    if (pose_params.size() != POSE_PARAM_COUNT) {
        return MeshResult("Invalid pose parameter count", 0.0);
    }
    
    std::vector<float> params(pose_params.data(), pose_params.data() + pose_params.size());
    return get_deformed_mesh(params);
}

STARDaemonClient::MeshResult STARDaemonClient::get_neutral_pose() {
    std::vector<float> neutral_params(POSE_PARAM_COUNT, 0.0f);
    return get_deformed_mesh(neutral_params);
}

// =============================================================================
// PERFORMANCE AND DIAGNOSTICS
// =============================================================================

STARDaemonClient::ClientStats STARDaemonClient::get_statistics() const {
    ClientStats current_stats = stats_;
    
    // Calculate derived statistics
    if (current_stats.total_requests > 0) {
        current_stats.avg_request_time_ms = current_stats.total_request_time_ms / current_stats.total_requests;
        current_stats.success_rate_percent = (static_cast<double>(current_stats.successful_requests) / 
                                            current_stats.total_requests) * 100.0;
    }
    
    return current_stats;
}

void STARDaemonClient::reset_statistics() {
    stats_ = ClientStats();
}

std::string STARDaemonClient::get_debug_info() const {
    auto stats = get_statistics();
    
    std::string info = "STARDaemonClient Debug Info:\n";
    info += "  Status: " + std::string(is_connected_ ? "Connected" : "Disconnected") + "\n";
    info += "  Socket: " + socket_path_ + "\n";
    info += "  Total Requests: " + std::to_string(stats.total_requests) + "\n";
    info += "  Successful: " + std::to_string(stats.successful_requests) + "\n";
    info += "  Failed: " + std::to_string(stats.failed_requests) + "\n";
    info += "  Success Rate: " + std::to_string(stats.success_rate_percent) + "%\n";
    info += "  Avg Request Time: " + std::to_string(stats.avg_request_time_ms) + "ms\n";
    info += "  Last Request Time: " + std::to_string(stats.last_request_time_ms) + "ms\n";
    
    return info;
}

bool STARDaemonClient::ping_daemon(int timeout_ms) {
    if (!is_connected_) {
        return false;
    }
    
    // Set temporary timeout
    if (!set_socket_timeout(timeout_ms)) {
        return false;
    }
    
    // Quick neutral pose test
    auto result = get_neutral_pose();
    
    // Restore original timeout
    set_socket_timeout(5000);
    
    return result.success;
}

// =============================================================================
// INTERNAL COMMUNICATION
// =============================================================================

bool STARDaemonClient::send_pose_parameters(const std::vector<float>& pose_params) {
    return send_data(pose_params.data(), POSE_DATA_SIZE);
}

bool STARDaemonClient::receive_mesh_data(std::vector<Eigen::Vector3d>& vertices, 
                                        std::vector<Eigen::Vector3d>& joints) {
    // Receive vertices
    std::vector<float> vertex_buffer(EXPECTED_VERTEX_COUNT * 3);
    size_t vertex_size;
    if (!receive_data_with_header(vertex_buffer.data(), VERTEX_DATA_SIZE, vertex_size)) {
        return false;
    }
    
    if (vertex_size != VERTEX_DATA_SIZE) {
        log_error("receive vertices", "Unexpected vertex data size: " + std::to_string(vertex_size));
        return false;
    }
    
    // Receive joints  
    std::vector<float> joint_buffer(EXPECTED_JOINT_COUNT * 3);
    size_t joint_size;
    if (!receive_data_with_header(joint_buffer.data(), JOINT_DATA_SIZE, joint_size)) {
        return false;
    }
    
    if (joint_size != JOINT_DATA_SIZE) {
        log_error("receive joints", "Unexpected joint data size: " + std::to_string(joint_size));
        return false;
    }
    
    // Convert to Eigen vectors
    vertices = convert_float_array_to_vectors(vertex_buffer.data(), EXPECTED_VERTEX_COUNT);
    joints = convert_float_array_to_vectors(joint_buffer.data(), EXPECTED_JOINT_COUNT);
    
    return true;
}

bool STARDaemonClient::send_data(const void* data, size_t size) {
    ssize_t sent = send(socket_fd_, data, size, 0);
    if (sent != static_cast<ssize_t>(size)) {
        log_error("send", "Failed to send data: " + std::string(strerror(errno)));
        return false;
    }
    return true;
}

bool STARDaemonClient::receive_exact(void* buffer, size_t size) {
    size_t received = 0;
    char* buf = static_cast<char*>(buffer);
    
    while (received < size) {
        ssize_t bytes = recv(socket_fd_, buf + received, size - received, 0);
        if (bytes <= 0) {
            log_error("receive", "Failed to receive data: " + std::string(strerror(errno)));
            return false;
        }
        received += bytes;
    }
    return true;
}

bool STARDaemonClient::receive_data_with_header(void* buffer, size_t max_size, size_t& actual_size) {
    // Receive size header
    uint32_t data_size;
    if (!receive_exact(&data_size, sizeof(data_size))) {
        return false;
    }
    
    if (data_size > max_size) {
        log_error("receive header", "Data size exceeds buffer: " + std::to_string(data_size));
        return false;
    }
    
    // Receive actual data
    if (!receive_exact(buffer, data_size)) {
        return false;
    }
    
    actual_size = data_size;
    return true;
}

// =============================================================================
// VALIDATION AND CONVERSION
// =============================================================================

bool STARDaemonClient::validate_pose_parameters(const std::vector<float>& pose_params) const {
    if (pose_params.size() != POSE_PARAM_COUNT) {
        return false;
    }
    
    // Check for NaN or infinite values
    for (float param : pose_params) {
        if (!std::isfinite(param)) {
            return false;
        }
    }
    
    return true;
}

std::vector<Eigen::Vector3d> STARDaemonClient::convert_float_array_to_vectors(const float* float_data, int count) const {
    std::vector<Eigen::Vector3d> vectors;
    vectors.reserve(count);
    
    for (int i = 0; i < count; ++i) {
        int idx = i * 3;
        vectors.emplace_back(
            static_cast<double>(float_data[idx]),
            static_cast<double>(float_data[idx + 1]),
            static_cast<double>(float_data[idx + 2])
        );
    }
    
    return vectors;
}

void STARDaemonClient::update_statistics(bool success, double time_ms) const {
    stats_.total_requests++;
    stats_.total_request_time_ms += time_ms;
    stats_.last_request_time_ms = time_ms;
    
    if (success) {
        stats_.successful_requests++;
    } else {
        stats_.failed_requests++;
    }
}

void STARDaemonClient::log_error(const std::string& operation, const std::string& error_msg) const {
    std::cerr << "STARDaemonClient [" << operation << "]: " << error_msg << std::endl;
}

bool STARDaemonClient::set_socket_timeout(int timeout_ms) {
    struct timeval timeout;
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1) {
        return false;
    }
    
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) == -1) {
        return false;
    }
    
    return true;
}

// =============================================================================
// CONVENIENCE FUNCTIONS
// =============================================================================

std::unique_ptr<STARDaemonClient> create_star_client(const std::string& socket_path) {
    auto client = std::make_unique<STARDaemonClient>();
    
    if (!client->connect(socket_path)) {
        return nullptr;
    }
    
    return client;
}

std::vector<float> generate_animated_pose_params(double time, double amplitude_scale) {
    std::vector<float> pose_params(72, 0.0f);
    
    // Clamp amplitude scale
    amplitude_scale = std::max(0.0, std::min(1.0, amplitude_scale));
    
    // Animation frequencies
    const double arm_freq = 0.8;      // Arm swing frequency (Hz)
    const double leg_freq = 0.6;      // Leg movement frequency (Hz)
    const double spine_freq = 0.4;    // Spine movement frequency (Hz)
    
    // Animation amplitudes (scaled)
    const double arm_amplitude = 0.15 * amplitude_scale;    // Max 15 degrees
    const double leg_amplitude = 0.10 * amplitude_scale;    // Max 10 degrees  
    const double spine_amplitude = 0.08 * amplitude_scale;  // Max 8 degrees
    
    // STAR joint indices (approximate - may need adjustment based on STAR model)
    // These indices are estimates and may need to be verified with STAR documentation
    
    // Spine animation (joints 3, 6, 9 - spine1, spine2, spine3)
    double spine_movement = spine_amplitude * std::sin(2.0 * M_PI * spine_freq * time);
    pose_params[9] = static_cast<float>(spine_movement);   // spine1 X rotation
    pose_params[18] = static_cast<float>(spine_movement * 0.8);  // spine2 X rotation
    pose_params[27] = static_cast<float>(spine_movement * 0.6);  // spine3 X rotation
    
    // Left arm animation (shoulder, elbow - joints 16, 18)
    double left_arm_offset = arm_amplitude * std::sin(2.0 * M_PI * arm_freq * time);
    pose_params[48] = static_cast<float>(left_arm_offset);      // left_shoulder X rotation
    pose_params[49] = static_cast<float>(left_arm_offset * 0.5); // left_shoulder Y rotation
    pose_params[54] = static_cast<float>(-left_arm_offset * 0.8); // left_elbow X rotation (opposite)
    
    // Right arm animation (opposite phase)
    double right_arm_offset = -arm_amplitude * std::sin(2.0 * M_PI * arm_freq * time);
    pose_params[51] = static_cast<float>(right_arm_offset);      // right_shoulder X rotation
    pose_params[52] = static_cast<float>(right_arm_offset * 0.5); // right_shoulder Y rotation
    pose_params[57] = static_cast<float>(-right_arm_offset * 0.8); // right_elbow X rotation
    
    // Leg animation (subtle weight shift - hips, knees)
    double leg_shift = leg_amplitude * std::sin(2.0 * M_PI * leg_freq * time * 0.7);
    
    // Left leg
    pose_params[3] = static_cast<float>(-leg_shift * 0.5);  // left_hip X rotation
    pose_params[4] = static_cast<float>(leg_shift * 0.3);   // left_hip Y rotation
    pose_params[12] = static_cast<float>(leg_shift * 0.6);  // left_knee X rotation
    
    // Right leg (opposite)
    pose_params[6] = static_cast<float>(leg_shift * 0.5);   // right_hip X rotation
    pose_params[7] = static_cast<float>(-leg_shift * 0.3);  // right_hip Y rotation
    pose_params[15] = static_cast<float>(-leg_shift * 0.6); // right_knee X rotation
    
    return pose_params;
}

} // namespace delta