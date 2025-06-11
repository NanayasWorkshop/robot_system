#ifndef DELTA_COLLISION_BLOCKS_CAPSULE_CREATION_BLOCK_HPP
#define DELTA_COLLISION_BLOCKS_CAPSULE_CREATION_BLOCK_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "../core/constants.hpp"
#include "../core/timing.hpp"

namespace delta {

struct CapsuleData {
    Eigen::Vector3d start_point;
    Eigen::Vector3d end_point;
    double radius;
    double length;  // Calculated automatically from start/end points
};

struct CapsuleChainResult {
    std::vector<CapsuleData> capsules;
    double total_chain_length;
    double calculation_time_ms;
    bool creation_successful;
    std::string error_message;
    
    CapsuleChainResult(const std::vector<CapsuleData>& caps, double total_length, 
                      double time_ms, bool successful, const std::string& error = "")
        : capsules(caps), total_chain_length(total_length), calculation_time_ms(time_ms)
        , creation_successful(successful), error_message(error) {}
    
    // Error constructor
    CapsuleChainResult(double time_ms, const std::string& error)
        : total_chain_length(0.0), calculation_time_ms(time_ms)
        , creation_successful(false), error_message(error) {}
};

class CapsuleCreationBlock {
public:
    static CapsuleChainResult create_capsule_chain(
        const std::vector<Eigen::Vector3d>& s_points,
        double robot_radius
    );
    
    static CapsuleChainResult update_capsule_positions(
        const std::vector<CapsuleData>& existing_capsules,
        const std::vector<Eigen::Vector3d>& new_s_points
    );

private:
    static bool validate_s_points(const std::vector<Eigen::Vector3d>& s_points);
    static bool validate_radius(double robot_radius);
    static bool validate_capsule_update(const std::vector<CapsuleData>& existing_capsules,
                                       const std::vector<Eigen::Vector3d>& new_s_points);
};

} // namespace delta

#endif // DELTA_COLLISION_BLOCKS_CAPSULE_CREATION_BLOCK_HPP