#ifndef DELTA_BLOCKS_KINEMATICS_BLOCK_HPP
#define DELTA_BLOCKS_KINEMATICS_BLOCK_HPP

#include <Eigen/Dense>
#include "../core/constants.hpp"
#include "../core/timing.hpp"
#include "fermat_block.hpp"

namespace delta {

struct KinematicsResult {
    Eigen::Vector3d point_H;               // Joint H position (0, 0, WORKING_HEIGHT)
    Eigen::Vector3d point_G;               // Point G along direction vector
    double HG_length;                      // Distance between H and G
    Eigen::Vector3d end_effector_position; // Final end-effector position
    double calculation_time_ms;            // Time taken for calculation
    
    // Reference data from FermatBlock (for debugging/visualization)
    FermatResult fermat_data;              // Complete fermat calculation results
    
    KinematicsResult(const Eigen::Vector3d& H, const Eigen::Vector3d& G, 
                    double length, const Eigen::Vector3d& end_effector, 
                    double time_ms, const FermatResult& fermat)
        : point_H(H), point_G(G), HG_length(length)
        , end_effector_position(end_effector), calculation_time_ms(time_ms)
        , fermat_data(fermat) {}
};

class KinematicsBlock {
public:
    // Main interface: input direction vector, get kinematics calculation
    static KinematicsResult calculate(double x, double y, double z);
    static KinematicsResult calculate(const Eigen::Vector3d& direction_vector);

private:
    // Internal calculation helpers
    static Eigen::Vector3d calculate_point_G(const Eigen::Vector3d& direction_vector, 
                                            const Eigen::Vector3d& fermat_point);
    static Eigen::Vector3d calculate_end_effector_position(const Eigen::Vector3d& point_H, 
                                                          const Eigen::Vector3d& point_G, 
                                                          const Eigen::Vector3d& direction_vector);
};

} // namespace delta

#endif // DELTA_BLOCKS_KINEMATICS_BLOCK_HPP