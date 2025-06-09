#ifndef DELTA_BLOCKS_JOINT_STATE_BLOCK_HPP
#define DELTA_BLOCKS_JOINT_STATE_BLOCK_HPP

#include <Eigen/Dense>
#include "../core/constants.hpp"
#include "../core/timing.hpp"
#include "fermat_block.hpp"

namespace delta {

struct JointStateResult {
    double prismatic_joint;                // Prismatic joint position
    double roll_joint;                     // Roll joint (rotation around X-axis)
    double pitch_joint;                    // Pitch joint (rotation around Y-axis)
    double calculation_time_ms;            // Time taken for calculation
    
    // Reference data from FermatBlock (for debugging/visualization)
    FermatResult fermat_data;              // Complete fermat calculation results
    
    JointStateResult(double prismatic, double roll, double pitch,
                    double time_ms, const FermatResult& fermat)
        : prismatic_joint(prismatic), roll_joint(roll), pitch_joint(pitch)
        , calculation_time_ms(time_ms), fermat_data(fermat) {}
};

class JointStateBlock {
public:
    // Main interface: input direction vector, get joint state calculation
    static JointStateResult calculate(double x, double y, double z);
    static JointStateResult calculate(const Eigen::Vector3d& direction_vector);

private:
    // Internal calculation helpers
    static double calculate_prismatic_joint(const Eigen::Vector3d& fermat_point);
    static double calculate_roll_joint(const Eigen::Vector3d& direction_vector);
    static double calculate_pitch_joint(const Eigen::Vector3d& direction_vector);
};

} // namespace delta

#endif // DELTA_BLOCKS_JOINT_STATE_BLOCK_HPP