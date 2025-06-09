#ifndef DELTA_BLOCKS_ORIENTATION_BLOCK_HPP
#define DELTA_BLOCKS_ORIENTATION_BLOCK_HPP

#include <Eigen/Dense>
#include "../core/constants.hpp"
#include "../core/timing.hpp"
#include "kinematics_block.hpp"

namespace delta {

// Coordinate Frame structure using Eigen types
struct CoordinateFrame {
    Eigen::Vector3d origin;
    Eigen::Vector3d u_axis;
    Eigen::Vector3d v_axis;
    Eigen::Vector3d w_axis;
    
    CoordinateFrame(const Eigen::Vector3d& orig = Eigen::Vector3d(0,0,0),
                    const Eigen::Vector3d& u = Eigen::Vector3d(1,0,0),
                    const Eigen::Vector3d& v = Eigen::Vector3d(0,1,0),
                    const Eigen::Vector3d& w = Eigen::Vector3d(0,0,1))
        : origin(orig), u_axis(u), v_axis(v), w_axis(w) {}
};

struct OrientationResult {
    Eigen::Vector3d end_effector_position;    // From kinematics_result
    Eigen::Vector3d point_G;                  // From kinematics_result  
    CoordinateFrame final_frame;              // U''V''W'' coordinate frame
    Eigen::Matrix4d transformation_matrix;    // 4x4 homogeneous transform
    double calculation_time_ms;               // Timing measurement
    
    // Reference data for debugging/visualization
    KinematicsResult kinematics_data;         // Complete input data
    
    OrientationResult(const Eigen::Vector3d& end_effector, const Eigen::Vector3d& pointG,
                     const CoordinateFrame& final, const Eigen::Matrix4d& transform,
                     double time_ms, const KinematicsResult& kinematics)
        : end_effector_position(end_effector), point_G(pointG), final_frame(final)
        , transformation_matrix(transform), calculation_time_ms(time_ms)
        , kinematics_data(kinematics) {}
};

class OrientationBlock {
public:
    // Main interface: input kinematics result, get orientation calculation
    static OrientationResult calculate(const KinematicsResult& kinematics_result);

private:
    // Step 1: Create UVW coordinate system at Fermat point
    static CoordinateFrame create_UVW_frame(const Eigen::Vector3d& fermat_point,
                                           const Eigen::Vector3d& A_point,
                                           const Eigen::Vector3d& B_point,
                                           const Eigen::Vector3d& C_point);
    
    // Step 2: Mirror UVW across XY plane to get IJK
    static CoordinateFrame mirror_across_XY(const CoordinateFrame& uvw_frame);
    
    // Step 3: Translate IJK to align with XYZ origin, get U'V'W'
    static CoordinateFrame align_with_origin(const CoordinateFrame& uvw_frame,
                                           const CoordinateFrame& ijk_frame);
    
    // Step 4: Translate to end-effector position to get U''V''W''
    static CoordinateFrame translate_to_endeffector(const CoordinateFrame& aligned_frame,
                                                   const Eigen::Vector3d& end_effector_position);
    
    // Helper: Create transformation matrix from coordinate frame
    static Eigen::Matrix4d create_transformation_matrix(const CoordinateFrame& frame);
    
    // Helper: Calculate normal to ABC plane
    static Eigen::Vector3d calculate_plane_normal(const Eigen::Vector3d& A_point,
                                                const Eigen::Vector3d& B_point,
                                                const Eigen::Vector3d& C_point);
    
    // Helper: Build A, B, C points from base positions and Z values
    static Eigen::Vector3d build_point_A(double z_A);
    static Eigen::Vector3d build_point_B(double z_B);
    static Eigen::Vector3d build_point_C(double z_C);
};

} // namespace delta

#endif // DELTA_BLOCKS_ORIENTATION_BLOCK_HPP