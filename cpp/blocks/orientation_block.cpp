#include "orientation_block.hpp"
#include <cmath>

namespace delta {

OrientationResult OrientationBlock::calculate(const KinematicsResult& kinematics_result) {
    double calculation_time_ms = 0.0;
    
    {
        ScopedTimer timer(calculation_time_ms);
        
        // Extract what we need from kinematics result
        Eigen::Vector3d end_effector = kinematics_result.end_effector_position;
        Eigen::Vector3d point_G = kinematics_result.point_G;
        Eigen::Vector3d fermat_point = kinematics_result.fermat_data.fermat_point;
        
        // Build A, B, C points from fermat Z values
        Eigen::Vector3d A_point = build_point_A(kinematics_result.fermat_data.z_A);
        Eigen::Vector3d B_point = build_point_B(kinematics_result.fermat_data.z_B);
        Eigen::Vector3d C_point = build_point_C(kinematics_result.fermat_data.z_C);
        
        // Step 1: Create UVW coordinate system at Fermat point
        CoordinateFrame UVW_frame = create_UVW_frame(fermat_point, A_point, B_point, C_point);
        
        // Step 2: Mirror UVW across XY plane to get IJK
        CoordinateFrame IJK_frame = mirror_across_XY(UVW_frame);
        
        // Step 3: Translate to align with XYZ origin, get U'V'W'
        CoordinateFrame aligned_frame = align_with_origin(UVW_frame, IJK_frame);
        
        // Step 4: Translate to end-effector position to get U''V''W''
        CoordinateFrame final_frame = translate_to_endeffector(aligned_frame, end_effector);
        
        // Create transformation matrix from final frame
        Eigen::Matrix4d transformation = create_transformation_matrix(final_frame);
        
        return OrientationResult(end_effector, point_G, final_frame, transformation,
                               calculation_time_ms, kinematics_result);
    }
}

CoordinateFrame OrientationBlock::create_UVW_frame(const Eigen::Vector3d& fermat_point,
                                                   const Eigen::Vector3d& A_point,
                                                   const Eigen::Vector3d& B_point,
                                                   const Eigen::Vector3d& C_point) {
    
    // V-axis: Fermat point → A point (aligns with Y-axis since A is on Y-axis)
    Eigen::Vector3d v_axis = (A_point - fermat_point).normalized();
    
    // W-axis: Normal to ABC plane (pointing +Z direction)
    Eigen::Vector3d w_axis = calculate_plane_normal(A_point, B_point, C_point);
    // Ensure W points in +Z direction
    if (w_axis.z() < 0) {
        w_axis = -w_axis;
    }
    
    // U-axis: V × W (cross product) - guarantees perfect orthogonality AND right-handed system
    Eigen::Vector3d u_axis = v_axis.cross(w_axis).normalized();
    
    return CoordinateFrame(fermat_point, u_axis, v_axis, w_axis);
}

CoordinateFrame OrientationBlock::mirror_across_XY(const CoordinateFrame& uvw_frame) {
    // Mirror across XY plane: (x,y,z) → (x,y,-z)
    Eigen::Vector3d mirrored_origin(uvw_frame.origin.x(), uvw_frame.origin.y(), -uvw_frame.origin.z());
    Eigen::Vector3d mirrored_u(uvw_frame.u_axis.x(), uvw_frame.u_axis.y(), -uvw_frame.u_axis.z());
    Eigen::Vector3d mirrored_v(uvw_frame.v_axis.x(), uvw_frame.v_axis.y(), -uvw_frame.v_axis.z());
    Eigen::Vector3d mirrored_w(uvw_frame.w_axis.x(), uvw_frame.w_axis.y(), -uvw_frame.w_axis.z());
    
    // THEN invert W to make K point upward (create IJK from mirrored UVW)
    Eigen::Vector3d inverted_k = -mirrored_w;
    
    return CoordinateFrame(mirrored_origin, mirrored_u, mirrored_v, inverted_k);
}

CoordinateFrame OrientationBlock::align_with_origin(const CoordinateFrame& uvw_frame,
                                                    const CoordinateFrame& ijk_frame) {
    // Calculate transformation matrix to align IJK with XYZ
    
    // 1. Translation: move IJK origin to XYZ origin (0,0,0)
    Eigen::Vector3d translation = Eigen::Vector3d(0, 0, 0) - ijk_frame.origin;
    
    // 2. Rotation: align IJK axes with XYZ axes
    // Build IJK matrix [I J K] as column vectors
    Eigen::Matrix3d ijk_matrix;
    ijk_matrix.col(0) = ijk_frame.u_axis;  // I column
    ijk_matrix.col(1) = ijk_frame.v_axis;  // J column  
    ijk_matrix.col(2) = ijk_frame.w_axis;  // K column
    
    // Calculate rotation matrix R = [I J K]^(-1)
    Eigen::Matrix3d R = ijk_matrix.inverse();
    
    // Apply translation to UVW origin
    Eigen::Vector3d new_uvw_origin = uvw_frame.origin + translation;
    
    // Apply rotation R to UVW axes
    Eigen::Vector3d new_u_axis = R * uvw_frame.u_axis;
    Eigen::Vector3d new_v_axis = R * uvw_frame.v_axis;
    Eigen::Vector3d new_w_axis = R * uvw_frame.w_axis;
    
    return CoordinateFrame(new_uvw_origin, new_u_axis, new_v_axis, new_w_axis);
}

CoordinateFrame OrientationBlock::translate_to_endeffector(const CoordinateFrame& aligned_frame,
                                                          const Eigen::Vector3d& end_effector_position) {
    // Translate coordinate frame to end-effector position
    return CoordinateFrame(end_effector_position, 
                          aligned_frame.u_axis, 
                          aligned_frame.v_axis, 
                          aligned_frame.w_axis);
}

Eigen::Matrix4d OrientationBlock::create_transformation_matrix(const CoordinateFrame& frame) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    
    // Set rotation part (upper 3x3) - each axis as a column
    matrix.block<3,3>(0,0).col(0) = frame.u_axis;
    matrix.block<3,3>(0,0).col(1) = frame.v_axis;
    matrix.block<3,3>(0,0).col(2) = frame.w_axis;
    
    // Set translation part (last column, first 3 rows)
    matrix.block<3,1>(0,3) = frame.origin;
    
    // Bottom row is [0, 0, 0, 1] (already set by Identity)
    
    return matrix;
}

Eigen::Vector3d OrientationBlock::calculate_plane_normal(const Eigen::Vector3d& A_point,
                                                        const Eigen::Vector3d& B_point,
                                                        const Eigen::Vector3d& C_point) {
    // Calculate two vectors in the plane
    Eigen::Vector3d AB = B_point - A_point;
    Eigen::Vector3d AC = C_point - A_point;
    
    // Cross product gives normal vector - REVERSED ORDER: AC × AB instead of AB × AC
    Eigen::Vector3d normal = AC.cross(AB);
    
    return normal.normalized();
}

Eigen::Vector3d OrientationBlock::build_point_A(double z_A) {
    return Eigen::Vector3d(
        ROBOT_RADIUS * std::cos(BASE_A_ANGLE),
        ROBOT_RADIUS * std::sin(BASE_A_ANGLE),
        z_A
    );
}

Eigen::Vector3d OrientationBlock::build_point_B(double z_B) {
    return Eigen::Vector3d(
        ROBOT_RADIUS * std::cos(BASE_B_ANGLE),
        ROBOT_RADIUS * std::sin(BASE_B_ANGLE),
        z_B
    );
}

Eigen::Vector3d OrientationBlock::build_point_C(double z_C) {
    return Eigen::Vector3d(
        ROBOT_RADIUS * std::cos(BASE_C_ANGLE),
        ROBOT_RADIUS * std::sin(BASE_C_ANGLE),
        z_C
    );
}

} // namespace delta