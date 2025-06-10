#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

// Include only existing blocks
#include "../blocks/fermat_block.hpp"
#include "../blocks/kinematics_block.hpp"
#include "../blocks/joint_state_block.hpp"
#include "../blocks/orientation_block.hpp"
#include "../blocks/segment_block.hpp"

namespace py = pybind11;

PYBIND11_MODULE(delta_robot_cpp, m) {
    m.doc() = "Delta Robot C++ Modules - Clean Block-Based Architecture";
    
    // ===== CONSTANTS =====
    m.attr("ROBOT_RADIUS") = delta::ROBOT_RADIUS;
    m.attr("MIN_HEIGHT") = delta::MIN_HEIGHT;
    m.attr("WORKING_HEIGHT") = delta::WORKING_HEIGHT;
    m.attr("MOTOR_LIMIT") = delta::MOTOR_LIMIT;
    m.attr("BASE_A_ANGLE") = delta::BASE_A_ANGLE;
    m.attr("BASE_B_ANGLE") = delta::BASE_B_ANGLE;
    m.attr("BASE_C_ANGLE") = delta::BASE_C_ANGLE;
    m.attr("DEFAULT_ROBOT_SEGMENTS") = delta::DEFAULT_ROBOT_SEGMENTS;
    m.attr("SPHERICAL_JOINT_CONE_ANGLE_RAD") = delta::SPHERICAL_JOINT_CONE_ANGLE_RAD;
    m.attr("FABRIK_TOLERANCE") = delta::FABRIK_TOLERANCE;
    m.attr("FABRIK_MAX_ITERATIONS") = delta::FABRIK_MAX_ITERATIONS;
    
    // ===== COORDINATE FRAME CLASS =====
    py::class_<delta::CoordinateFrame>(m, "CoordinateFrame")
        .def(py::init<const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&>())
        .def_readwrite("origin", &delta::CoordinateFrame::origin)
        .def_readwrite("u_axis", &delta::CoordinateFrame::u_axis)
        .def_readwrite("v_axis", &delta::CoordinateFrame::v_axis)
        .def_readwrite("w_axis", &delta::CoordinateFrame::w_axis);
    
    // ===== FERMAT BLOCK =====
    m.def("calculate_fermat", [](double x, double y, double z) {
        auto result = delta::FermatBlock::calculate(x, y, z);
        return std::make_tuple(result.z_A, result.z_B, result.z_C, result.fermat_point, result.calculation_time_ms);
    }, "Calculate Fermat point and Z positions from direction vector");
    
    // ===== KINEMATICS BLOCK =====
    m.def("calculate_kinematics", [](double x, double y, double z) {
        auto result = delta::KinematicsBlock::calculate(x, y, z);
        auto fermat_tuple = std::make_tuple(result.fermat_data.z_A, result.fermat_data.z_B, result.fermat_data.z_C, 
                                           result.fermat_data.fermat_point, result.fermat_data.calculation_time_ms);
        return std::make_tuple(result.point_H, result.point_G, result.HG_length, 
                              result.end_effector_position, result.calculation_time_ms, fermat_tuple);
    }, "Calculate kinematics (H, G, end-effector) from direction vector");
    
    // ===== JOINT STATE BLOCK =====
    m.def("calculate_joint_state", [](double x, double y, double z) {
        auto result = delta::JointStateBlock::calculate(x, y, z);
        auto fermat_tuple = std::make_tuple(result.fermat_data.z_A, result.fermat_data.z_B, result.fermat_data.z_C,
                                           result.fermat_data.fermat_point, result.fermat_data.calculation_time_ms);
        return std::make_tuple(result.prismatic_joint, result.roll_joint, result.pitch_joint,
                              result.calculation_time_ms, fermat_tuple);
    }, "Calculate joint states (prismatic, roll, pitch) from direction vector");
    
    // ===== ORIENTATION BLOCK =====
    m.def("calculate_orientation", [](double x, double y, double z) {
        auto kinematics_result = delta::KinematicsBlock::calculate(x, y, z);
        auto result = delta::OrientationBlock::calculate(kinematics_result);
        
        auto final_frame_tuple = std::make_tuple(result.final_frame.origin, result.final_frame.u_axis, 
                                                result.final_frame.v_axis, result.final_frame.w_axis);
        auto fermat_tuple = std::make_tuple(kinematics_result.fermat_data.z_A, kinematics_result.fermat_data.z_B, 
                                           kinematics_result.fermat_data.z_C, kinematics_result.fermat_data.fermat_point, 
                                           kinematics_result.fermat_data.calculation_time_ms);
        auto kinematics_tuple = std::make_tuple(kinematics_result.point_H, kinematics_result.point_G, 
                                               kinematics_result.HG_length, kinematics_result.end_effector_position, 
                                               kinematics_result.calculation_time_ms, fermat_tuple);
        
        return std::make_tuple(result.end_effector_position, result.point_G, final_frame_tuple,
                              result.transformation_matrix, result.calculation_time_ms, kinematics_tuple);
    }, "Calculate orientation from direction vector");
    
    // ===== SEGMENT BLOCK - ESSENTIAL INTERFACE =====
    m.def("calculate_segment_essential", [](double x, double y, double z, 
                                           double prev_x = 0.0, double prev_y = 0.0, double prev_z = 1.0) {
        auto result = delta::SegmentBlock::calculate_essential(
            Eigen::Vector3d(x, y, z), 
            Eigen::Vector3d(prev_x, prev_y, prev_z)
        );
        return std::make_tuple(result.prismatic_length, result.calculation_time_ms, 
                              result.calculation_successful, result.error_message);
    }, "Calculate segment prismatic length (direction-based)",
       py::arg("x"), py::arg("y"), py::arg("z"), 
       py::arg("prev_x") = 0.0, py::arg("prev_y") = 0.0, py::arg("prev_z") = 1.0);
    
    m.def("calculate_segment_essential_from_joints", [](const std::vector<Eigen::Vector3d>& joint_positions, int segment_index) {
        auto result = delta::SegmentBlock::calculate_essential_from_joints(joint_positions, segment_index);
        
        // Return essential data + J→S conversion info
        py::object calc_seg_pos = result.calculated_segment_position.has_value() ? 
            py::cast(result.calculated_segment_position.value()) : py::none();
        py::object calc_dir = result.calculated_direction.has_value() ? 
            py::cast(result.calculated_direction.value()) : py::none();
        
        return std::make_tuple(result.prismatic_length, result.calculation_time_ms, 
                              result.calculation_successful, result.error_message,
                              calc_seg_pos, calc_dir);
    }, "Calculate segment prismatic length from FABRIK joints (with J→S conversion)",
       py::arg("joint_positions"), py::arg("segment_index"));
    
    // ===== SEGMENT BLOCK - COMPLETE INTERFACE (for detailed analysis only) =====
    m.def("calculate_segment_complete", [](double x, double y, double z,
                                          double prev_x = 0.0, double prev_y = 0.0, double prev_z = 1.0) {
        auto result = delta::SegmentBlock::calculate_complete(
            Eigen::Vector3d(x, y, z), 
            Eigen::Vector3d(prev_x, prev_y, prev_z)
        );
        
        // Just return success status for complete mode - detailed data available in C++ if needed
        return std::make_tuple(result.prismatic_length, result.calculation_time_ms, 
                              result.calculation_successful, result.error_message,
                              result.kinematics_data.has_value(), 
                              result.joint_state_data.has_value(),
                              result.orientation_data.has_value());
    }, "Calculate segment with full analysis (direction-based)",
       py::arg("x"), py::arg("y"), py::arg("z"), 
       py::arg("prev_x") = 0.0, py::arg("prev_y") = 0.0, py::arg("prev_z") = 1.0);
    
    m.def("calculate_segment_complete_from_joints", [](const std::vector<Eigen::Vector3d>& joint_positions, int segment_index) {
        auto result = delta::SegmentBlock::calculate_complete_from_joints(joint_positions, segment_index);
        
        py::object calc_seg_pos = result.calculated_segment_position.has_value() ? 
            py::cast(result.calculated_segment_position.value()) : py::none();
        py::object calc_dir = result.calculated_direction.has_value() ? 
            py::cast(result.calculated_direction.value()) : py::none();
        
        return std::make_tuple(result.prismatic_length, result.calculation_time_ms, 
                              result.calculation_successful, result.error_message,
                              calc_seg_pos, calc_dir,
                              result.kinematics_data.has_value(), 
                              result.joint_state_data.has_value(),
                              result.orientation_data.has_value());
    }, "Calculate segment with full analysis from FABRIK joints (with J→S conversion)",
       py::arg("joint_positions"), py::arg("segment_index"));
}