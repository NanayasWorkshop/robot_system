#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

// Include all blocks
#include "../blocks/fermat_block.hpp"
#include "../blocks/kinematics_block.hpp"
#include "../blocks/joint_state_block.hpp"

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
    
    // ===== FERMAT BLOCK =====
    m.def("calculate_fermat", [](double x, double y, double z) {
        auto result = delta::FermatBlock::calculate(x, y, z);
        return std::make_tuple(result.z_A, result.z_B, result.z_C, result.fermat_point, result.calculation_time_ms);
    }, "Calculate Fermat point and Z positions from direction vector");
    
    // ===== KINEMATICS BLOCK =====
    m.def("calculate_kinematics", [](double x, double y, double z) {
        auto result = delta::KinematicsBlock::calculate(x, y, z);
        // Return: point_H, point_G, HG_length, end_effector, calculation_time_ms, fermat_data
        auto fermat_tuple = std::make_tuple(result.fermat_data.z_A, result.fermat_data.z_B, result.fermat_data.z_C, 
                                           result.fermat_data.fermat_point, result.fermat_data.calculation_time_ms);
        return std::make_tuple(result.point_H, result.point_G, result.HG_length, 
                              result.end_effector_position, result.calculation_time_ms, fermat_tuple);
    }, "Calculate kinematics (H, G, end-effector) from direction vector");
    
    // ===== JOINT STATE BLOCK =====
    m.def("calculate_joint_state", [](double x, double y, double z) {
        auto result = delta::JointStateBlock::calculate(x, y, z);
        // Return: prismatic_joint, roll_joint, pitch_joint, calculation_time_ms, fermat_data
        auto fermat_tuple = std::make_tuple(result.fermat_data.z_A, result.fermat_data.z_B, result.fermat_data.z_C,
                                           result.fermat_data.fermat_point, result.fermat_data.calculation_time_ms);
        return std::make_tuple(result.prismatic_joint, result.roll_joint, result.pitch_joint,
                              result.calculation_time_ms, fermat_tuple);
    }, "Calculate joint states (prismatic, roll, pitch) from direction vector");
    
    // ===== BLOCKS (FUTURE) =====
    // m.def("calculate_workspace", ...)
    
    // ===== ADVANCED (FUTURE) =====
    // m.def("solve_fabrik", ...)
}