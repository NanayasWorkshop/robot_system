#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

// Include all blocks
#include "../blocks/fermat_block.hpp"

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
    
    // ===== BLOCKS (FUTURE) =====
    // m.def("calculate_kinematics", ...)
    // m.def("calculate_joint_state", ...)
    
    // ===== ADVANCED (FUTURE) =====
    // m.def("solve_fabrik", ...)
}