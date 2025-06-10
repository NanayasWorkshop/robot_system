#ifndef DELTA_CORE_CONSTANTS_HPP
#define DELTA_CORE_CONSTANTS_HPP

#include <cmath>

namespace delta {

// Robot Physical Constants
constexpr double ROBOT_RADIUS = 24.8;
constexpr double MIN_HEIGHT = 101.0;
constexpr double WORKING_HEIGHT = 11.5; 
constexpr double MOTOR_LIMIT = 11.0;

// FABRIK Configuration Constants
constexpr int DEFAULT_ROBOT_SEGMENTS = 7;                              // Default number of stacked segments
constexpr double SPHERICAL_JOINT_CONE_ANGLE_RAD = 2.0 * M_PI / 3.0;   // 120 degrees full cone
constexpr double FABRIK_TOLERANCE = 0.01;                              // Convergence tolerance
constexpr int FABRIK_MAX_ITERATIONS = 100;                             // Maximum solver iterations

// NEW: Prismatic Refinement Constants
constexpr double FABRIK_PRISMATIC_TOLERANCE = 0.01;                    // 0.01mm prismatic change tolerance
constexpr int FABRIK_MAX_REFINEMENT_ITERATIONS = 5;                    // Maximum refinement loops

// Geometry Constants - Base actuator positions (angles in radians)
constexpr double BASE_A_ANGLE = M_PI / 2.0;                // 90 degrees (top)
constexpr double BASE_B_ANGLE = -M_PI / 6.0;               // -30 degrees (bottom right)  
constexpr double BASE_C_ANGLE = -5.0 * M_PI / 6.0;         // -150 degrees (bottom left)

} // namespace delta

#endif // DELTA_CORE_CONSTANTS_HPP