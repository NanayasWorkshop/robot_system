#ifndef DELTA_BLOCKS_FERMAT_BLOCK_HPP
#define DELTA_BLOCKS_FERMAT_BLOCK_HPP

#include <Eigen/Dense>
#include "../core/constants.hpp"
#include "../core/timing.hpp"

namespace delta {

struct FermatResult {
    double z_A;                    // Z-position of point A
    double z_B;                    // Z-position of point B  
    double z_C;                    // Z-position of point C
    Eigen::Vector3d fermat_point;  // Calculated Fermat point
    double calculation_time_ms;    // Time taken for calculation
    
    FermatResult(double zA, double zB, double zC, const Eigen::Vector3d& fermat, double time_ms)
        : z_A(zA), z_B(zB), z_C(zC), fermat_point(fermat), calculation_time_ms(time_ms) {}
};

class FermatBlock {
public:
    // Main interface: input direction vector, get true Fermat calculation
    static FermatResult calculate(double x, double y, double z);
    static FermatResult calculate(const Eigen::Vector3d& input_vector);
    
    // Helper: get base positions (for reference/debugging)
    static Eigen::Vector3d get_base_position_A();
    static Eigen::Vector3d get_base_position_B();
    static Eigen::Vector3d get_base_position_C();

private:
    // Internal data structure for detailed Fermat calculation
    struct FermatCalculationData {
        double z_A, z_B, z_C;           // Z positions of triangle vertices
        Eigen::Vector3d fermat_point;   // True Fermat point
    };
    
    // Internal calculation helpers
    static FermatCalculationData calculate_true_fermat_point(const Eigen::Vector3d& normalized_direction);
    static double calculate_z_intersection(double base_x, double base_y, 
                                         const Eigen::Vector3d& normal);
};

} // namespace delta

#endif // DELTA_BLOCKS_FERMAT_BLOCK_HPP