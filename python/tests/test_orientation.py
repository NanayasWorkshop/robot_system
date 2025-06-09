#!/usr/bin/env python3
"""
Orientation Block Test Runner
"""

import sys
sys.path.append('python/visualization')

from kinematics_wrapper import calculate_kinematics
from orientation_wrapper import calculate_orientation

def main():
    try:
        # Test input direction vector
        input_x, input_y, input_z = 0.2, 0.8, 1.0
        
        # First get kinematics result for comparison
        print("=== Step 1: Calculate Kinematics ===")
        kinematics_result = calculate_kinematics(input_x, input_y, input_z, debug=False)
        point_H, point_G, HG_length, end_effector, kinematics_calc_time, fermat_data = kinematics_result
        print(f"Kinematics calculated in {kinematics_calc_time:.3f}ms")
        print(f"End-effector: ({end_effector[0]:.3f}, {end_effector[1]:.3f}, {end_effector[2]:.3f})")
        print(f"Point G: ({point_G[0]:.3f}, {point_G[1]:.3f}, {point_G[2]:.3f})")
        
        # Then calculate orientation using same input vector
        print("\n=== Step 2: Calculate Orientation ===")
        orientation_result = calculate_orientation(input_x, input_y, input_z, debug=True)
        
        end_effector_out, point_G_out, final_frame, transformation_matrix, calc_time, kinematics_data_out = orientation_result
        
        print(f"\nOrientation calculated in {calc_time:.3f}ms")
        print(f"Transformation matrix shape: {transformation_matrix.shape}")
        
        # Verify consistency between separate kinematics call and orientation's internal kinematics
        print(f"\n=== Verification ===")
        print(f"End-effector consistency: {np.allclose(end_effector, end_effector_out)}")
        print(f"Point G consistency: {np.allclose(point_G, point_G_out)}")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    import numpy as np
    main()