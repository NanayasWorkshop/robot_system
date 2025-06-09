#!/usr/bin/env python3
"""
Kinematics Block Test Runner
"""

import sys
sys.path.append('python/visualization')

from kinematics_wrapper import calculate_kinematics

def main():
    try:
        point_H, point_G, HG_length, end_effector, calc_time, fermat_data = calculate_kinematics(0.2, 0.8, 1, debug=True)
        
        print(f"Point H: ({point_H[0]:.3f}, {point_H[1]:.3f}, {point_H[2]:.3f})")
        print(f"Point G: ({point_G[0]:.3f}, {point_G[1]:.3f}, {point_G[2]:.3f})")
        print(f"H→G Length: {HG_length:.3f}mm")
        print(f"End-Effector: ({end_effector[0]:.3f}, {end_effector[1]:.3f}, {end_effector[2]:.3f})")
        print(f"Calculation time: {calc_time:.3f}ms")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()