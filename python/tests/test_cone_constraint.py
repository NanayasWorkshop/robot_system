#!/usr/bin/env python3
"""
Cone Constraint Block Test Runner
"""

import sys
sys.path.append('python/visualization')

from cone_constraint_wrapper import calculate_cone_constraint
import numpy as np

def main():
    try:
        # Test case 1: Direction within cone (should not be constrained)
        print("=== Test 1: Direction within cone ===")
        desired_direction = np.array([0.1, 0.1, 1.0])
        cone_apex = np.array([0.0, 0.0, 10.0])
        cone_axis = np.array([0.0, 0.0, 1.0])  # Z+ axis
        cone_angle_rad = np.radians(120)  # 120 degrees
        
        projected_direction, constraint_applied, calc_time = calculate_cone_constraint(
            desired_direction, cone_apex, cone_axis, cone_angle_rad, debug=True)
        
        print(f"Projected direction: ({projected_direction[0]:.3f}, {projected_direction[1]:.3f}, {projected_direction[2]:.3f})")
        print(f"Constraint applied: {constraint_applied}")
        print(f"Calculation time: {calc_time:.3f}ms")
        
        # Test case 2: Direction outside cone (should be constrained)
        print("\n=== Test 2: Direction outside cone ===")
        desired_direction = np.array([1.0, 1.0, 0.1])  # Nearly horizontal
        
        projected_direction, constraint_applied, calc_time = calculate_cone_constraint(
            desired_direction, cone_apex, cone_axis, cone_angle_rad, debug=False)
        
        print(f"Projected direction: ({projected_direction[0]:.3f}, {projected_direction[1]:.3f}, {projected_direction[2]:.3f})")
        print(f"Constraint applied: {constraint_applied}")
        print(f"Calculation time: {calc_time:.3f}ms")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()