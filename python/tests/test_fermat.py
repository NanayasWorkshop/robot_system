#!/usr/bin/env python3
"""
Fermat Block Test Runner
"""

import sys
sys.path.append('python/visualization')

from fermat_wrapper import calculate_fermat

def main():
    try:
        z_A, z_B, z_C, fermat_point, calc_time = calculate_fermat(0.2, 0.1, 1, debug=True)
        
        print(f"Z positions: A={z_A:.3f}, B={z_B:.3f}, C={z_C:.3f}")
        print(f"Fermat point: ({fermat_point[0]:.3f}, {fermat_point[1]:.3f}, {fermat_point[2]:.3f})")
        print(f"Calculation time: {calc_time:.3f}ms")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()