#!/usr/bin/env python3
"""
Joint State Block Test Runner
"""

import sys
sys.path.append('python/visualization')

from joint_state_wrapper import calculate_joint_state

def main():
    try:
        prismatic, roll, pitch, calc_time, fermat_data = calculate_joint_state(1, 1, 1, debug=True)
        
        print(f"Prismatic joint: {prismatic:.3f}")
        print(f"Roll joint: {roll:.3f} rad")
        print(f"Pitch joint: {pitch:.3f} rad")
        print(f"Calculation time: {calc_time:.3f}ms")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()