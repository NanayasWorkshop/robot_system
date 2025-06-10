#!/usr/bin/env python3
"""
Simple timer test to isolate the issue
"""

def test_simple_function():
    """Test the simplest possible function"""
    try:
        import delta_robot_cpp
        
        print("Testing Fermat function with debug output...")
        result = delta_robot_cpp.calculate_fermat(0.2, 0.8, 1.0)
        z_A, z_B, z_C, fermat_point, calc_time = result
        
        print(f"Fermat result: z_A={z_A:.3f}, z_B={z_B:.3f}, z_C={z_C:.3f}")
        print(f"Fermat point: ({fermat_point[0]:.3f}, {fermat_point[1]:.3f}, {fermat_point[2]:.3f})")
        print(f"Calculation time: {calc_time}ms")
        print(f"Time type: {type(calc_time)}")
        
        # Check if it's actually 0 or just very small
        if calc_time == 0.0:
            print("Time is exactly 0.0")
        elif calc_time < 0.001:
            print(f"Time is very small: {calc_time}")
        else:
            print(f"Time is measurable: {calc_time}")
            
    except Exception as e:
        print(f"Test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_simple_function()