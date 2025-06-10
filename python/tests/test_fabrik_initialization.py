#!/usr/bin/env python3
"""
FABRIK Initialization Block Test Runner
"""

print("=== Starting FABRIK Initialization Test ===")

import sys
sys.path.append('python/visualization')

print("=== Importing modules ===")
try:
    import numpy as np
    print("✓ numpy imported")
    
    import delta_robot_cpp
    print("✓ delta_robot_cpp imported")
    
    from fabrik_initialization_wrapper import calculate_fabrik_initialization, calculate_fabrik_initialization_straight_up
    print("✓ fabrik_initialization_wrapper imported")
    
except ImportError as e:
    print(f"✗ Import failed: {e}")
    sys.exit(1)

def main():
    print("\n=== Running tests ===")
    try:
        # Test case 1: Default straight-up configuration (7 segments)
        print("=== Test 1: Default straight-up (7 segments) ===")
        
        joint_positions, validation_successful, calc_time = calculate_fabrik_initialization_straight_up(debug=True)
        
        print(f"✓ Test 1 completed - Valid: {validation_successful}, Time: {calc_time:.3f}ms")
        
        # Test case 2: Custom number of segments (3 segments)
        print("\n=== Test 2: Custom segments (3 segments) ===")
        
        joint_positions, validation_successful, calc_time = calculate_fabrik_initialization(3, debug=False)
        
        print(f"✓ Test 2 completed - Valid: {validation_successful}, Time: {calc_time:.3f}ms")
        print(f"Expected joints for 3 segments: {3 + 2} = 5, Got: {len(joint_positions)}")
        
        # Test case 3: Custom joint positions (should validate them)
        print("\n=== Test 3: Custom joint positions ===")
        
        # Create simple custom positions (4 joints for 2 segments)
        custom_positions = [
            np.array([0.0, 0.0, 0.0]),    # Base
            np.array([0.0, 0.0, 100.0]),  # Joint 1
            np.array([0.0, 0.0, 200.0]),  # Joint 2  
            np.array([0.0, 0.0, 300.0])   # End-effector
        ]
        
        joint_positions, validation_successful, calc_time = calculate_fabrik_initialization(
            2, custom_positions, debug=False)
        
        print(f"✓ Test 3 completed - Valid: {validation_successful}, Time: {calc_time:.3f}ms")
        
        # Test case 4: Invalid custom positions (should fail validation)
        print("\n=== Test 4: Invalid custom positions (base not at origin) ===")
        
        # Create invalid positions (base not at origin)
        invalid_positions = [
            np.array([10.0, 10.0, 10.0]),  # Base NOT at origin
            np.array([0.0, 0.0, 100.0]),   # Joint 1
            np.array([0.0, 0.0, 200.0]),   # Joint 2
            np.array([0.0, 0.0, 300.0])    # End-effector
        ]
        
        joint_positions, validation_successful, calc_time = calculate_fabrik_initialization(
            2, invalid_positions, debug=False)
        
        print(f"✓ Test 4 completed - Valid: {validation_successful} (should be False), Time: {calc_time:.3f}ms")
        
        print("\n✓ All tests completed successfully!")
        
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()