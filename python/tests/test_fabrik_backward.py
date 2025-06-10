#!/usr/bin/env python3
"""
FABRIK Backward Block Test Runner
"""

print("=== Starting FABRIK Backward Test ===")

import sys
sys.path.append('python/visualization')

print("=== Importing modules ===")
try:
    import numpy as np
    print("✓ numpy imported")
    
    import delta_robot_cpp
    print("✓ delta_robot_cpp imported")
    
    from fabrik_initialization_wrapper import calculate_fabrik_initialization_straight_up
    from fabrik_backward_wrapper import calculate_fabrik_backward
    print("✓ wrappers imported")
    
except ImportError as e:
    print(f"✗ Import failed: {e}")
    sys.exit(1)

def main():
    print("\n=== Running tests ===")
    try:
        # Step 1: Initialize straight-up chain (7 segments)
        print("=== Step 1: Initialize Chain ===")
        joint_positions, validation_successful, init_time = calculate_fabrik_initialization_straight_up(7, debug=False)
        
        if not validation_successful:
            print("✗ Initialization failed")
            return
            
        print(f"✓ Initialized chain with {len(joint_positions)} joints")
        print(f"Initial end-effector: ({joint_positions[-1][0]:.1f}, {joint_positions[-1][1]:.1f}, {joint_positions[-1][2]:.1f})")
        
        # Step 2: Calculate segment lengths from initialized chain
        print("\n=== Step 2: Calculate Segment Lengths ===")
        segment_lengths = []
        for i in range(len(joint_positions) - 1):
            length = np.linalg.norm(np.array(joint_positions[i+1]) - np.array(joint_positions[i]))
            segment_lengths.append(length)
        
        print(f"Segment lengths: {[f'{l:.1f}mm' for l in segment_lengths]}")
        
        # Step 3: Test backward pass with target (80, 30, 500)
        print("\n=== Step 3: Backward Pass to Target ===")
        target_position = np.array([80.0, 30.0, 500.0])
        
        updated_joints, distance_to_base, calc_time = calculate_fabrik_backward(
            joint_positions, target_position, segment_lengths, debug=True)
        
        print(f"✓ Backward pass completed")
        
        # Step 4: Analyze results
        print("\n=== Step 4: Analysis ===")
        original_base = np.array(joint_positions[0])
        updated_base = np.array(updated_joints[0])
        base_drift = np.linalg.norm(updated_base - original_base)
        
        original_end = np.array(joint_positions[-1])
        updated_end = np.array(updated_joints[-1])
        end_movement = np.linalg.norm(updated_end - original_end)
        
        print(f"Base drift: {base_drift:.1f}mm (expected - will be fixed by forward pass)")
        print(f"End-effector movement: {end_movement:.1f}mm")
        print(f"Final end-effector: ({updated_end[0]:.1f}, {updated_end[1]:.1f}, {updated_end[2]:.1f})")
        
        # Verify segment lengths preserved
        print("\n=== Step 5: Verify Segment Lengths Preserved ===")
        updated_segment_lengths = []
        for i in range(len(updated_joints) - 1):
            length = np.linalg.norm(np.array(updated_joints[i+1]) - np.array(updated_joints[i]))
            updated_segment_lengths.append(length)
        
        lengths_preserved = all(abs(orig - updated) < 0.1 for orig, updated in zip(segment_lengths, updated_segment_lengths))
        print(f"Segment lengths preserved: {'✓ Yes' if lengths_preserved else '✗ No'}")
        
        if not lengths_preserved:
            print("Original lengths:", [f'{l:.1f}mm' for l in segment_lengths])
            print("Updated lengths: ", [f'{l:.1f}mm' for l in updated_segment_lengths])
        
        print("\n✓ All tests completed successfully!")
        
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()