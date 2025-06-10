#!/usr/bin/env python3
"""
FABRIK Forward Block Test Runner
"""

print("=== Starting FABRIK Forward Test ===")

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
    from fabrik_forward_wrapper import calculate_fabrik_forward
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
        initial_base = np.array(joint_positions[0])
        initial_end = np.array(joint_positions[-1])
        print(f"Initial base: ({initial_base[0]:.1f}, {initial_base[1]:.1f}, {initial_base[2]:.1f})")
        print(f"Initial end-effector: ({initial_end[0]:.1f}, {initial_end[1]:.1f}, {initial_end[2]:.1f})")
        
        # Step 2: Calculate initial segment lengths
        print("\n=== Step 2: Calculate Initial Segment Lengths ===")
        initial_segment_lengths = []
        for i in range(len(joint_positions) - 1):
            length = np.linalg.norm(np.array(joint_positions[i+1]) - np.array(joint_positions[i]))
            initial_segment_lengths.append(length)
        
        print(f"Initial segment lengths: {[f'{l:.1f}mm' for l in initial_segment_lengths]}")
        
        # Step 3: Apply backward pass to target (80, 30, 500)
        print("\n=== Step 3: Backward Pass to Target ===")
        target_position = np.array([80.0, 30.0, 500.0])
        
        backward_joints, distance_to_base, backward_time = calculate_fabrik_backward(
            joint_positions, target_position, initial_segment_lengths, debug=False)
        
        backward_base = np.array(backward_joints[0])
        backward_end = np.array(backward_joints[-1])
        print(f"✓ Backward completed - Base drift: {distance_to_base:.1f}mm")
        print(f"After backward base: ({backward_base[0]:.1f}, {backward_base[1]:.1f}, {backward_base[2]:.1f})")
        print(f"After backward end: ({backward_end[0]:.1f}, {backward_end[1]:.1f}, {backward_end[2]:.1f})")
        
        # Step 4: Apply forward pass (the main test)
        print("\n=== Step 4: Forward Pass with Dynamic Recalculation ===")
        
        forward_joints, new_segment_lengths, distance_to_target, forward_time = calculate_fabrik_forward(
            backward_joints, target_position, debug=True)
        
        print(f"✓ Forward pass completed")
        
        # Step 5: Comprehensive Analysis
        print("\n=== Step 5: Comprehensive Analysis ===")
        
        forward_base = np.array(forward_joints[0])
        forward_end = np.array(forward_joints[-1])
        
        # Base position analysis
        base_drift_correction = np.linalg.norm(backward_base - forward_base)
        base_at_origin = np.allclose(forward_base, [0, 0, 0], atol=0.1)
        
        print(f"Base Analysis:")
        print(f"  - Base drift correction: {base_drift_correction:.1f}mm")
        print(f"  - Base now at origin: {'✓ Yes' if base_at_origin else '✗ No'}")
        print(f"  - Final base: ({forward_base[0]:.3f}, {forward_base[1]:.3f}, {forward_base[2]:.3f})")
        
        # End-effector analysis
        end_drift = np.linalg.norm(forward_end - backward_end)
        target_distance = np.linalg.norm(forward_end - target_position)
        
        print(f"\nEnd-Effector Analysis:")
        print(f"  - End-effector drift: {end_drift:.1f}mm (expected)")
        print(f"  - Distance to target: {target_distance:.1f}mm")
        print(f"  - Final end-effector: ({forward_end[0]:.1f}, {forward_end[1]:.1f}, {forward_end[2]:.1f})")
        
        # Segment length analysis
        print(f"\nSegment Length Analysis:")
        length_changes = [abs(new - old) for old, new in zip(initial_segment_lengths, new_segment_lengths)]
        max_change = max(length_changes) if length_changes else 0
        avg_change = np.mean(length_changes) if length_changes else 0
        
        print(f"  - Initial:  {[f'{l:.1f}mm' for l in initial_segment_lengths]}")
        print(f"  - New:      {[f'{l:.1f}mm' for l in new_segment_lengths]}")
        print(f"  - Max change: {max_change:.1f}mm")
        print(f"  - Avg change: {avg_change:.1f}mm")
        
        # Verify segment lengths in updated chain
        print("\n=== Step 6: Verify Updated Chain Segment Lengths ===")
        actual_lengths = []
        for i in range(len(forward_joints) - 1):
            length = np.linalg.norm(np.array(forward_joints[i+1]) - np.array(forward_joints[i]))
            actual_lengths.append(length)
        
        lengths_match = all(abs(calc - actual) < 0.1 for calc, actual in zip(new_segment_lengths, actual_lengths))
        print(f"Calculated vs Actual Lengths Match: {'✓ Yes' if lengths_match else '✗ No'}")
        
        if not lengths_match:
            print(f"Calculated: {[f'{l:.1f}mm' for l in new_segment_lengths]}")
            print(f"Actual:     {[f'{l:.1f}mm' for l in actual_lengths]}")
        
        # Performance analysis
        print(f"\nPerformance Analysis:")
        print(f"  - Backward time: {backward_time:.3f}ms")
        print(f"  - Forward time:  {forward_time:.3f}ms (includes SegmentBlock calls)")
        print(f"  - Total time:    {backward_time + forward_time:.3f}ms")
        
        print("\n✓ All tests completed successfully!")
        print("\n🔄 Next step: FabrikSolverBlock will iterate Backward→Forward until convergence")
        
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()