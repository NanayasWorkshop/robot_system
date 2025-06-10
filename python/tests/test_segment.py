#!/usr/bin/env python3
"""
Segment Block Test Runner - Updated to calculate ALL possible segments
"""

import sys
sys.path.append('python/visualization')

from segment_wrapper import (
    calculate_segment_essential,
    calculate_segment_essential_from_joints,
    calculate_segment_complete,
    calculate_segment_complete_from_joints,
    visualize_all_segments_and_joints
)
import numpy as np

def test_direction_based_methods():
    """Test direction-based segment calculation methods"""
    print("=" * 50)
    print("TESTING DIRECTION-BASED METHODS")
    print("=" * 50)
    
    # Test case 1: Essential calculation with default previous direction
    print("\n=== Test 1: Essential (Default Previous Direction) ===")
    try:
        current_x, current_y, current_z = 0.2, 0.8, 1.0
        result = calculate_segment_essential(current_x, current_y, current_z, debug=True)
        prismatic_length, calc_time, success, error_msg = result
        
        print(f"Summary: Prismatic={prismatic_length:.3f}, Time={calc_time:.3f}ms, Success={success}")
            
    except Exception as e:
        print(f"Error in Test 1: {e}")
    
    # Test case 2: Essential calculation with custom previous direction
    print("\n=== Test 2: Essential (Custom Previous Direction) ===")
    try:
        current_x, current_y, current_z = 1.0, 0.5, 0.8
        prev_x, prev_y, prev_z = 0.5, 0.3, 1.0
        result = calculate_segment_essential(current_x, current_y, current_z, 
                                           prev_x, prev_y, prev_z, debug=True)
        prismatic_length, calc_time, success, error_msg = result
        
        print(f"Summary: Prismatic={prismatic_length:.3f}, Time={calc_time:.3f}ms, Success={success}")
            
    except Exception as e:
        print(f"Error in Test 2: {e}")

def test_joint_based_methods():
    """Test joint-based segment calculation methods with J→S conversion"""
    print("\n" + "=" * 50)
    print("TESTING JOINT-BASED METHODS (J→S CONVERSION)")
    print("=" * 50)
    
    # Real FABRIK joint positions
    joint_positions = [
        [0.000, 0.000, 0.000],      # J0 - Base
        [0.000, 0.000, 73.591],     # J1 
        [-57.155, -28.577, 205.521], # J2
        [-109.723, -54.861, 338.912], # J3
        [-119.628, -59.814, 475.607], # J4
        [-29.336, -14.668, 553.110], # J5
        [78.418, 39.209, 504.765],   # J6
        [112.637, 56.319, 370.903],  # J7
        [99.999, 49.999, 299.992]    # J8 - Final
    ]
    
    max_possible_segments = len(joint_positions) - 1  # Should be 8 segments (0-7)
    
    # Test case 1: Essential calculation from joints - First segment
    print(f"\n=== Test 3: Essential from Joints (Segment 0) ===")
    try:
        segment_index = 0
        result = calculate_segment_essential_from_joints(joint_positions, segment_index, debug=True)
        prismatic_length, calc_time, success, error_msg, calc_seg_pos, calc_dir = result
        
        print(f"Summary: Segment {segment_index}, Prismatic={prismatic_length:.3f}, Success={success}")
            
    except Exception as e:
        print(f"Error in Test 3: {e}")
    
    # Test case 2: Essential calculation from joints - Middle segment
    print(f"\n=== Test 4: Essential from Joints (Segment 3) ===")
    try:
        segment_index = 3
        result = calculate_segment_essential_from_joints(joint_positions, segment_index, debug=True)
        prismatic_length, calc_time, success, error_msg, calc_seg_pos, calc_dir = result
        
        print(f"Summary: Segment {segment_index}, Prismatic={prismatic_length:.3f}, Success={success}")
            
    except Exception as e:
        print(f"Error in Test 4: {e}")
    
    # Test case 3: Essential calculation from joints - Last segment  
    print(f"\n=== Test 5: Essential from Joints (Last Segment {max_possible_segments-1}) ===")
    try:
        segment_index = max_possible_segments - 1  # Should be segment 7
        result = calculate_segment_essential_from_joints(joint_positions, segment_index, debug=True)
        prismatic_length, calc_time, success, error_msg, calc_seg_pos, calc_dir = result
        
        print(f"Summary: Segment {segment_index}, Prismatic={prismatic_length:.3f}, Success={success}")
        if success and calc_seg_pos is not None:
            print(f"Final S-Point: ({calc_seg_pos[0]:.3f}, {calc_seg_pos[1]:.3f}, {calc_seg_pos[2]:.3f})")
            print(f"Final Joint J8: ({joint_positions[-1][0]:.3f}, {joint_positions[-1][1]:.3f}, {joint_positions[-1][2]:.3f})")
            print(f"Should be equal for last segment (end-effector)")
            
    except Exception as e:
        print(f"Error in Test 5: {e}")
    
    # Test all segments in summary
    print(f"\n=== Test 6: All Segments Summary ===")
    print(f"Testing all {max_possible_segments} possible segments...")
    successful_segments = 0
    
    for segment_idx in range(max_possible_segments):
        try:
            result = calculate_segment_essential_from_joints(joint_positions, segment_idx, debug=False)
            prismatic_length, calc_time, success, error_msg, calc_seg_pos, calc_dir = result
            
            if success:
                successful_segments += 1
                print(f"  Segment {segment_idx}: ✓ Prismatic={prismatic_length:.3f}")
            else:
                print(f"  Segment {segment_idx}: ✗ Error: {error_msg}")
                
        except Exception as e:
            print(f"  Segment {segment_idx}: ✗ Exception: {e}")
    
    print(f"Success Rate: {successful_segments}/{max_possible_segments} ({100*successful_segments/max_possible_segments:.1f}%)")

def test_error_cases():
    """Test error handling"""
    print("\n" + "=" * 50)
    print("TESTING ERROR CASES")
    print("=" * 50)
    
    # Test case 1: Invalid direction (negative Z)
    print("\n=== Test 7: Invalid Direction (Negative Z) ===")
    try:
        result = calculate_segment_essential(0.5, 0.5, -0.1, debug=False)
        prismatic_length, calc_time, success, error_msg = result
        print(f"Expected failure: Success={success}, Error='{error_msg}'")
        
    except Exception as e:
        print(f"Exception caught: {e}")
    
    # Test case 2: Invalid segment index (too high)
    print("\n=== Test 8: Invalid Segment Index ===")
    try:
        valid_joints = [
            [0.0, 0.0, 0.0], [10.0, 0.0, 20.0], [20.0, 10.0, 40.0], 
            [30.0, 20.0, 60.0], [40.0, 30.0, 80.0]
        ]
        # Try to calculate segment 10 with only 5 joints (should fail)
        result = calculate_segment_essential_from_joints(valid_joints, 10, debug=False)
        prismatic_length, calc_time, success, error_msg, calc_seg_pos, calc_dir = result
        print(f"Expected failure: Success={success}, Error='{error_msg}'")
        
    except Exception as e:
        print(f"Exception caught: {e}")

def main():
    """Run all tests"""
    print("SEGMENT BLOCK COMPREHENSIVE TEST - ALL SEGMENTS")
    print("=" * 70)
    
    # Real FABRIK joint positions for visualization
    joint_positions = [
        [0.000, 0.000, 0.000],      # J0 - Base
        [0.000, 0.000, 73.591],     # J1 
        [-57.155, -28.577, 205.521], # J2
        [-109.723, -54.861, 338.912], # J3
        [-119.628, -59.814, 475.607], # J4
        [-29.336, -14.668, 553.110], # J5
        [78.418, 39.209, 504.765],   # J6
        [112.637, 56.319, 370.903],  # J7
        [99.999, 49.999, 299.992]    # J8 - Final
    ]
    
    print(f"\n" + "=" * 70)
    print("COMPREHENSIVE J→S VISUALIZATION - ALL SEGMENTS")
    print("=" * 70)
    print(f"Expected: {len(joint_positions)-1} segments from {len(joint_positions)} joints")
    
    # Show comprehensive visualization with ALL segments
    visualize_all_segments_and_joints(joint_positions)
    
    # Run all test categories
    test_direction_based_methods()
    test_joint_based_methods()
    test_error_cases()
    
    print("\n" + "=" * 70)
    print("ALL TESTS COMPLETED")
    print("=" * 70)

if __name__ == "__main__":
    main()