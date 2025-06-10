#!/usr/bin/env python3
"""
Segment Block Test Runner
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
        
        print(f"Prismatic Length: {prismatic_length:.3f}")
        print(f"Calculation Time: {calc_time:.3f}ms")
        print(f"Success: {success}")
        if not success:
            print(f"Error: {error_msg}")
            
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
        
        print(f"Prismatic Length: {prismatic_length:.3f}")
        print(f"Calculation Time: {calc_time:.3f}ms")
        print(f"Success: {success}")
        if not success:
            print(f"Error: {error_msg}")
            
    except Exception as e:
        print(f"Error in Test 2: {e}")
    
    # Test case 3: Complete calculation
    print("\n=== Test 3: Complete Analysis ===")
    try:
        current_x, current_y, current_z = 0.1, 0.2, 1.0
        result = calculate_segment_complete(current_x, current_y, current_z, debug=True)
        prismatic_length, calc_time, success, error_msg, has_kinematics, has_joint_state, has_orientation = result
        
        print(f"Prismatic Length: {prismatic_length:.3f}")
        print(f"Calculation Time: {calc_time:.3f}ms")
        print(f"Success: {success}")
        print(f"Has Kinematics Data: {has_kinematics}")
        print(f"Has Joint State Data: {has_joint_state}")
        print(f"Has Orientation Data: {has_orientation}")
        if not success:
            print(f"Error: {error_msg}")
            
    except Exception as e:
        print(f"Error in Test 3: {e}")

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
    
    # Test case 1: Essential calculation from joints - Segment 0 (S1)
    print("\n=== Test 4: Essential from Joints (Segment 0) ===")
    try:
        segment_index = 0
        result = calculate_segment_essential_from_joints(joint_positions, segment_index, debug=True)
        prismatic_length, calc_time, success, error_msg, calc_seg_pos, calc_dir = result
        
        print(f"Segment Index: {segment_index}")
        print(f"Prismatic Length: {prismatic_length:.3f}")
        print(f"Calculation Time: {calc_time:.3f}ms")
        print(f"Success: {success}")
        if success and calc_seg_pos is not None:
            print(f"Calculated S-Point: ({calc_seg_pos[0]:.3f}, {calc_seg_pos[1]:.3f}, {calc_seg_pos[2]:.3f})")
            print(f"Calculated Direction: ({calc_dir[0]:.3f}, {calc_dir[1]:.3f}, {calc_dir[2]:.3f})")
        if not success:
            print(f"Error: {error_msg}")
            
    except Exception as e:
        print(f"Error in Test 4: {e}")
    
    # Test case 2: Essential calculation from joints - Segment 2 (S3)  
    print("\n=== Test 5: Essential from Joints (Segment 2) ===")
    try:
        segment_index = 2
        result = calculate_segment_essential_from_joints(joint_positions, segment_index, debug=True)
        prismatic_length, calc_time, success, error_msg, calc_seg_pos, calc_dir = result
        
        print(f"Segment Index: {segment_index}")
        print(f"Prismatic Length: {prismatic_length:.3f}")
        print(f"Calculation Time: {calc_time:.3f}ms")
        print(f"Success: {success}")
        if success and calc_seg_pos is not None:
            print(f"Calculated S-Point: ({calc_seg_pos[0]:.3f}, {calc_seg_pos[1]:.3f}, {calc_seg_pos[2]:.3f})")
            print(f"Calculated Direction: ({calc_dir[0]:.3f}, {calc_dir[1]:.3f}, {calc_dir[2]:.3f})")
        if not success:
            print(f"Error: {error_msg}")
            
    except Exception as e:
        print(f"Error in Test 5: {e}")
    
    # Test case 3: Essential calculation from joints - Segment 4 (S5) - Mid-chain
    print("\n=== Test 5b: Essential from Joints (Segment 4) ===")
    try:
        segment_index = 4
        result = calculate_segment_essential_from_joints(joint_positions, segment_index, debug=True)
        prismatic_length, calc_time, success, error_msg, calc_seg_pos, calc_dir = result
        
        print(f"Segment Index: {segment_index}")
        print(f"Prismatic Length: {prismatic_length:.3f}")
        print(f"Calculation Time: {calc_time:.3f}ms")
        print(f"Success: {success}")
        if success and calc_seg_pos is not None:
            print(f"Calculated S-Point: ({calc_seg_pos[0]:.3f}, {calc_seg_pos[1]:.3f}, {calc_seg_pos[2]:.3f})")
            print(f"Calculated Direction: ({calc_dir[0]:.3f}, {calc_dir[1]:.3f}, {calc_dir[2]:.3f})")
        if not success:
            print(f"Error: {error_msg}")
            
    except Exception as e:
        print(f"Error in Test 5b: {e}")
    
    # Test case 3: Complete calculation from joints
    print("\n=== Test 6: Complete from Joints (Segment 0) ===")
    try:
        segment_index = 0
        result = calculate_segment_complete_from_joints(joint_positions, segment_index, debug=True)
        prismatic_length, calc_time, success, error_msg, calc_seg_pos, calc_dir, has_kinematics, has_joint_state, has_orientation = result
        
        print(f"Segment Index: {segment_index}")
        print(f"Prismatic Length: {prismatic_length:.3f}")
        print(f"Calculation Time: {calc_time:.3f}ms")
        print(f"Success: {success}")
        print(f"Has Kinematics Data: {has_kinematics}")
        print(f"Has Joint State Data: {has_joint_state}")
        print(f"Has Orientation Data: {has_orientation}")
        if success and calc_seg_pos is not None:
            print(f"Calculated S-Point: ({calc_seg_pos[0]:.3f}, {calc_seg_pos[1]:.3f}, {calc_seg_pos[2]:.3f})")
            print(f"Calculated Direction: ({calc_dir[0]:.3f}, {calc_dir[1]:.3f}, {calc_dir[2]:.3f})")
        if not success:
            print(f"Error: {error_msg}")
            
    except Exception as e:
        print(f"Error in Test 6: {e}")

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
        print(f"Success: {success}")
        print(f"Error Message: {error_msg}")
        
    except Exception as e:
        print(f"Exception caught: {e}")
    
    # Test case 2: Insufficient joint positions
    print("\n=== Test 8: Insufficient Joint Positions ===")
    try:
        insufficient_joints = [
            [0.0, 0.0, 0.0],
            [5.0, 5.0, 10.0]
        ]
        result = calculate_segment_essential_from_joints(insufficient_joints, 0, debug=False)
        prismatic_length, calc_time, success, error_msg, calc_seg_pos, calc_dir = result
        print(f"Success: {success}")
        print(f"Error Message: {error_msg}")
        
    except Exception as e:
        print(f"Exception caught: {e}")

def main():
    """Run all tests"""
    print("SEGMENT BLOCK COMPREHENSIVE TEST")
    print("=" * 60)
    
    # First show comprehensive visualization
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
    
    print("\n" + "=" * 60)
    print("COMPREHENSIVE J→S VISUALIZATION")
    print("=" * 60)
    visualize_all_segments_and_joints(joint_positions, max_segments=5)
    
    # Run all test categories
    test_direction_based_methods()
    test_joint_based_methods()
    test_error_cases()
    
    print("\n" + "=" * 60)
    print("ALL TESTS COMPLETED")
    print("=" * 60)

if __name__ == "__main__":
    main()