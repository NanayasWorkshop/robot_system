#!/usr/bin/env python3
"""
Test SegmentBlock with actual FABRIK backward joint positions
to isolate if the issue is in SegmentBlock or FabrikForwardBlock
"""

import sys
sys.path.append('python/visualization')

from segment_wrapper import (
    calculate_segment_essential_from_joints,
    visualize_all_segments_and_joints
)
import numpy as np
import delta_robot_cpp

def test_segment_with_fabrik_backward_joints():
    """Test SegmentBlock with the exact joint positions from FABRIK backward iteration"""
    
    print("=" * 70)
    print("TESTING SEGMENTBLOCK WITH FABRIK BACKWARD JOINT POSITIONS")
    print("=" * 70)
    
    # These are the joint positions after FABRIK backward pass from your test output
    # (the ones that cause issues in FabrikForwardBlock)
    backward_joint_positions = [
        [-32.5, -12.2, 161.5],      # J0 - Base (drifted from origin)
        [-32.5, -12.2, 235.0],     # J1 
        [-89.7, -40.8, 367.0],     # J2
        [-142.2, -69.1, 500.4],    # J3 (approximate)
        [-152.1, -74.0, 637.1],    # J4 (approximate)
        [-61.8, -26.9, 714.6],     # J5 (approximate)
        [45.9, 27.0, 666.3],       # J6 (approximate)
        [80.1, 44.1, 532.4],       # J7 (approximate)
        [80.0, 30.0, 500.0]        # J8 - End-effector at target
    ]
    
    print(f"Using {len(backward_joint_positions)} joint positions from FABRIK backward pass")
    print(f"Base position (should be drifted): ({backward_joint_positions[0][0]:.1f}, {backward_joint_positions[0][1]:.1f}, {backward_joint_positions[0][2]:.1f})")
    print(f"End-effector position (should be at target): ({backward_joint_positions[-1][0]:.1f}, {backward_joint_positions[-1][1]:.1f}, {backward_joint_positions[-1][2]:.1f})")
    
    max_segments = len(backward_joint_positions) - 1  # Should be 8 segments
    
    print(f"\n=== Testing All {max_segments} Segments ===")
    
    # Calculate original segment lengths from these joint positions
    original_segment_lengths = []
    for i in range(len(backward_joint_positions) - 1):
        length = np.linalg.norm(np.array(backward_joint_positions[i+1]) - np.array(backward_joint_positions[i]))
        original_segment_lengths.append(length)
    
    print(f"Original FABRIK segment lengths: {[f'{l:.1f}mm' for l in original_segment_lengths]}")
    
    # Test each segment calculation
    segment_results = []
    successful_calculations = 0
    
    for segment_idx in range(max_segments):
        print(f"\n--- Testing Segment {segment_idx} (S{segment_idx+1}) ---")
        
        try:
            result = calculate_segment_essential_from_joints(
                backward_joint_positions, segment_idx, debug=True
            )
            prismatic_length, calc_time, success, error_msg, calc_seg_pos, calc_dir = result
            
            if success:
                successful_calculations += 1
                segment_results.append({
                    'index': segment_idx,
                    'prismatic_length': prismatic_length,
                    'segment_position': calc_seg_pos,
                    'direction': calc_dir,
                    'success': True
                })
                
                # Calculate what FABRIK length this would become
                # (using the same conversion logic from FabrikForwardBlock)
                fabrik_length = convert_prismatic_to_fabrik_length(
                    prismatic_length, segment_idx, max_segments - 1
                )
                
                print(f"✓ Segment {segment_idx}: Prismatic={prismatic_length:.3f} → FABRIK={fabrik_length:.1f}mm")
                print(f"  Original FABRIK length: {original_segment_lengths[segment_idx]:.1f}mm")
                print(f"  Length change: {abs(fabrik_length - original_segment_lengths[segment_idx]):.1f}mm")
                
                if calc_seg_pos is not None:
                    print(f"  S-Point: ({calc_seg_pos[0]:.1f}, {calc_seg_pos[1]:.1f}, {calc_seg_pos[2]:.1f})")
                if calc_dir is not None:
                    print(f"  Direction: ({calc_dir[0]:.3f}, {calc_dir[1]:.3f}, {calc_dir[2]:.3f})")
                
            else:
                segment_results.append({
                    'index': segment_idx,
                    'success': False,
                    'error': error_msg
                })
                print(f"✗ Segment {segment_idx}: Failed - {error_msg}")
                
        except Exception as e:
            print(f"✗ Segment {segment_idx}: Exception - {e}")
            segment_results.append({
                'index': segment_idx,
                'success': False,
                'error': str(e)
            })
    
    print(f"\n=== Summary ===")
    print(f"Successful calculations: {successful_calculations}/{max_segments}")
    
    # Calculate new FABRIK lengths and compare
    new_fabrik_lengths = []
    length_changes = []
    
    for result in segment_results:
        if result['success']:
            fabrik_length = convert_prismatic_to_fabrik_length(
                result['prismatic_length'], result['index'], max_segments - 1
            )
            new_fabrik_lengths.append(fabrik_length)
            
            original_length = original_segment_lengths[result['index']]
            change = abs(fabrik_length - original_length)
            length_changes.append(change)
        else:
            new_fabrik_lengths.append(0.0)  # Placeholder for failed calculations
            length_changes.append(0.0)
    
    if length_changes:
        max_change = max(length_changes)
        avg_change = np.mean([c for c in length_changes if c > 0])
        
        print(f"\nSegment Length Analysis:")
        print(f"Original: {[f'{l:.1f}mm' for l in original_segment_lengths]}")
        print(f"New:      {[f'{l:.1f}mm' for l in new_fabrik_lengths if l > 0]}")
        print(f"Max change: {max_change:.1f}mm")
        print(f"Avg change: {avg_change:.1f}mm")
        
        # Check if we get the same dramatic changes as in FABRIK forward
        if max_change > 100:
            print(f"⚠️  WARNING: Large segment length changes detected ({max_change:.1f}mm)")
            print("   This matches the issue seen in FABRIK forward block!")
        else:
            print(f"✓ Segment length changes are reasonable ({max_change:.1f}mm)")
    
    # Visualize the results
    print(f"\n=== Visualization ===")
    try:
        visualize_all_segments_and_joints(backward_joint_positions)
    except Exception as e:
        print(f"Visualization failed: {e}")
    
    return segment_results

def convert_prismatic_to_fabrik_length(prismatic_length, segment_index, total_segments):
    """
    Convert prismatic length to FABRIK segment length using the same logic as FabrikForwardBlock
    """
    # Get constants from C++
    MIN_HEIGHT = delta_robot_cpp.MIN_HEIGHT
    MOTOR_LIMIT = delta_robot_cpp.MOTOR_LIMIT  
    WORKING_HEIGHT = delta_robot_cpp.WORKING_HEIGHT
    
    # H→G distance from prismatic length
    h_to_g_distance = MIN_HEIGHT + 2.0 * MOTOR_LIMIT + prismatic_length
    
    # Convert to FABRIK segment length using the same pattern as initialization
    if segment_index == 0:
        # First FABRIK segment: WORKING_HEIGHT + h_to_g_distance/2
        return WORKING_HEIGHT + h_to_g_distance / 2.0
    elif segment_index == total_segments:
        # Last FABRIK segment: h_to_g_distance/2 + WORKING_HEIGHT
        return h_to_g_distance / 2.0 + WORKING_HEIGHT
    else:
        # Middle FABRIK segments: h_to_g_distance/2 + 2*WORKING_HEIGHT + h_to_g_distance/2
        return h_to_g_distance / 2.0 + 2.0 * WORKING_HEIGHT + h_to_g_distance / 2.0

def main():
    """Run the test"""
    print("SEGMENT BLOCK TEST WITH FABRIK BACKWARD JOINT POSITIONS")
    print("Testing if SegmentBlock itself causes the large segment length changes")
    
    try:
        results = test_segment_with_fabrik_backward_joints()
        
        # Count successful vs failed calculations
        successful = sum(1 for r in results if r['success'])
        total = len(results)
        
        print(f"\n" + "=" * 70)
        print(f"TEST COMPLETED: {successful}/{total} segments calculated successfully")
        
        if successful == total:
            print("✓ All segment calculations successful!")
        else:
            print(f"⚠️  {total - successful} segment calculations failed")
            
        print("=" * 70)
        
    except Exception as e:
        print(f"Test failed with error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()