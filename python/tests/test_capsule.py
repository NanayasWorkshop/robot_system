#!/usr/bin/env python3
"""
Capsule Creation Block Test Runner - Complete Pipeline Test
"""

import sys
sys.path.append('python/visualization')

from capsule_wrapper import create_capsule_chain, update_capsule_positions, visualize_capsule_chain
import delta_robot_cpp
import numpy as np

def test_pipeline_integration():
    """Test complete pipeline: Target → FABRIK → Joints → S-points → Capsules"""
    print("=" * 70)
    print("COMPLETE PIPELINE TEST: Target → FABRIK → Joints → S-points → Capsules")
    print("=" * 70)
    
    # Step 1: Set target and solve FABRIK
    target = [80.0, 30.0, 400.0]
    print(f"\nStep 1: FABRIK Solution")
    print(f"Target: ({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})")
    
    try:
        fabrik_result = delta_robot_cpp.fabrik_solve_with_refinement(
            target, 
            delta_robot_cpp.DEFAULT_ROBOT_SEGMENTS
        )
        
        if not fabrik_result[8]:  # solving_successful
            print(f"❌ FABRIK failed: {fabrik_result[9]}")
            return
            
        final_joints = fabrik_result[0]
        print(f"✓ FABRIK solved with {len(final_joints)} joints")
        print(f"  Final joint: ({final_joints[-1][0]:.3f}, {final_joints[-1][1]:.3f}, {final_joints[-1][2]:.3f})")
        print(f"  Target error: {np.linalg.norm(np.array(final_joints[-1]) - np.array(target)):.3f}mm")
        
    except Exception as e:
        print(f"❌ FABRIK error: {e}")
        return
    
    # Step 2: Extract S-points from joints
    print(f"\nStep 2: S-Point Extraction")
    num_segments = len(final_joints) - 1
    
    s_points = []
    successful_segments = 0
    
    for segment_idx in range(num_segments):
        try:
            segment_result = delta_robot_cpp.calculate_segment_essential_from_joints(
                final_joints, segment_idx
            )
            
            prismatic_length, calc_time, success, error_msg, calc_seg_pos, calc_dir = segment_result
            
            if success and calc_seg_pos is not None:
                s_points.append(calc_seg_pos)
                successful_segments += 1
            else:
                print(f"  ❌ Segment {segment_idx} failed: {error_msg}")
                
        except Exception as e:
            print(f"  ❌ Segment {segment_idx} error: {e}")
    
    if successful_segments == 0:
        print("❌ No S-points extracted")
        return
        
    print(f"✓ Extracted {len(s_points)} S-points from {successful_segments}/{num_segments} segments")
    print(f"  S0: ({s_points[0][0]:.3f}, {s_points[0][1]:.3f}, {s_points[0][2]:.3f})")
    print(f"  S{len(s_points)-1}: ({s_points[-1][0]:.3f}, {s_points[-1][1]:.3f}, {s_points[-1][2]:.3f})")
    
    # Step 3: Create capsule chain
    print(f"\nStep 3: Capsule Creation")
    robot_radius = delta_robot_cpp.ROBOT_RADIUS
    
    try:
        capsules, total_length, calc_time, success, error_msg = create_capsule_chain(
            s_points, robot_radius, debug=False
        )
        
        if not success:
            print(f"❌ Capsule creation failed: {error_msg}")
            return
            
        print(f"✓ Created {len(capsules)} capsules")
        print(f"  Robot radius: {robot_radius:.3f}mm")
        print(f"  Total chain length: {total_length:.3f}mm")
        print(f"  Creation time: {calc_time:.3f}ms")
        
        # Print individual capsules
        for i, capsule in enumerate(capsules):
            print(f"  Capsule {i+1}: Length={capsule['length']:.3f}mm")
        
    except Exception as e:
        print(f"❌ Capsule creation error: {e}")
        return
    
    # Step 4: Save results for next pipeline step
    print(f"\nStep 4: Save Pipeline Results")
    
    pipeline_data = {
        'target': target,
        'joints': final_joints,
        's_points': s_points,
        'capsules': capsules,
        'robot_radius': robot_radius,
        'total_chain_length': total_length
    }
    
    np.save('test_capsule_pipeline.npy', pipeline_data)
    print(f"✓ Saved complete pipeline data to 'test_capsule_pipeline.npy'")
    
    # Step 5: Visualization
    print(f"\nStep 5: Pipeline Visualization")
    try:
        visualize_capsule_chain(s_points, capsules, robot_radius)
        print("✓ Visualization complete")
        
    except Exception as e:
        print(f"❌ Visualization error: {e}")
    
    print(f"\n{'='*70}")
    print("PIPELINE TEST COMPLETED SUCCESSFULLY")
    print(f"{'='*70}")


def test_capsule_update():
    """Test capsule position update functionality"""
    print("\n" + "=" * 50)
    print("CAPSULE UPDATE TEST")
    print("=" * 50)
    
    # Load existing pipeline data
    try:
        pipeline_data = np.load('test_capsule_pipeline.npy', allow_pickle=True).item()
        initial_s_points = pipeline_data['s_points']
        robot_radius = pipeline_data['robot_radius']
        
        print("Step 1: Load existing capsules from pipeline")
        initial_capsules, _, _, success, error_msg = create_capsule_chain(
            initial_s_points, robot_radius, debug=False
        )
        
        if not success:
            print(f"❌ Initial creation failed: {error_msg}")
            return
            
        print(f"✓ Created {len(initial_capsules)} initial capsules")
        
    except FileNotFoundError:
        print("❌ Pipeline data not found. Run test_pipeline_integration() first.")
        return
    except Exception as e:
        print(f"❌ Initial creation error: {e}")
        return
    
    # Update with new target
    new_target = [100.0, 40.0, 350.0]
    print(f"\nStep 2: Solve for new target: ({new_target[0]:.1f}, {new_target[1]:.1f}, {new_target[2]:.1f})")
    
    try:
        new_fabrik_result = delta_robot_cpp.fabrik_solve_with_refinement(
            new_target, 
            delta_robot_cpp.DEFAULT_ROBOT_SEGMENTS
        )
        
        if not new_fabrik_result[8]:  # solving_successful
            print(f"❌ New FABRIK failed: {new_fabrik_result[9]}")
            return
            
        new_joints = new_fabrik_result[0]
        print(f"✓ New FABRIK solved")
        
        # Extract new S-points
        new_s_points = []
        num_segments = len(new_joints) - 1
        
        for segment_idx in range(num_segments):
            segment_result = delta_robot_cpp.calculate_segment_essential_from_joints(
                new_joints, segment_idx
            )
            
            prismatic_length, calc_time, success, error_msg, calc_seg_pos, calc_dir = segment_result
            
            if success and calc_seg_pos is not None:
                new_s_points.append(calc_seg_pos)
        
        print(f"✓ Extracted {len(new_s_points)} new S-points")
        
    except Exception as e:
        print(f"❌ New FABRIK error: {e}")
        return
    
    # Update capsule positions
    print(f"\nStep 3: Update capsule positions")
    try:
        updated_capsules, updated_length, update_time, update_success, update_error = update_capsule_positions(
            initial_capsules, new_s_points, debug=True
        )
        
        if not update_success:
            print(f"❌ Capsule update failed: {update_error}")
            return
            
        print(f"✓ Updated {len(updated_capsules)} capsules")
        print(f"  Update time: {update_time:.3f}ms")
        print(f"  New total length: {updated_length:.3f}mm")
        
    except Exception as e:
        print(f"❌ Capsule update error: {e}")


def test_error_cases():
    """Test error handling"""
    print("\n" + "=" * 50)
    print("ERROR HANDLING TEST")
    print("=" * 50)
    
    robot_radius = delta_robot_cpp.ROBOT_RADIUS
    
    # Test case 1: Insufficient S-points
    print("\n=== Test 1: Insufficient S-points ===")
    try:
        single_point = [[0.0, 0.0, 0.0]]
        result = create_capsule_chain(single_point, robot_radius, debug=False)
        capsules, total_length, calc_time, success, error_msg = result
        print(f"Expected failure: Success={success}, Error='{error_msg}'")
        
    except Exception as e:
        print(f"Exception caught: {e}")
    
    # Test case 2: Invalid radius
    print("\n=== Test 2: Invalid radius ===")
    try:
        valid_s_points = [[0.0, 0.0, 0.0], [10.0, 0.0, 20.0]]
        result = create_capsule_chain(valid_s_points, -5.0, debug=False)
        capsules, total_length, calc_time, success, error_msg = result
        print(f"Expected failure: Success={success}, Error='{error_msg}'")
        
    except Exception as e:
        print(f"Exception caught: {e}")
    
    # Test case 3: Degenerate S-points (duplicate points)
    print("\n=== Test 3: Degenerate S-points ===")
    try:
        duplicate_points = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        result = create_capsule_chain(duplicate_points, robot_radius, debug=False)
        capsules, total_length, calc_time, success, error_msg = result
        print(f"Expected failure: Success={success}, Error='{error_msg}'")
        
    except Exception as e:
        print(f"Exception caught: {e}")


def main():
    """Run all tests"""
    print("CAPSULE CREATION BLOCK COMPREHENSIVE TEST")
    print("=" * 70)
    
    # Run all test categories
    test_pipeline_integration()
    test_capsule_update()
    test_error_cases()
    
    print("\n" + "=" * 70)
    print("ALL TESTS COMPLETED")
    print("=" * 70)

if __name__ == "__main__":
    main()