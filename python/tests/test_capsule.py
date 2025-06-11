#!/usr/bin/env python3
"""
Capsule Creation Block Test Runner - FIXED VERSION
Complete Pipeline Test with proper debugging
"""

import sys
sys.path.append('python/visualization')

# Test imports first
try:
    from capsule_wrapper import create_capsule_chain, update_capsule_positions, visualize_capsule_chain
    print("✓ Successfully imported capsule wrapper functions")
except ImportError as e:
    print(f"❌ Failed to import capsule wrapper: {e}")
    sys.exit(1)

try:
    import delta_robot_cpp
    print("✓ Successfully imported delta_robot_cpp")
except ImportError as e:
    print(f"❌ Failed to import delta_robot_cpp: {e}")
    sys.exit(1)

import numpy as np

def are_points_equal(point1, point2, tolerance=1e-6):
    """Check if two 3D points are equal within tolerance"""
    return (abs(point1[0] - point2[0]) < tolerance and 
            abs(point1[1] - point2[1]) < tolerance and 
            abs(point1[2] - point2[2]) < tolerance)

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
                # Convert to regular Python list for consistency
                s_point = [float(calc_seg_pos[0]), float(calc_seg_pos[1]), float(calc_seg_pos[2])]
                
                # FIXED: Check for duplicate consecutive S-points
                if len(s_points) == 0 or not are_points_equal(s_points[-1], s_point, tolerance=1e-6):
                    s_points.append(s_point)
                    successful_segments += 1
                    print(f"  ✓ S{len(s_points)-1}: ({s_point[0]:.3f}, {s_point[1]:.3f}, {s_point[2]:.3f})")
                else:
                    print(f"  ○ S{segment_idx}: Duplicate point, skipped ({s_point[0]:.3f}, {s_point[1]:.3f}, {s_point[2]:.3f})")
            else:
                print(f"  ❌ Segment {segment_idx} failed: {error_msg}")
                
        except Exception as e:
            print(f"  ❌ Segment {segment_idx} error: {e}")
    
    if len(s_points) < 2:
        print(f"❌ Insufficient unique S-points: {len(s_points)} (need at least 2)")
        return
        
    print(f"✓ Extracted {len(s_points)} unique S-points from {successful_segments} successful segments")
    
    # Step 3: Create capsule chain
    print(f"\nStep 3: Capsule Creation")
    robot_radius = delta_robot_cpp.ROBOT_RADIUS
    
    print(f"Debug info:")
    print(f"  Number of S-points: {len(s_points)}")
    print(f"  S-points type: {type(s_points)}")
    print(f"  First S-point: {s_points[0]}")
    print(f"  Robot radius: {robot_radius}")
    
    try:
        # FIXED: Call the function with proper debugging
        capsules, total_length, calc_time, success, error_msg = create_capsule_chain(
            s_points, robot_radius, debug=True
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
        import traceback
        traceback.print_exc()
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
                # Convert to regular Python list for consistency
                s_point = [float(calc_seg_pos[0]), float(calc_seg_pos[1]), float(calc_seg_pos[2])]
                
                # FIXED: Check for duplicate consecutive S-points
                if len(new_s_points) == 0 or not are_points_equal(new_s_points[-1], s_point, tolerance=1e-6):
                    new_s_points.append(s_point)
        
        print(f"✓ Extracted {len(new_s_points)} unique new S-points")
        
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


def test_simple_capsule_creation():
    """Test capsule creation with simple data to isolate the issue"""
    print("\n" + "=" * 50)
    print("SIMPLE CAPSULE CREATION TEST")
    print("=" * 50)
    
    # Simple test data
    simple_s_points = [
        [0.0, 0.0, 0.0],
        [10.0, 0.0, 20.0],
        [20.0, 10.0, 40.0]
    ]
    robot_radius = 25.0
    
    print(f"Testing with {len(simple_s_points)} simple S-points:")
    for i, point in enumerate(simple_s_points):
        print(f"  S{i}: {point}")
    
    try:
        result = create_capsule_chain(simple_s_points, robot_radius, debug=True)
        capsules, total_length, calc_time, success, error_msg = result
        
        if success:
            print(f"✓ Simple test succeeded: {len(capsules)} capsules, {total_length:.3f}mm total")
        else:
            print(f"❌ Simple test failed: {error_msg}")
            
    except Exception as e:
        print(f"❌ Simple test exception: {e}")
        import traceback
        traceback.print_exc()


def main():
    """Run all tests"""
    print("CAPSULE CREATION BLOCK COMPREHENSIVE TEST (FIXED VERSION)")
    print("=" * 70)
    
    # Run simple test first to isolate any issues
    test_simple_capsule_creation()
    
    # Run all test categories
    test_pipeline_integration()
    test_capsule_update()
    test_error_cases()
    
    print("\n" + "=" * 70)
    print("ALL TESTS COMPLETED")
    print("=" * 70)

if __name__ == "__main__":
    main()