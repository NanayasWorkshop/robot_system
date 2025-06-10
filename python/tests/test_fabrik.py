#!/usr/bin/env python3
"""
FABRIK Complete Pipeline Test Runner
Tests all three FABRIK blocks: Initialization, Iteration, and Solver
"""

import sys
sys.path.append('python/visualization')

from fabrik_wrapper import test_fabrik_solver, visualize_fabrik_result
import numpy as np

def test_fabrik_initialization():
    """Test FABRIK Initialization Block"""
    print("=" * 60)
    print("TESTING FABRIK INITIALIZATION BLOCK")
    print("=" * 60)
    
    try:
        import delta_robot_cpp
        
        # Test with default 7 segments
        print("\n=== Test 1: Default 7 Segments ===")
        result = delta_robot_cpp.create_fabrik_straight_chain(7)
        initial_joints, joint_distances, num_segments, num_joints, calc_time, success, error_msg = result
        
        if success:
            print(f"✓ Initialization successful!")
            print(f"  Segments: {num_segments}, Joints: {num_joints}")
            print(f"  Calculation time: {calc_time:.3f}ms")
            print(f"  Joint distances: {[f'{d:.1f}mm' for d in joint_distances]}")
            print(f"  First joint: {initial_joints[0]}")
            print(f"  Last joint: {initial_joints[-1]}")
            
            # Verify expected pattern
            expected_pattern = [73.0, 146.0, 146.0, 146.0, 146.0, 146.0, 146.0, 73.0]
            actual_distances = [round(d, 1) for d in joint_distances]
            if actual_distances == expected_pattern:
                print(f"✓ Joint distances match expected pattern!")
            else:
                print(f"⚠️  Joint distances don't match expected pattern")
                print(f"   Expected: {expected_pattern}")
                print(f"   Actual:   {actual_distances}")
        else:
            print(f"✗ Initialization failed: {error_msg}")
            
        # Test with different segment counts
        for segments in [3, 5, 10]:
            print(f"\n=== Test: {segments} Segments ===")
            result = delta_robot_cpp.create_fabrik_straight_chain(segments)
            _, _, num_segs, num_joints, calc_time, success, error_msg = result
            
            if success:
                expected_joints = segments + 2
                if num_joints == expected_joints:
                    print(f"✓ {segments} segments → {num_joints} joints (correct)")
                else:
                    print(f"✗ {segments} segments → {num_joints} joints (expected {expected_joints})")
            else:
                print(f"✗ Failed: {error_msg}")
                
    except Exception as e:
        print(f"✗ Test failed with exception: {e}")
        import traceback
        traceback.print_exc()

def test_fabrik_iteration():
    """Test FABRIK Iteration Block"""
    print("\n" + "=" * 60)
    print("TESTING FABRIK ITERATION BLOCK")
    print("=" * 60)
    
    try:
        import delta_robot_cpp
        
        # First get initial joints
        init_result = delta_robot_cpp.create_fabrik_straight_chain(7)
        initial_joints, joint_distances, num_segments, num_joints, _, init_success, _ = init_result
        
        if not init_success:
            print("✗ Cannot test iteration - initialization failed")
            return
            
        print(f"Using {num_joints} joints with {len(joint_distances)} distances")
        
        # Test targets
        test_targets = [
            [0, 0, 1000],      # Straight up (easy)
            [50, 30, 800],     # Moderate displacement
            [80, 30, 400],     # Your Python example
            [100, 100, 200],   # More challenging
        ]
        
        for i, target in enumerate(test_targets):
            print(f"\n=== Iteration Test {i+1}: Target {target} ===")
            
            try:
                result = delta_robot_cpp.fabrik_iterate(
                    initial_joints, 
                    np.array(target, dtype=float), 
                    joint_distances
                )
                updated_joints, distance_to_target, calc_time, success, error_msg = result
                
                if success:
                    print(f"✓ Iteration successful!")
                    print(f"  Distance to target: {distance_to_target:.3f}mm")
                    print(f"  Calculation time: {calc_time:.3f}ms")
                    print(f"  End-effector moved from {initial_joints[-1]} to {updated_joints[-1]}")
                    
                    # Check if improvement
                    initial_distance = np.linalg.norm(np.array(initial_joints[-1]) - np.array(target))
                    if distance_to_target < initial_distance:
                        print(f"✓ Improved: {initial_distance:.1f}mm → {distance_to_target:.1f}mm")
                    else:
                        print(f"⚠️  No improvement: {initial_distance:.1f}mm → {distance_to_target:.1f}mm")
                        
                else:
                    print(f"✗ Iteration failed: {error_msg}")
                    
            except Exception as e:
                print(f"✗ Exception during iteration: {e}")
                
    except Exception as e:
        print(f"✗ Test failed with exception: {e}")
        import traceback
        traceback.print_exc()

def test_fabrik_solver():
    """Test FABRIK Solver Block"""
    print("\n" + "=" * 60)
    print("TESTING FABRIK SOLVER BLOCK")
    print("=" * 60)
    
    try:
        import delta_robot_cpp
        
        # Test targets with expected outcomes
        test_cases = [
            {
                'target': [0, 0, 1000],
                'description': 'Straight up (should converge quickly)',
                'expected_convergence': True
            },
            {
                'target': [80, 30, 400], 
                'description': 'Python example target',
                'expected_convergence': True
            },
            {
                'target': [200, 200, 200],
                'description': 'Challenging but reachable',
                'expected_convergence': True  # Might not converge
            },
            {
                'target': [2000, 0, 0],
                'description': 'Likely unreachable',
                'expected_convergence': False
            }
        ]
        
        for i, test_case in enumerate(test_cases):
            target = test_case['target']
            print(f"\n=== Solver Test {i+1}: {test_case['description']} ===")
            print(f"Target: {target}")
            
            try:
                result = delta_robot_cpp.fabrik_solve(
                    np.array(target, dtype=float),
                    7,      # num_segments
                    0.01,   # tolerance  
                    50      # max_iterations
                )
                
                final_joints, iterations_used, final_distance, converged, calc_time, success, error_msg, init_data, initial_joints = result
                
                if success:
                    print(f"✓ Solving completed!")
                    print(f"  Converged: {converged} (expected: {test_case['expected_convergence']})")
                    print(f"  Iterations used: {iterations_used}")
                    print(f"  Final distance: {final_distance:.3f}mm")
                    print(f"  Calculation time: {calc_time:.3f}ms")
                    print(f"  Final end-effector: {final_joints[-1]}")
                    
                    # Validate convergence expectation
                    if converged == test_case['expected_convergence']:
                        print(f"✓ Convergence matches expectation")
                    else:
                        print(f"⚠️  Convergence unexpected (got {converged}, expected {test_case['expected_convergence']})")
                        
                    # Check if we're close to target when converged
                    if converged and final_distance <= 0.01:
                        print(f"✓ Converged within tolerance!")
                    elif converged:
                        print(f"⚠️  Converged but distance {final_distance:.3f} > tolerance 0.01")
                        
                else:
                    print(f"✗ Solving failed: {error_msg}")
                    
            except Exception as e:
                print(f"✗ Exception during solving: {e}")
        
        # Performance test
        print(f"\n=== Performance Test: Multiple Solves ===")
        target = [80, 30, 400]
        times = []
        
        for i in range(10):
            try:
                result = delta_robot_cpp.fabrik_solve(np.array(target, dtype=float))
                _, _, _, _, calc_time, success, _, _, _ = result
                if success:
                    times.append(calc_time)
            except:
                pass
                
        if times:
            avg_time = np.mean(times)
            min_time = np.min(times)
            max_time = np.max(times)
            print(f"✓ Performance over 10 runs:")
            print(f"  Average: {avg_time:.3f}ms")
            print(f"  Range: {min_time:.3f}ms - {max_time:.3f}ms")
        else:
            print(f"✗ No successful performance measurements")
                
    except Exception as e:
        print(f"✗ Test failed with exception: {e}")
        import traceback
        traceback.print_exc()

def main():
    """Run all FABRIK tests"""
    print("FABRIK COMPLETE PIPELINE TEST SUITE")
    print("=" * 70)
    print("Testing all three FABRIK blocks in sequence")
    
    # Test each block individually
    test_fabrik_initialization()
    test_fabrik_iteration() 
    test_fabrik_solver()
    
    # Integration test with visualization
    print("\n" + "=" * 70)
    print("INTEGRATION TEST WITH VISUALIZATION")
    print("=" * 70)
    
    try:
        # Test the complete pipeline with visualization
        target = [80, 30, 400]
        print(f"Testing complete pipeline with target {target}")
        
        # This should call the wrapper which uses the solver  
        # Note: test_fabrik_solver doesn't have debug parameter, use the wrapper directly
        from fabrik_wrapper import test_fabrik_solver as wrapper_test
        try:
            wrapper_test(target[0], target[1], target[2], debug=True)
        except ImportError:
            # Fallback to direct C++ call
            import delta_robot_cpp
            result = delta_robot_cpp.fabrik_solve(np.array(target, dtype=float))
            final_joints, iterations, distance, converged, calc_time, success, error, init_data, initial_joints = result
            print(f"✓ Direct C++ solve: converged={converged}, iterations={iterations}, distance={distance:.3f}mm")
        
        print("✓ Integration test with visualization completed!")
        
    except Exception as e:
        print(f"✗ Integration test failed: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n" + "=" * 70)
    print("ALL FABRIK TESTS COMPLETED")
    print("=" * 70)

if __name__ == "__main__":
    main()