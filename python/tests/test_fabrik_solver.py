#!/usr/bin/env python3
"""
FABRIK Solver Block Test Runner
Complete end-to-end FABRIK solving test
"""

print("=== Starting FABRIK Solver Test ===")

import sys
sys.path.append('python/visualization')

print("=== Importing modules ===")
try:
    import numpy as np
    print("✓ numpy imported")
    
    import delta_robot_cpp
    print("✓ delta_robot_cpp imported")
    
    from fabrik_solver_wrapper import calculate_fabrik_solver, visualize_fabrik_solver_result
    print("✓ fabrik_solver_wrapper imported")
    
except ImportError as e:
    print(f"✗ Import failed: {e}")
    sys.exit(1)

def analyze_chain_differences(result_7seg, result_5seg, target_position):
    """Quick analysis of why 7 vs 5 segments behave differently"""
    
    if not result_7seg or not result_5seg:
        print("Cannot analyze - missing results")
        return
    
    print(f"\n=== Chain Analysis: 7 vs 5 Segments ===")
    
    # Unpack results
    joints_7, achieved_7, conv_7, err_7, iter_7, time_7 = result_7seg
    joints_5, achieved_5, conv_5, err_5, iter_5, time_5 = result_5seg
    
    print(f"7 Segments: {len(joints_7)} joints, Error: {err_7:.2f}mm, Iterations: {iter_7}")
    print(f"5 Segments: {len(joints_5)} joints, Error: {err_5:.2f}mm, Iterations: {iter_5}")
    
    # Calculate total chain lengths
    def chain_length(joints):
        total = 0
        for i in range(len(joints) - 1):
            seg_len = np.linalg.norm(np.array(joints[i+1]) - np.array(joints[i]))
            total += seg_len
        return total
    
    len_7 = chain_length(joints_7)
    len_5 = chain_length(joints_5)
    
    print(f"Total chain length - 7seg: {len_7:.1f}mm, 5seg: {len_5:.1f}mm")
    
    # Check if target is reachable
    target_distance_from_base = np.linalg.norm(target_position)
    print(f"Target distance from origin: {target_distance_from_base:.1f}mm")
    print(f"7seg reach: {'✓' if len_7 >= target_distance_from_base else '✗'} ({len_7:.1f} vs {target_distance_from_base:.1f})")
    print(f"5seg reach: {'✓' if len_5 >= target_distance_from_base else '✗'} ({len_5:.1f} vs {target_distance_from_base:.1f})")
    
    # Compare final positions
    print(f"Final positions:")
    print(f"  7seg achieved: ({achieved_7[0]:.1f}, {achieved_7[1]:.1f}, {achieved_7[2]:.1f})")
    print(f"  5seg achieved: ({achieved_5[0]:.1f}, {achieved_5[1]:.1f}, {achieved_5[2]:.1f})")
    print(f"  Target:        ({target_position[0]:.1f}, {target_position[1]:.1f}, {target_position[2]:.1f})")

def main():
    print("\n=== Running FABRIK Solver Test ===")
    try:
        # Test target position
        target_position = np.array([80.0, 30.0, 500.0])
        print(f"Target Position: ({target_position[0]:.1f}, {target_position[1]:.1f}, {target_position[2]:.1f})")
        
        # Test 1: Default parameters (no initial joint positions)
        print("\n=== Test 1: FABRIK Solver with Default Parameters ===")
        result = calculate_fabrik_solver(target_position, debug=False)  # Don't show viz yet
        
        if result is not None:
            final_joints, achieved_pos, converged, final_error, total_iterations, solve_time = result
            
            print(f"\n=== FABRIK Solver Results ===")
            print(f"✓ Solver completed successfully!")
            print(f"Target Position: ({target_position[0]:.1f}, {target_position[1]:.1f}, {target_position[2]:.1f})")
            print(f"Achieved Position: ({achieved_pos[0]:.1f}, {achieved_pos[1]:.1f}, {achieved_pos[2]:.1f})")
            print(f"Converged: {'✓ Yes' if converged else '✗ No'}")
            print(f"Final Error: {final_error:.3f}mm")
            print(f"Total Iterations: {total_iterations}")
            print(f"Solve Time: {solve_time:.3f}ms")
            print(f"Total Joints: {len(final_joints)}")
            
            # Verify base at origin
            base_position = np.array(final_joints[0])
            base_at_origin = np.allclose(base_position, [0, 0, 0], atol=0.1)
            print(f"Base at Origin: {'✓ Yes' if base_at_origin else '✗ No'}")
            print(f"Final Base: ({base_position[0]:.3f}, {base_position[1]:.3f}, {base_position[2]:.3f})")
            
            # Analyze performance
            if converged:
                print(f"🎉 SUCCESS: Target reached within tolerance!")
                if total_iterations <= 10:
                    print(f"⚡ EXCELLENT: Fast convergence ({total_iterations} iterations)")
                elif total_iterations <= 25:
                    print(f"✓ GOOD: Reasonable convergence ({total_iterations} iterations)")
                else:
                    print(f"⚠️  SLOW: Many iterations needed ({total_iterations} iterations)")
            else:
                print(f"⚠️  Target not reached within tolerance, but got close ({final_error:.3f}mm)")
            
            # Test 2: Custom tolerance (looser)
            print(f"\n=== Test 2: FABRIK Solver with Looser Tolerance (1.0mm) ===")
            result2 = calculate_fabrik_solver(target_position, tolerance=1.0, debug=False)
            
            if result2 is not None:
                _, _, converged2, final_error2, total_iterations2, solve_time2 = result2
                print(f"Converged (1.0mm tolerance): {'✓ Yes' if converged2 else '✗ No'}")
                print(f"Final Error: {final_error2:.3f}mm")
                print(f"Iterations: {total_iterations2} (vs {total_iterations} with 0.01mm tolerance)")
                print(f"Solve Time: {solve_time2:.3f}ms")
                
                if converged2 and not converged:
                    print(f"✓ Looser tolerance helped convergence!")
                elif total_iterations2 < total_iterations:
                    print(f"✓ Looser tolerance reduced iterations!")
            
            # Test 3: Custom number of segments
            print(f"\n=== Test 3: FABRIK Solver with 5 Segments ===")
            result3 = calculate_fabrik_solver(target_position, num_robot_segments=5, debug=False)
            
            if result3 is not None:
                final_joints3, _, converged3, final_error3, total_iterations3, solve_time3 = result3
                print(f"Segments: 5 (vs default 7)")
                print(f"Total Joints: {len(final_joints3)}")
                print(f"Converged: {'✓ Yes' if converged3 else '✗ No'}")
                print(f"Final Error: {final_error3:.3f}mm")
                print(f"Iterations: {total_iterations3}")
                print(f"Solve Time: {solve_time3:.3f}ms")
                
                if solve_time3 < solve_time:
                    print(f"✓ Fewer segments solved faster!")
            
            print(f"\n=== Summary ===")
            print(f"Default (7 seg, 0.01mm): {final_error:.3f}mm in {total_iterations} iterations")
            if result2:
                print(f"Loose tolerance (1.0mm): {final_error2:.3f}mm in {total_iterations2} iterations")
            if result3:
                print(f"5 segments: {final_error3:.3f}mm in {total_iterations3} iterations")
            
            # Quick analysis of differences
            analyze_chain_differences(result, result3, target_position)
            
            # Now visualize the interesting cases
            print(f"\n=== Visualization Analysis ===")
            
            # Always visualize the 5-segment success case
            if result3:
                print("Visualizing 5-segment SUCCESS case:")
                final_joints3, achieved_pos3, converged3, final_error3, total_iterations3, solve_time3 = result3
                visualize_fabrik_solver_result(final_joints3, target_position, achieved_pos3, 
                                             converged3, final_error3, total_iterations3, solve_time3)
            
            # If 7-segment failed, visualize it for comparison
            if not converged:
                print("Visualizing 7-segment FAILED case:")
                visualize_fabrik_solver_result(final_joints, target_position, achieved_pos, 
                                             converged, final_error, total_iterations, solve_time)
            else:
                print("7-segment also succeeded, showing that case:")
                visualize_fabrik_solver_result(final_joints, target_position, achieved_pos, 
                                             converged, final_error, total_iterations, solve_time)
        
        else:
            print("✗ FABRIK Solver failed!")
        
        print("\n✓ All tests completed!")
        
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()