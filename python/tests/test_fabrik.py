#!/usr/bin/env python3
"""
FABRIK Prismatic Refinement Test
Tests the new enhanced FABRIK solver with prismatic refinement
"""

import sys
sys.path.append('python/visualization')

import numpy as np
from base_plotter import create_base_3d_figure, add_robot_base_circle, add_vector_arrow, ROBOT_COLORS
import plotly.graph_objects as go

def visualize_refinement_result(target_position, final_joints, final_prismatic, 
                               fabrik_iters, refinement_iters, converged, 
                               prismatic_converged, final_distance, calc_time):
    """Visualize FABRIK refinement results"""
    
    try:
        import delta_robot_cpp
        ROBOT_RADIUS = delta_robot_cpp.ROBOT_RADIUS
        
        # Create visualization
        title = f"FABRIK Prismatic Refinement - Target: ({target_position[0]:.0f}, {target_position[1]:.0f}, {target_position[2]:.0f})<br>"
        title += f"FABRIK: {fabrik_iters} iters, Refinement: {refinement_iters} loops, Distance: {final_distance:.3f}mm"
        fig = create_base_3d_figure(title)
        
        add_robot_base_circle(fig, ROBOT_RADIUS)
        
        # Target
        fig.add_trace(go.Scatter3d(
            x=[target_position[0]], y=[target_position[1]], z=[target_position[2]],
            mode='markers+text', marker=dict(size=15, color='red', symbol='x'),
            text=['TARGET'], name='Target'
        ))
        
        # Final joint chain
        final_x = [pos[0] for pos in final_joints]
        final_y = [pos[1] for pos in final_joints]
        final_z = [pos[2] for pos in final_joints]
        
        fig.add_trace(go.Scatter3d(
            x=final_x, y=final_y, z=final_z,
            mode='lines+markers+text',
            line=dict(color=ROBOT_COLORS['joint_chain'], width=5),
            marker=dict(size=8, color=ROBOT_COLORS['joint_points']),
            text=[f'J{i}' for i in range(len(final_joints))],
            name='Refined Chain'
        ))
        
        # End-effector
        fig.add_trace(go.Scatter3d(
            x=[final_joints[-1][0]], y=[final_joints[-1][1]], z=[final_joints[-1][2]],
            mode='markers+text', marker=dict(size=12, color=ROBOT_COLORS['end_effector']),
            text=['END-EFF'], name='End-Effector'
        ))
        
        fig.show()
        
    except Exception as e:
        print(f"Visualization failed: {e}")

def test_refinement_solver():
    """Test the new FABRIK solver with prismatic refinement"""
    print("=" * 60)
    print("TESTING FABRIK PRISMATIC REFINEMENT SOLVER")
    print("=" * 60)
    
    try:
        import delta_robot_cpp
        
        # Test target
        target = [80, 30, 400]
        print(f"Target: {target}")
        
        # Test basic solver first
        print(f"\n=== Basic FABRIK Solver ===")
        basic_result = delta_robot_cpp.fabrik_solve(np.array(target, dtype=float))
        basic_joints, basic_iters, basic_distance, basic_converged, basic_time, basic_success, basic_error, _, _ = basic_result
        
        if basic_success:
            print(f"✓ Basic: {basic_iters} iters, distance {basic_distance:.3f}mm, time {basic_time:.1f}ms")
        else:
            print(f"✗ Basic failed: {basic_error}")
            return
            
        # Test refinement solver
        print(f"\n=== Prismatic Refinement Solver ===")
        refined_result = delta_robot_cpp.fabrik_solve_with_refinement(np.array(target, dtype=float))
        
        (final_joints, final_prismatic, fabrik_iters, refinement_iters, 
         final_distance, converged, prismatic_converged, calc_time,
         success, error_msg, init_data, initial_joints, initial_prismatic) = refined_result
        
        if success:
            print(f"✓ Refinement successful!")
            print(f"  FABRIK iterations: {fabrik_iters}")
            print(f"  Refinement loops: {refinement_iters}")
            print(f"  Final distance: {final_distance:.3f}mm")
            print(f"  FABRIK converged: {converged}")
            print(f"  Prismatic converged: {prismatic_converged}")
            print(f"  Total time: {calc_time:.1f}ms")
            
            # Show prismatic values
            print(f"  Prismatic lengths: {[f'{p:.2f}mm' for p in final_prismatic]}")
            
            # Compare with basic solver
            basic_end = np.array(basic_joints[-1])
            refined_end = np.array(final_joints[-1])
            improvement = np.linalg.norm(basic_end - refined_end)
            
            print(f"\n=== Comparison ===")
            print(f"Basic end-effector: ({basic_end[0]:.1f}, {basic_end[1]:.1f}, {basic_end[2]:.1f})")
            print(f"Refined end-effector: ({refined_end[0]:.1f}, {refined_end[1]:.1f}, {refined_end[2]:.1f})")
            print(f"Position difference: {improvement:.3f}mm")
            print(f"Distance improvement: {basic_distance:.3f}mm → {final_distance:.3f}mm")
            
            if improvement > 0.1:
                print(f"✓ Refinement made a difference!")
            else:
                print(f"○ Refinement converged to similar result")
                
            # Visualize
            visualize_refinement_result(target, final_joints, final_prismatic,
                                      fabrik_iters, refinement_iters, converged,
                                      prismatic_converged, final_distance, calc_time)
            
        else:
            print(f"✗ Refinement failed: {error_msg}")
            
        # Test segment calculations
        print(f"\n=== Segment Calculation Test ===")
        for i in range(min(3, len(final_joints)-1)):  # Test first 3 segments
            try:
                seg_result = delta_robot_cpp.calculate_segment_essential_from_joints(final_joints, i)
                prismatic, seg_time, seg_success, seg_error, calc_pos, calc_dir = seg_result
                
                if seg_success:
                    print(f"✓ Segment {i}: prismatic {prismatic:.2f}mm (time {seg_time:.3f}ms)")
                else:
                    print(f"✗ Segment {i} failed: {seg_error}")
            except Exception as e:
                print(f"✗ Segment {i} exception: {e}")
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()

def test_constants():
    """Test that new constants are available"""
    print(f"\n=== Constants Test ===")
    try:
        import delta_robot_cpp
        
        print(f"FABRIK_TOLERANCE: {delta_robot_cpp.FABRIK_TOLERANCE}")
        print(f"FABRIK_MAX_ITERATIONS: {delta_robot_cpp.FABRIK_MAX_ITERATIONS}")
        print(f"FABRIK_PRISMATIC_TOLERANCE: {delta_robot_cpp.FABRIK_PRISMATIC_TOLERANCE}")
        print(f"FABRIK_MAX_REFINEMENT_ITERATIONS: {delta_robot_cpp.FABRIK_MAX_REFINEMENT_ITERATIONS}")
        print(f"✓ All constants available")
        
    except Exception as e:
        print(f"✗ Constants test failed: {e}")

def main():
    """Run FABRIK prismatic refinement tests"""
    print("FABRIK PRISMATIC REFINEMENT TEST SUITE")
    print("=" * 60)
    
    test_constants()
    test_refinement_solver()
    
    print(f"\n" + "=" * 60)
    print("PRISMATIC REFINEMENT TESTS COMPLETED")
    print("=" * 60)

if __name__ == "__main__":
    main()