"""
FABRIK Python Wrapper with Prismatic Refinement
Enhanced FABRIK solving with physical accuracy
"""

import numpy as np
from base_plotter import create_base_3d_figure, add_robot_base_circle, ROBOT_COLORS
import plotly.graph_objects as go

import delta_robot_cpp

def test_fabrik_refinement(target_x: float, target_y: float, target_z: float, 
                          debug: bool = False):
    """
    Test enhanced FABRIK solver with prismatic refinement
    
    Args:
        target_x, target_y, target_z: Target position components
        debug: If True, show visualization and detailed output
        
    Returns:
        Refinement result tuple
    """
    
    try:
        target_position = np.array([target_x, target_y, target_z], dtype=float)
        
        # Call enhanced solver
        result = delta_robot_cpp.fabrik_solve_with_refinement(target_position)
        
        (final_joints, final_prismatic, fabrik_iters, refinement_iters,
         final_distance, converged, prismatic_converged, calc_time,
         success, error_msg, init_data, initial_joints, initial_prismatic) = result
        
        if not success:
            raise ValueError(f"Refinement failed: {error_msg}")
        
        if debug:
            print(f"\n=== FABRIK Refinement Results ===")
            print(f"Target: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})")
            print(f"FABRIK iterations: {fabrik_iters}")
            print(f"Refinement loops: {refinement_iters}")
            print(f"Final distance: {final_distance:.3f}mm")
            print(f"FABRIK converged: {converged}")
            print(f"Prismatic converged: {prismatic_converged}")
            print(f"Total time: {calc_time:.1f}ms")
            print(f"Final end-effector: ({final_joints[-1][0]:.1f}, {final_joints[-1][1]:.1f}, {final_joints[-1][2]:.1f})")
            print(f"Prismatic values: {[f'{p:.2f}' for p in final_prismatic]}")
            
            # Create visualization
            if len(final_joints) > 0:
                visualize_refinement_result(target_position, final_joints, final_prismatic,
                                          fabrik_iters, refinement_iters, converged,
                                          prismatic_converged, final_distance, calc_time)
        
        return result
        
    except Exception as e:
        raise ValueError(f"FABRIK refinement test failed: {str(e)}")

def visualize_refinement_result(target_position, final_joints, final_prismatic,
                               fabrik_iters, refinement_iters, converged,
                               prismatic_converged, final_distance, calc_time):
    """Visualize FABRIK refinement results"""
    
    try:
        ROBOT_RADIUS = delta_robot_cpp.ROBOT_RADIUS
        
        title = f"FABRIK Prismatic Refinement<br>"
        title += f"Target: ({target_position[0]:.0f}, {target_position[1]:.0f}, {target_position[2]:.0f}) | "
        title += f"FABRIK: {fabrik_iters} | Refinement: {refinement_iters} | Distance: {final_distance:.2f}mm"
        
        fig = create_base_3d_figure(title)
        add_robot_base_circle(fig, ROBOT_RADIUS)
        
        # Target
        fig.add_trace(go.Scatter3d(
            x=[target_position[0]], y=[target_position[1]], z=[target_position[2]],
            mode='markers+text', marker=dict(size=15, color='red', symbol='x'),
            text=['TARGET'], textposition='top center', name='Target'
        ))
        
        # Joint chain
        x_coords = [pos[0] for pos in final_joints]
        y_coords = [pos[1] for pos in final_joints]
        z_coords = [pos[2] for pos in final_joints]
        
        fig.add_trace(go.Scatter3d(
            x=x_coords, y=y_coords, z=z_coords,
            mode='lines+markers',
            line=dict(color=ROBOT_COLORS['joint_chain'], width=5),
            marker=dict(size=8, color=ROBOT_COLORS['joint_points']),
            name='Refined Chain'
        ))
        
        # End-effector highlight
        fig.add_trace(go.Scatter3d(
            x=[final_joints[-1][0]], y=[final_joints[-1][1]], z=[final_joints[-1][2]],
            mode='markers+text', marker=dict(size=12, color=ROBOT_COLORS['end_effector']),
            text=['END-EFF'], name='End-Effector'
        ))
        
        # Convergence status annotation
        status = f"FABRIK: {'✓' if converged else '✗'} | Prismatic: {'✓' if prismatic_converged else '✗'}"
        fig.add_annotation(x=0.02, y=0.98, xref="paper", yref="paper",
                          text=status, showarrow=False, font=dict(size=12))
        
        fig.show()
        
    except Exception as e:
        print(f"Visualization failed: {e}")

def compare_basic_vs_refinement(target_x: float, target_y: float, target_z: float):
    """Compare basic FABRIK vs refinement solver"""
    
    target_position = np.array([target_x, target_y, target_z], dtype=float)
    
    print(f"\n=== Basic vs Refinement Comparison ===")
    print(f"Target: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})")
    
    try:
        # Basic solver
        basic_result = delta_robot_cpp.fabrik_solve(target_position)
        basic_joints, basic_iters, basic_distance, basic_converged, basic_time, basic_success, _, _, _ = basic_result
        
        # Refinement solver
        refined_result = delta_robot_cpp.fabrik_solve_with_refinement(target_position)
        (refined_joints, prismatic_values, refined_fabrik_iters, refinement_iters,
         refined_distance, refined_converged, prismatic_converged, refined_time,
         refined_success, _, _, _, _) = refined_result
        
        if basic_success and refined_success:
            print(f"\nBasic FABRIK:")
            print(f"  Iterations: {basic_iters}, Distance: {basic_distance:.3f}mm, Time: {basic_time:.1f}ms")
            print(f"  End-effector: ({basic_joints[-1][0]:.1f}, {basic_joints[-1][1]:.1f}, {basic_joints[-1][2]:.1f})")
            
            print(f"\nRefinement FABRIK:")
            print(f"  FABRIK iters: {refined_fabrik_iters}, Refinement loops: {refinement_iters}")
            print(f"  Distance: {refined_distance:.3f}mm, Time: {refined_time:.1f}ms")
            print(f"  End-effector: ({refined_joints[-1][0]:.1f}, {refined_joints[-1][1]:.1f}, {refined_joints[-1][2]:.1f})")
            print(f"  Prismatic converged: {prismatic_converged}")
            
            # Compare results
            end_diff = np.linalg.norm(np.array(basic_joints[-1]) - np.array(refined_joints[-1]))
            distance_improvement = basic_distance - refined_distance
            
            print(f"\nComparison:")
            print(f"  End-effector difference: {end_diff:.3f}mm")
            print(f"  Distance improvement: {distance_improvement:.3f}mm")
            print(f"  Time ratio: {refined_time/basic_time:.1f}x")
            
            if abs(distance_improvement) > 0.01:
                print(f"  ✓ Refinement improved accuracy!")
            else:
                print(f"  ○ Similar accuracy achieved")
        
    except Exception as e:
        print(f"Comparison failed: {e}")

# Simple test function
if __name__ == "__main__":
    print("FABRIK Refinement Wrapper Test")
    print("=" * 40)
    
    # Single test target
    target = [80, 30, 400]
    
    print(f"Testing target: {target}")
    
    try:
        # Test refinement solver
        result = test_fabrik_refinement(target[0], target[1], target[2], debug=True)
        
        # Compare basic vs refinement
        compare_basic_vs_refinement(target[0], target[1], target[2])
        
        print(f"\n✓ Test completed successfully!")
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
    
    print("=" * 40)