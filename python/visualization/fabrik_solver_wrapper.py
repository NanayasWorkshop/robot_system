"""
FABRIK Solver Block Python Wrapper
Complete FABRIK solving with visualization
"""

import numpy as np
import delta_robot_cpp

try:
    from base_plotter import create_base_3d_figure, add_robot_base_circle, add_vector_arrow, ROBOT_COLORS
    import plotly.graph_objects as go
    VISUALIZATION_AVAILABLE = True
except ImportError as e:
    print(f"Visualization not available: {e}")
    VISUALIZATION_AVAILABLE = False

def visualize_fabrik_solver_result(final_joint_positions: list, target_position: np.ndarray,
                                  achieved_position: np.ndarray, converged: bool, final_error: float,
                                  total_iterations: int, solve_time_ms: float) -> None:
    """Visualize complete FABRIK solving results"""
    
    if not VISUALIZATION_AVAILABLE:
        print("Visualization not available")
        return
    
    # Get constants from C++
    ROBOT_RADIUS = delta_robot_cpp.ROBOT_RADIUS
    
    # Create base figure
    convergence_status = "CONVERGED" if converged else "MAX ITERATIONS"
    num_segments = len(final_joint_positions) - 1
    fig = create_base_3d_figure(
        f"FABRIK Solver Results ({num_segments} segments) - {convergence_status}<br>"
        f"Target: ({target_position[0]:.1f}, {target_position[1]:.1f}, {target_position[2]:.1f}), "
        f"Error: {final_error:.2f}mm<br>"
        f"Iterations: {total_iterations}, Time: {solve_time_ms:.2f}ms"
    )
    
    # Add robot base circle for reference
    add_robot_base_circle(fig, ROBOT_RADIUS)
    
    # Convert to numpy arrays
    joint_positions = [np.array(pos) for pos in final_joint_positions]
    
    # Calculate segment lengths for analysis
    segment_lengths = []
    for i in range(len(joint_positions) - 1):
        length = np.linalg.norm(joint_positions[i+1] - joint_positions[i])
        segment_lengths.append(length)
    
    print(f"\nSegment Length Analysis ({len(segment_lengths)} segments):")
    for i, length in enumerate(segment_lengths):
        print(f"  Segment {i}: {length:.1f}mm")
    print(f"  Length variation: {max(segment_lengths) - min(segment_lengths):.1f}mm")
    print(f"  Average length: {np.mean(segment_lengths):.1f}mm")
    
    # Extract coordinates
    joint_x = [pos[0] for pos in joint_positions]
    joint_y = [pos[1] for pos in joint_positions]
    joint_z = [pos[2] for pos in joint_positions]
    
    # Plot final joint chain
    fig.add_trace(go.Scatter3d(
        x=joint_x,
        y=joint_y,
        z=joint_z,
        mode='lines+markers',
        line=dict(color=ROBOT_COLORS['joint_chain'], width=6),
        marker=dict(size=10, color=ROBOT_COLORS['joint_points']),
        name='Final Joint Chain'
    ))
    
    # Highlight base joint (should be at origin)
    base_position = joint_positions[0]
    fig.add_trace(go.Scatter3d(
        x=[base_position[0]],
        y=[base_position[1]],
        z=[base_position[2]],
        mode='markers+text',
        marker=dict(size=15, color='green', symbol='diamond'),
        text=['Base (Origin)'],
        textposition='bottom center',
        name='Base Joint'
    ))
    
    # Highlight end-effector
    end_effector = joint_positions[-1]
    fig.add_trace(go.Scatter3d(
        x=[end_effector[0]],
        y=[end_effector[1]],
        z=[end_effector[2]],
        mode='markers+text',
        marker=dict(size=15, color='blue', symbol='circle'),
        text=['End-Effector'],
        textposition='top center',
        name='End-Effector'
    ))
    
    # Target position
    fig.add_trace(go.Scatter3d(
        x=[target_position[0]],
        y=[target_position[1]],
        z=[target_position[2]],
        mode='markers+text',
        marker=dict(size=18, color='red', symbol='x'),
        text=['TARGET'],
        textposition='top center',
        name='Target Position'
    ))
    
    # Error vector from end-effector to target
    if final_error > 0.1:  # Only show if significant error
        fig.add_trace(go.Scatter3d(
            x=[end_effector[0], target_position[0]],
            y=[end_effector[1], target_position[1]],
            z=[end_effector[2], target_position[2]],
            mode='lines',
            line=dict(color='red', width=4, dash='dash'),
            name=f'Error Vector ({final_error:.2f}mm)'
        ))
    
    # Add joint labels
    joint_labels = [f'J{i}' for i in range(len(joint_positions))]
    fig.add_trace(go.Scatter3d(
        x=joint_x,
        y=joint_y,
        z=joint_z,
        mode='text',
        text=joint_labels,
        textposition='middle right',
        name='Joint Labels',
        showlegend=False
    ))
    
    # Origin marker
    fig.add_trace(go.Scatter3d(
        x=[0],
        y=[0],
        z=[0],
        mode='markers+text',
        marker=dict(size=12, color='black', symbol='diamond-open'),
        text=['Origin'],
        textposition='bottom center',
        name='Origin Reference'
    ))
    
    # Add analysis annotation
    convergence_color = "green" if converged else "orange"
    performance_rating = "EXCELLENT" if total_iterations <= 10 else "GOOD" if total_iterations <= 25 else "SLOW"
    
    fig.add_annotation(
        x=0.02, y=0.98,
        xref="paper", yref="paper",
        text=f"<b>FABRIK Solver Analysis:</b><br>"
             f"• Status: <span style='color:{convergence_color}'><b>{convergence_status}</b></span><br>"
             f"• Final Error: {final_error:.3f}mm<br>"
             f"• Convergence: {performance_rating} ({total_iterations} iterations)<br>"
             f"• Solve Time: {solve_time_ms:.2f}ms<br>"
             f"• Chain Length: {len(joint_positions)} joints<br>"
             f"• Base at Origin: {'✓' if np.allclose(base_position, [0,0,0], atol=0.1) else '✗'}",
        showarrow=False,
        align="left",
        bgcolor="rgba(255,255,255,0.9)",
        bordercolor=convergence_color,
        borderwidth=2,
        font=dict(size=12)
    )
    
    # Add workspace visualization (robot reach)
    robot_height = max(joint_z) if joint_z else 1000
    workspace_radius = ROBOT_RADIUS * 3  # Approximate workspace
    
    # Add workspace circle at target height
    theta = np.linspace(0, 2*np.pi, 50)
    workspace_x = workspace_radius * np.cos(theta)
    workspace_y = workspace_radius * np.sin(theta)
    workspace_z = [target_position[2]] * len(theta)
    
    fig.add_trace(go.Scatter3d(
        x=workspace_x,
        y=workspace_y,
        z=workspace_z,
        mode='lines',
        line=dict(color='lightgray', width=2, dash='dot'),
        name='Approx. Workspace',
        showlegend=False
    ))
    
    fig.show()

def calculate_fabrik_solver(target_position: np.ndarray, 
                           initial_joint_positions: list = None,
                           num_robot_segments: int = None,
                           tolerance: float = None,
                           max_iterations: int = None,
                           debug: bool = False):
    """
    Solve FABRIK for target position using C++ implementation
    
    Args:
        target_position: Target position for end-effector as [x, y, z]
        initial_joint_positions: Optional initial joint positions (defaults to straight-up)
        num_robot_segments: Number of segments (defaults to 7)
        tolerance: Convergence tolerance (defaults to 0.01mm)
        max_iterations: Maximum iterations (defaults to 100)
        debug: If True, show detailed visualization
        
    Returns:
        Tuple of (final_joint_positions, achieved_position, converged, final_error, 
                 total_iterations, solve_time_ms) or None if failed
        
    Raises:
        ValueError: If input constraints are violated
    """
    
    try:
        # Prepare arguments - only pass non-None values to use C++ defaults
        args = [target_position]
        
        if initial_joint_positions is not None:
            # Convert to list of Eigen vectors for C++
            joint_vectors = [np.array(pos, dtype=float) for pos in initial_joint_positions]
            args.append(joint_vectors)
        else:
            args.append(None)  # Use default straight-up initialization
            
        if num_robot_segments is not None:
            args.append(num_robot_segments)
        else:
            args.append(7)  # Default value
            
        if tolerance is not None:
            args.append(tolerance)
        else:
            args.append(0.01)  # Default value
            
        if max_iterations is not None:
            args.append(max_iterations)
        else:
            args.append(100)  # Default value
        
        # Call C++ implementation
        result = delta_robot_cpp.calculate_fabrik_solver(*args)
        final_joint_positions, achieved_position, converged, final_error, total_iterations, solve_time_ms = result
        
        # Print basic results
        print(f"\n=== FABRIK Solver Block Results ===")
        print(f"Target Position: ({target_position[0]:.1f}, {target_position[1]:.1f}, {target_position[2]:.1f})")
        print(f"Achieved Position: ({achieved_position[0]:.1f}, {achieved_position[1]:.1f}, {achieved_position[2]:.1f})")
        print(f"Converged: {'✓ Yes' if converged else '✗ No'}")
        print(f"Final Error: {final_error:.3f}mm")
        print(f"Total Iterations: {total_iterations}")
        print(f"Solve Time: {solve_time_ms:.3f}ms")
        print(f"Joints in Solution: {len(final_joint_positions)}")
        
        # Base position verification
        base_position = np.array(final_joint_positions[0])
        base_at_origin = np.allclose(base_position, [0, 0, 0], atol=0.1)
        print(f"Base at Origin: {'✓ Yes' if base_at_origin else '✗ No'}")
        
        # Performance analysis
        if converged:
            if total_iterations <= 10:
                print(f"Performance: ⚡ EXCELLENT (fast convergence)")
            elif total_iterations <= 25:
                print(f"Performance: ✓ GOOD (reasonable convergence)")
            else:
                print(f"Performance: ⚠️ SLOW (many iterations needed)")
        else:
            print(f"Performance: ⚠️ Did not converge within {max_iterations or 100} iterations")
        
        # Optionally visualize
        if debug and VISUALIZATION_AVAILABLE:
            visualize_fabrik_solver_result(final_joint_positions, target_position, achieved_position,
                                         converged, final_error, total_iterations, solve_time_ms)
        elif debug:
            print("Visualization not available, showing text results only")
        
        return final_joint_positions, achieved_position, converged, final_error, total_iterations, solve_time_ms
        
    except Exception as e:
        print(f"FABRIK solver calculation failed: {str(e)}")
        return None