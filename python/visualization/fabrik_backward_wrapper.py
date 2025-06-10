"""
FABRIK Backward Block Python Wrapper
Pure visualization wrapper - calls C++ and plots results
"""

import numpy as np
import delta_robot_cpp

try:
    from base_plotter import create_base_3d_figure, add_vector_arrow, ROBOT_COLORS
    import plotly.graph_objects as go
    VISUALIZATION_AVAILABLE = True
except ImportError as e:
    print(f"Visualization not available: {e}")
    VISUALIZATION_AVAILABLE = False

def visualize_fabrik_backward_result(original_joints: list, updated_joints: list, target_position: np.ndarray,
                                   distance_to_base: float, calculation_time_ms: float) -> None:
    """Visualize FABRIK Backward results from C++"""
    
    if not VISUALIZATION_AVAILABLE:
        print("Visualization not available")
        return
    
    # Create base figure
    fig = create_base_3d_figure(
        f"FABRIK Backward Block Results<br>"
        f"Target: ({target_position[0]:.1f}, {target_position[1]:.1f}, {target_position[2]:.1f})<br>"
        f"Base Drift: {distance_to_base:.1f}mm, Time: {calculation_time_ms:.3f}ms"
    )
    
    # Convert to numpy arrays
    original_positions = [np.array(pos) for pos in original_joints]
    updated_positions = [np.array(pos) for pos in updated_joints]
    
    # Extract coordinates for original chain
    orig_x = [pos[0] for pos in original_positions]
    orig_y = [pos[1] for pos in original_positions]
    orig_z = [pos[2] for pos in original_positions]
    
    # Extract coordinates for updated chain
    updated_x = [pos[0] for pos in updated_positions]
    updated_y = [pos[1] for pos in updated_positions]
    updated_z = [pos[2] for pos in updated_positions]
    
    # Original joint chain (before backward pass)
    fig.add_trace(go.Scatter3d(
        x=orig_x,
        y=orig_y,
        z=orig_z,
        mode='lines+markers',
        line=dict(color='lightgray', width=3, dash='dash'),
        marker=dict(size=6, color='lightgray'),
        name='Original Chain'
    ))
    
    # Updated joint chain (after backward pass)
    fig.add_trace(go.Scatter3d(
        x=updated_x,
        y=updated_y,
        z=updated_z,
        mode='lines+markers',
        line=dict(color=ROBOT_COLORS['joint_chain'], width=4),
        marker=dict(size=8, color=ROBOT_COLORS['joint_points']),
        name='Updated Chain'
    ))
    
    # Target position
    fig.add_trace(go.Scatter3d(
        x=[target_position[0]],
        y=[target_position[1]],
        z=[target_position[2]],
        mode='markers+text',
        marker=dict(size=12, color='red', symbol='x'),
        text=['Target'],
        name='Target Position'
    ))
    
    # Original base (should be at origin)
    fig.add_trace(go.Scatter3d(
        x=[original_positions[0][0]],
        y=[original_positions[0][1]],
        z=[original_positions[0][2]],
        mode='markers+text',
        marker=dict(size=12, color='green', symbol='diamond'),
        text=['Original Base'],
        name='Original Base'
    ))
    
    # Updated base (drifted away from origin)
    fig.add_trace(go.Scatter3d(
        x=[updated_positions[0][0]],
        y=[updated_positions[0][1]],
        z=[updated_positions[0][2]],
        mode='markers+text',
        marker=dict(size=12, color='orange', symbol='diamond'),
        text=['Drifted Base'],
        name='Drifted Base'
    ))
    
    # End-effector movement arrow
    original_end = original_positions[-1]
    updated_end = updated_positions[-1]
    
    fig.add_trace(go.Scatter3d(
        x=[original_end[0], updated_end[0]],
        y=[original_end[1], updated_end[1]],
        z=[original_end[2], updated_end[2]],
        mode='lines+markers',
        line=dict(color='purple', width=6),
        marker=dict(size=[8, 12], color=['purple', 'red'], symbol=['circle', 'x']),
        name='End-Effector Movement'
    ))
    
    # Add joint labels
    labels = [f'J{i}' for i in range(len(updated_positions))]
    fig.add_trace(go.Scatter3d(
        x=updated_x,
        y=updated_y,
        z=updated_z,
        mode='text',
        text=labels,
        textposition='top center',
        name='Joint Labels',
        showlegend=False
    ))
    
    # Add analysis annotation
    end_to_target_distance = np.linalg.norm(updated_end - target_position)
    
    fig.add_annotation(
        x=0.02, y=0.95,
        xref="paper", yref="paper",
        text=f"<b>Backward Pass Analysis:</b><br>"
             f"• End-Effector to Target: {end_to_target_distance:.1f}mm<br>"
             f"• Base Drift Distance: {distance_to_base:.1f}mm<br>"
             f"• Joints Updated: {len(updated_positions)}<br>"
             f"• Target Reached: {'✓' if end_to_target_distance < 1.0 else '✗'}",
        showarrow=False,
        align="left",
        bgcolor="rgba(255,255,255,0.8)",
        bordercolor="black",
        borderwidth=1
    )
    
    fig.show()


def calculate_fabrik_backward(joint_positions: list, target_position: np.ndarray, 
                            segment_lengths: list, debug: bool = False):
    """
    Perform single backward pass using C++ implementation
    
    Args:
        joint_positions: List of 3D joint positions
        target_position: Target position for end-effector
        segment_lengths: List of segment lengths
        debug: If True, show visualization
        
    Returns:
        Tuple of (updated_joint_positions, distance_to_base, calculation_time_ms)
        
    Raises:
        ValueError: If input constraints are violated
    """
    
    try:
        # Call C++ implementation
        updated_joint_positions, distance_to_base, calculation_time_ms = delta_robot_cpp.calculate_fabrik_backward(
            joint_positions, target_position, segment_lengths
        )
        
        # Print results
        print(f"\n=== FABRIK Backward Block Results ===")
        print(f"Target Position: ({target_position[0]:.1f}, {target_position[1]:.1f}, {target_position[2]:.1f})")
        print(f"Joints Updated: {len(updated_joint_positions)}")
        print(f"Base Drift Distance: {distance_to_base:.1f}mm")
        print(f"Calculation Time: {calculation_time_ms:.3f}ms")
        
        # Check if end-effector reached target
        end_effector = np.array(updated_joint_positions[-1])
        distance_to_target = np.linalg.norm(end_effector - target_position)
        print(f"End-Effector to Target Distance: {distance_to_target:.3f}mm")
        print(f"Target Reached: {'✓ Yes' if distance_to_target < 1.0 else '✗ No'}")
        
        # Optionally visualize
        if debug and VISUALIZATION_AVAILABLE:
            visualize_fabrik_backward_result(joint_positions, updated_joint_positions, target_position,
                                           distance_to_base, calculation_time_ms)
        elif debug:
            print("Visualization not available, showing text results only")
        
        return updated_joint_positions, distance_to_base, calculation_time_ms
        
    except Exception as e:
        raise ValueError(f"FABRIK backward calculation failed: {str(e)}")