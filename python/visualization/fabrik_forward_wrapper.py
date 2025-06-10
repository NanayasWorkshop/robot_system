"""
FABRIK Forward Block Python Wrapper
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

def visualize_fabrik_forward_result(original_joints: list, updated_joints: list, target_position: np.ndarray,
                                  original_lengths: list, new_lengths: list, distance_to_target: float, 
                                  calculation_time_ms: float) -> None:
    """Visualize FABRIK Forward results from C++"""
    
    if not VISUALIZATION_AVAILABLE:
        print("Visualization not available")
        return
    
    # Create base figure
    fig = create_base_3d_figure(
        f"FABRIK Forward Block Results<br>"
        f"Target: ({target_position[0]:.1f}, {target_position[1]:.1f}, {target_position[2]:.1f})<br>"
        f"Target Drift: {distance_to_target:.1f}mm, Time: {calculation_time_ms:.3f}ms"
    )
    
    # Convert to numpy arrays
    original_positions = [np.array(pos) for pos in original_joints]
    updated_positions = [np.array(pos) for pos in updated_joints]
    
    # Extract coordinates for original chain (after backward pass)
    orig_x = [pos[0] for pos in original_positions]
    orig_y = [pos[1] for pos in original_positions]
    orig_z = [pos[2] for pos in original_positions]
    
    # Extract coordinates for updated chain (after forward pass)
    updated_x = [pos[0] for pos in updated_positions]
    updated_y = [pos[1] for pos in updated_positions]
    updated_z = [pos[2] for pos in updated_positions]
    
    # Original joint chain (after backward, before forward)
    fig.add_trace(go.Scatter3d(
        x=orig_x,
        y=orig_y,
        z=orig_z,
        mode='lines+markers',
        line=dict(color='lightcoral', width=3, dash='dash'),
        marker=dict(size=6, color='lightcoral'),
        name='After Backward (Base Drifted)'
    ))
    
    # Updated joint chain (after forward pass)
    fig.add_trace(go.Scatter3d(
        x=updated_x,
        y=updated_y,
        z=updated_z,
        mode='lines+markers',
        line=dict(color=ROBOT_COLORS['joint_chain'], width=4),
        marker=dict(size=8, color=ROBOT_COLORS['joint_points']),
        name='After Forward (Base Fixed)'
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
    
    # Origin marker
    fig.add_trace(go.Scatter3d(
        x=[0],
        y=[0],
        z=[0],
        mode='markers+text',
        marker=dict(size=12, color='green', symbol='diamond'),
        text=['Origin'],
        name='Origin (Fixed Base)'
    ))
    
    # Base movement arrow (drifted → fixed)
    original_base = original_positions[0]
    updated_base = updated_positions[0]  # Should be (0,0,0)
    
    if np.linalg.norm(original_base) > 0.1:  # Only show if base actually moved
        fig.add_trace(go.Scatter3d(
            x=[original_base[0], updated_base[0]],
            y=[original_base[1], updated_base[1]],
            z=[original_base[2], updated_base[2]],
            mode='lines+markers',
            line=dict(color='green', width=6),
            marker=dict(size=[8, 12], color=['orange', 'green'], symbol=['circle', 'diamond']),
            name='Base Correction'
        ))
    
    # End-effector drift visualization
    original_end = original_positions[-1]
    updated_end = updated_positions[-1]
    
    fig.add_trace(go.Scatter3d(
        x=[original_end[0], updated_end[0]],
        y=[original_end[1], updated_end[1]],
        z=[original_end[2], updated_end[2]],
        mode='lines+markers',
        line=dict(color='purple', width=4, dash='dot'),
        marker=dict(size=[8, 8], color=['red', 'purple']),
        name='End-Effector Drift'
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
    
    # Segment length comparison
    length_changes = []
    for i, (old_len, new_len) in enumerate(zip(original_lengths, new_lengths)):
        change = abs(new_len - old_len)
        length_changes.append(change)
    
    max_change = max(length_changes) if length_changes else 0
    avg_change = np.mean(length_changes) if length_changes else 0
    
    # Add analysis annotation (moved to avoid legend overlap)
    fig.add_annotation(
        x=0.02, y=0.75,  # Moved down from 0.95 to 0.75
        xref="paper", yref="paper",
        text=f"<b>Forward Pass Analysis:</b><br>"
             f"• Base Fixed: {'✓' if np.allclose(updated_base, [0,0,0], atol=0.1) else '✗'}<br>"
             f"• Target Drift: {distance_to_target:.1f}mm<br>"
             f"• Segments Recalculated: {len(new_lengths)}<br>"
             f"• Max Length Change: {max_change:.1f}mm<br>"
             f"• Avg Length Change: {avg_change:.1f}mm",
        showarrow=False,
        align="left",
        bgcolor="rgba(255,255,255,0.8)",
        bordercolor="blue",
        borderwidth=1
    )
    
    fig.show()


def calculate_fabrik_forward(joint_positions: list, target_position: np.ndarray, debug: bool = False):
    """
    Perform single forward pass with dynamic segment recalculation using C++ implementation
    
    Args:
        joint_positions: List of 3D joint positions (after backward pass)
        target_position: Target position for drift calculation
        debug: If True, show visualization
        
    Returns:
        Tuple of (updated_joint_positions, recalculated_segment_lengths, distance_to_target, calculation_time_ms)
        
    Raises:
        ValueError: If input constraints are violated
    """
    
    try:
        # Calculate original segment lengths for comparison
        original_lengths = []
        for i in range(len(joint_positions) - 1):
            length = np.linalg.norm(np.array(joint_positions[i+1]) - np.array(joint_positions[i]))
            original_lengths.append(length)
        
        # Call C++ implementation
        updated_joint_positions, recalculated_lengths, distance_to_target, calculation_time_ms = delta_robot_cpp.calculate_fabrik_forward(
            joint_positions, target_position
        )
        
        # Print results
        print(f"\n=== FABRIK Forward Block Results ===")
        print(f"Target Position: ({target_position[0]:.1f}, {target_position[1]:.1f}, {target_position[2]:.1f})")
        print(f"Joints Updated: {len(updated_joint_positions)}")
        print(f"Target Drift Distance: {distance_to_target:.1f}mm")
        print(f"Calculation Time: {calculation_time_ms:.3f}ms")
        
        # Base position analysis
        updated_base = np.array(updated_joint_positions[0])
        base_at_origin = np.allclose(updated_base, [0, 0, 0], atol=0.1)
        print(f"Base Fixed at Origin: {'✓ Yes' if base_at_origin else '✗ No'}")
        print(f"Updated Base Position: ({updated_base[0]:.3f}, {updated_base[1]:.3f}, {updated_base[2]:.3f})")
        
        # Segment length analysis
        print(f"\nSegment Length Recalculation:")
        print(f"Original Lengths: {[f'{l:.1f}mm' for l in original_lengths]}")
        print(f"New Lengths:      {[f'{l:.1f}mm' for l in recalculated_lengths]}")
        
        # Calculate changes
        length_changes = [abs(new - old) for old, new in zip(original_lengths, recalculated_lengths)]
        max_change = max(length_changes) if length_changes else 0
        avg_change = np.mean(length_changes) if length_changes else 0
        print(f"Max Change: {max_change:.1f}mm, Avg Change: {avg_change:.1f}mm")
        
        # Optionally visualize
        if debug and VISUALIZATION_AVAILABLE:
            visualize_fabrik_forward_result(joint_positions, updated_joint_positions, target_position,
                                          original_lengths, recalculated_lengths, distance_to_target, calculation_time_ms)
        elif debug:
            print("Visualization not available, showing text results only")
        
        return updated_joint_positions, recalculated_lengths, distance_to_target, calculation_time_ms
        
    except Exception as e:
        raise ValueError(f"FABRIK forward calculation failed: {str(e)}")