"""
FABRIK Initialization Block Python Wrapper
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

def visualize_fabrik_initialization_result(joint_positions: list, validation_successful: bool, 
                                         calculation_time_ms: float, num_robot_segments: int) -> None:
    """Visualize FABRIK Initialization results from C++"""
    
    if not VISUALIZATION_AVAILABLE:
        print("Visualization not available")
        return
    
    # Create base figure
    fig = create_base_3d_figure(
        f"FABRIK Initialization Block Results<br>"
        f"Segments: {num_robot_segments}, Joints: {len(joint_positions)}<br>"
        f"Valid: {validation_successful}, Time: {calculation_time_ms:.3f}ms"
    )
    
    # Convert positions to numpy arrays for easier handling
    positions = [np.array(pos) for pos in joint_positions]
    
    # Extract coordinates
    x_coords = [pos[0] for pos in positions]
    y_coords = [pos[1] for pos in positions]
    z_coords = [pos[2] for pos in positions]
    
    # Joint chain as connected line
    fig.add_trace(go.Scatter3d(
        x=x_coords,
        y=y_coords,
        z=z_coords,
        mode='lines+markers',
        line=dict(color=ROBOT_COLORS['joint_chain'], width=4),
        marker=dict(size=8, color=ROBOT_COLORS['joint_points']),
        name='Joint Chain'
    ))
    
    # Label joints
    labels = ['J0_base'] + [f'J{i}' for i in range(1, len(positions)-1)] + ['JN+1_end']
    
    fig.add_trace(go.Scatter3d(
        x=x_coords,
        y=y_coords,
        z=z_coords,
        mode='text',
        text=labels,
        textposition='top center',
        name='Joint Labels',
        showlegend=False
    ))
    
    # Highlight base joint
    fig.add_trace(go.Scatter3d(
        x=[positions[0][0]],
        y=[positions[0][1]],
        z=[positions[0][2]],
        mode='markers',
        marker=dict(size=12, color='red', symbol='diamond'),
        name='Base Joint'
    ))
    
    # Highlight end-effector
    fig.add_trace(go.Scatter3d(
        x=[positions[-1][0]],
        y=[positions[-1][1]],
        z=[positions[-1][2]],
        mode='markers',
        marker=dict(size=12, color=ROBOT_COLORS['end_effector'], symbol='diamond'),
        name='End-Effector'
    ))
    
    # Add segment spacing visualization
    for i in range(len(positions)-1):
        distance = np.linalg.norm(positions[i+1] - positions[i])
        # Add text annotation for segment lengths
        mid_point = (positions[i] + positions[i+1]) / 2
        fig.add_trace(go.Scatter3d(
            x=[mid_point[0]],
            y=[mid_point[1]],
            z=[mid_point[2]],
            mode='text',
            text=[f'{distance:.1f}mm'],
            textposition='middle left',
            name='Segment Lengths',
            showlegend=False if i > 0 else True
        ))
    
    # Add validation status annotation
    status_color = 'green' if validation_successful else 'red'
    status_text = '✓ Valid Configuration' if validation_successful else '✗ Invalid Configuration'
    
    fig.add_annotation(
        x=0.02, y=0.95,
        xref="paper", yref="paper",
        text=f"<b>{status_text}</b><br>"
             f"• Segments: {num_robot_segments}<br>"
             f"• Total Joints: {len(joint_positions)}<br>"
             f"• Base at Origin: {np.allclose(positions[0], [0,0,0])}<br>"
             f"• Total Height: {positions[-1][2]:.1f}mm",
        showarrow=False,
        align="left",
        bgcolor=f"rgba({'0,255,0' if validation_successful else '255,0,0'},0.1)",
        bordercolor=status_color,
        borderwidth=2
    )
    
    fig.show()


def calculate_fabrik_initialization(num_robot_segments: int, initial_joint_positions=None, debug: bool = False):
    """
    Create initial joint positions for FABRIK solving using C++ implementation
    
    Args:
        num_robot_segments: Number of robot segments (typically 3-7)
        initial_joint_positions: Optional custom positions (if None, creates straight-up)
        debug: If True, show visualization
        
    Returns:
        Tuple of (joint_positions, validation_successful, calculation_time_ms)
        
    Raises:
        ValueError: If input constraints are violated
    """
    
    try:
        # Call C++ implementation
        joint_positions, validation_successful, calculation_time_ms = delta_robot_cpp.calculate_fabrik_initialization(
            num_robot_segments, initial_joint_positions
        )
        
        # Print results
        print(f"\n=== FABRIK Initialization Block Results ===")
        print(f"Robot Segments: {num_robot_segments}")
        print(f"Total Joints: {len(joint_positions)}")
        print(f"Validation Successful: {validation_successful}")
        print(f"Calculation Time: {calculation_time_ms:.3f}ms")
        
        if validation_successful:
            print(f"Base Joint: ({joint_positions[0][0]:.3f}, {joint_positions[0][1]:.3f}, {joint_positions[0][2]:.3f})")
            print(f"End-Effector: ({joint_positions[-1][0]:.3f}, {joint_positions[-1][1]:.3f}, {joint_positions[-1][2]:.3f})")
            print(f"Total Height: {joint_positions[-1][2]:.1f}mm")
            
            # Calculate segment lengths
            segments_lengths = []
            for i in range(len(joint_positions)-1):
                length = np.linalg.norm(np.array(joint_positions[i+1]) - np.array(joint_positions[i]))
                segments_lengths.append(length)
            print(f"Segment Lengths: {[f'{l:.1f}mm' for l in segments_lengths]}")
        
        # Optionally visualize
        if debug and VISUALIZATION_AVAILABLE:
            visualize_fabrik_initialization_result(joint_positions, validation_successful, 
                                                 calculation_time_ms, num_robot_segments)
        elif debug:
            print("Visualization not available, showing text results only")
        
        return joint_positions, validation_successful, calculation_time_ms
        
    except Exception as e:
        raise ValueError(f"FABRIK initialization failed: {str(e)}")


def calculate_fabrik_initialization_straight_up(num_robot_segments: int = 7, debug: bool = False):
    """
    Create straight-up initial joint positions for FABRIK solving
    
    Args:
        num_robot_segments: Number of robot segments (default 7)
        debug: If True, show visualization
        
    Returns:
        Tuple of (joint_positions, validation_successful, calculation_time_ms)
    """
    return calculate_fabrik_initialization(num_robot_segments, None, debug)