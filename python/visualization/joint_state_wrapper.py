"""
Joint State Block Python Wrapper
Pure visualization wrapper - calls C++ and plots results
"""

import numpy as np
from base_plotter import create_base_3d_figure, add_robot_base_circle, add_vector_arrow, ROBOT_COLORS
import plotly.graph_objects as go

import delta_robot_cpp

def visualize_joint_state_result(input_vector: np.ndarray, prismatic: float, roll: float, pitch: float, 
                                calculation_time_ms: float, fermat_data: tuple) -> None:
    """Visualize Joint State calculation results from C++"""
    
    # Extract fermat data (z_A, z_B, z_C, fermat_point, fermat_calc_time)
    z_A, z_B, z_C, fermat_point, fermat_calc_time = fermat_data
    
    # Create base figure with coordinate system
    fig = create_base_3d_figure(f"Joint State Block Results - Input: ({input_vector[0]:.2f}, {input_vector[1]:.2f}, {input_vector[2]:.2f})<br>Calculation Time: {calculation_time_ms:.3f}ms")
    
    # Add robot base circle for reference
    ROBOT_RADIUS = delta_robot_cpp.ROBOT_RADIUS
    add_robot_base_circle(fig, ROBOT_RADIUS)
    
    # Fermat point
    fig.add_trace(go.Scatter3d(
        x=[fermat_point[0]],
        y=[fermat_point[1]],
        z=[fermat_point[2]],
        mode='markers+text',
        marker=dict(size=12, color=ROBOT_COLORS['fermat_point']),
        text=['Fermat'],
        name='Fermat Point'
    ))
    
    # Input direction vector
    normalized_input = input_vector / np.linalg.norm(input_vector)
    vector_end = normalized_input * 30
    add_vector_arrow(fig, [0, 0, 0], vector_end, color=ROBOT_COLORS['input_vector'], name='Input Direction')
    
    # Add text annotations for joint values
    fig.add_trace(go.Scatter3d(
        x=[35], y=[35], z=[20],
        mode='text',
        text=[f'Prismatic: {prismatic:.3f}<br>Roll: {roll:.3f} rad<br>Pitch: {pitch:.3f} rad'],
        textfont=dict(size=14),
        name='Joint Values',
        showlegend=False
    ))
    
    fig.show()
    
    # Print results
    print(f"\n=== Joint State Block Results ===")
    print(f"Input Vector: ({input_vector[0]:.3f}, {input_vector[1]:.3f}, {input_vector[2]:.3f})")
    print(f"Prismatic Joint: {prismatic:.3f}")
    print(f"Roll Joint: {roll:.3f} rad ({np.degrees(roll):.1f}°)")
    print(f"Pitch Joint: {pitch:.3f} rad ({np.degrees(pitch):.1f}°)")
    print(f"Calculation Time: {calculation_time_ms:.3f}ms")


def calculate_joint_state(x: float, y: float, z: float, debug: bool = False):
    """
    Calculate joint states from direction vector using C++ implementation
    
    Args:
        x, y, z: Input direction vector components (z must be positive)
        debug: If True, show visualization
        
    Returns:
        Tuple of (prismatic_joint, roll_joint, pitch_joint, calculation_time_ms, fermat_data)
        
    Raises:
        ValueError: If input constraints are violated
    """
    
    try:
        # Call C++ implementation - validation happens in C++
        prismatic, roll, pitch, calculation_time_ms, fermat_data = delta_robot_cpp.calculate_joint_state(x, y, z)
        
        # Optionally visualize
        if debug:
            input_vector = np.array([x, y, z])
            visualize_joint_state_result(input_vector, prismatic, roll, pitch, 
                                       calculation_time_ms, fermat_data)
        
        return prismatic, roll, pitch, calculation_time_ms, fermat_data
        
    except Exception as e:
        raise ValueError(f"Joint state calculation failed: {str(e)}")