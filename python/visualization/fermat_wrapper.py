"""
Fermat Block Python Wrapper
Pure visualization wrapper - calls C++ and plots results
"""

import numpy as np
from base_plotter import create_base_3d_figure, add_robot_base_circle, add_vector_arrow, ROBOT_COLORS
import plotly.graph_objects as go

import delta_robot_cpp

def visualize_fermat_result(input_vector: np.ndarray, z_A: float, z_B: float, z_C: float, fermat_point: np.ndarray, calculation_time_ms: float) -> None:
    """Visualize Fermat calculation results from C++"""
    
    # Get constants from C++
    ROBOT_RADIUS = delta_robot_cpp.ROBOT_RADIUS
    BASE_A_ANGLE = delta_robot_cpp.BASE_A_ANGLE
    BASE_B_ANGLE = delta_robot_cpp.BASE_B_ANGLE
    BASE_C_ANGLE = delta_robot_cpp.BASE_C_ANGLE
    
    # Calculate base positions using C++ constants
    base_A = np.array([ROBOT_RADIUS * np.cos(BASE_A_ANGLE), ROBOT_RADIUS * np.sin(BASE_A_ANGLE), 0.0])
    base_B = np.array([ROBOT_RADIUS * np.cos(BASE_B_ANGLE), ROBOT_RADIUS * np.sin(BASE_B_ANGLE), 0.0])
    base_C = np.array([ROBOT_RADIUS * np.cos(BASE_C_ANGLE), ROBOT_RADIUS * np.sin(BASE_C_ANGLE), 0.0])
    
    # Points with calculated Z positions
    point_A = np.array([base_A[0], base_A[1], z_A])
    point_B = np.array([base_B[0], base_B[1], z_B])
    point_C = np.array([base_C[0], base_C[1], z_C])
    
    # Create base figure with coordinate system
    fig = create_base_3d_figure(f"Fermat Block Results - Input: ({input_vector[0]:.2f}, {input_vector[1]:.2f}, {input_vector[2]:.2f})<br>Calculation Time: {calculation_time_ms:.3f}ms")
    
    # Add robot base circle for reference
    add_robot_base_circle(fig, ROBOT_RADIUS)
    
    # Base positions
    fig.add_trace(go.Scatter3d(
        x=[base_A[0], base_B[0], base_C[0]],
        y=[base_A[1], base_B[1], base_C[1]], 
        z=[base_A[2], base_B[2], base_C[2]],
        mode='markers+text',
        marker=dict(size=8, color=[ROBOT_COLORS['actuator_A'], ROBOT_COLORS['actuator_B'], ROBOT_COLORS['actuator_C']]),
        text=['A_base', 'B_base', 'C_base'],
        name='Base Positions'
    ))
    
    # Calculated points with Z positions
    fig.add_trace(go.Scatter3d(
        x=[point_A[0], point_B[0], point_C[0]],
        y=[point_A[1], point_B[1], point_C[1]],
        z=[point_A[2], point_B[2], point_C[2]],
        mode='markers+text',
        marker=dict(size=10, color=[ROBOT_COLORS['actuator_A'], ROBOT_COLORS['actuator_B'], ROBOT_COLORS['actuator_C']], symbol='diamond'),
        text=['A', 'B', 'C'],
        name='Calculated Points'
    ))
    
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
    
    # Lines from base to calculated points
    colors = [ROBOT_COLORS['actuator_A'], ROBOT_COLORS['actuator_B'], ROBOT_COLORS['actuator_C']]
    for i, (base, point, color) in enumerate(zip([base_A, base_B, base_C], [point_A, point_B, point_C], colors)):
        fig.add_trace(go.Scatter3d(
            x=[base[0], point[0]],
            y=[base[1], point[1]],
            z=[base[2], point[2]],
            mode='lines',
            line=dict(color=color, width=3, dash='dash'),
            showlegend=False
        ))
    
    fig.show()
    
    # Print results
    print(f"\n=== Fermat Block Results ===")
    print(f"Input Vector: ({input_vector[0]:.3f}, {input_vector[1]:.3f}, {input_vector[2]:.3f})")
    print(f"Fermat Point: ({fermat_point[0]:.3f}, {fermat_point[1]:.3f}, {fermat_point[2]:.3f})")
    print(f"Z Positions: A={z_A:.3f}, B={z_B:.3f}, C={z_C:.3f}")
    print(f"Calculation Time: {calculation_time_ms:.3f}ms")


def calculate_fermat(x: float, y: float, z: float, debug: bool = False):
    """
    Calculate Fermat point and Z positions using C++ implementation
    
    Args:
        x, y, z: Input direction vector components (z must be positive)
        debug: If True, show visualization
        
            Returns:
        Tuple of (z_A, z_B, z_C, fermat_point, calculation_time_ms)
        
    Raises:
        ValueError: If input constraints are violated
    """
    
    try:
        # Call C++ implementation - validation happens in C++
        z_A, z_B, z_C, fermat_point, calculation_time_ms = delta_robot_cpp.calculate_fermat(x, y, z)
        
        # Optionally visualize
        if debug:
            input_vector = np.array([x, y, z])
            visualize_fermat_result(input_vector, z_A, z_B, z_C, fermat_point, calculation_time_ms)
        
        return z_A, z_B, z_C, fermat_point, calculation_time_ms
        
    except Exception as e:
        raise ValueError(f"Fermat calculation failed: {str(e)}")