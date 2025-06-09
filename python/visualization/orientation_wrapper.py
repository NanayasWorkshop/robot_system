"""
Orientation Block Python Wrapper
Pure visualization wrapper - calls C++ and plots results
"""

import numpy as np
from base_plotter import create_base_3d_figure, add_robot_base_circle, add_vector_arrow, ROBOT_COLORS
import plotly.graph_objects as go

import delta_robot_cpp

def visualize_orientation_result(kinematics_result: tuple, end_effector: np.ndarray, point_G: np.ndarray,
                                final_frame: tuple, transformation_matrix: np.ndarray, 
                                calculation_time_ms: float) -> None:
    """Visualize Orientation calculation results from C++"""
    
    # Get constants from C++
    ROBOT_RADIUS = delta_robot_cpp.ROBOT_RADIUS
    BASE_A_ANGLE = delta_robot_cpp.BASE_A_ANGLE
    BASE_B_ANGLE = delta_robot_cpp.BASE_B_ANGLE
    BASE_C_ANGLE = delta_robot_cpp.BASE_C_ANGLE
    
    # Calculate base positions using C++ constants
    base_A = np.array([ROBOT_RADIUS * np.cos(BASE_A_ANGLE), ROBOT_RADIUS * np.sin(BASE_A_ANGLE), 0.0])
    base_B = np.array([ROBOT_RADIUS * np.cos(BASE_B_ANGLE), ROBOT_RADIUS * np.sin(BASE_B_ANGLE), 0.0])
    base_C = np.array([ROBOT_RADIUS * np.cos(BASE_C_ANGLE), ROBOT_RADIUS * np.sin(BASE_C_ANGLE), 0.0])
    
    # Extract kinematics data
    point_H, _, HG_length, kinematics_end_effector, kinematics_calc_time, fermat_data = kinematics_result
    z_A, z_B, z_C, fermat_point, fermat_calc_time = fermat_data
    
    # Points with calculated Z positions from fermat
    point_A = np.array([base_A[0], base_A[1], z_A])
    point_B = np.array([base_B[0], base_B[1], z_B])
    point_C = np.array([base_C[0], base_C[1], z_C])
    
    # Extract final frame components (origin, u_axis, v_axis, w_axis)
    frame_origin, frame_u, frame_v, frame_w = final_frame
    
    # Create base figure with coordinate system
    fig = create_base_3d_figure(f"Orientation Block Results<br>Calculation Time: {calculation_time_ms:.3f}ms")
    
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
    
    # Fermat calculated points
    fig.add_trace(go.Scatter3d(
        x=[point_A[0], point_B[0], point_C[0]],
        y=[point_A[1], point_B[1], point_C[1]],
        z=[point_A[2], point_B[2], point_C[2]],
        mode='markers+text',
        marker=dict(size=10, color=[ROBOT_COLORS['actuator_A'], ROBOT_COLORS['actuator_B'], ROBOT_COLORS['actuator_C']], symbol='diamond'),
        text=['A', 'B', 'C'],
        name='Fermat Points'
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
    
    # Joint H position
    fig.add_trace(go.Scatter3d(
        x=[point_H[0]],
        y=[point_H[1]],
        z=[point_H[2]],
        mode='markers+text',
        marker=dict(size=12, color='darkblue'),
        text=['H'],
        name='Point H'
    ))
    
    # Point G
    fig.add_trace(go.Scatter3d(
        x=[point_G[0]],
        y=[point_G[1]],
        z=[point_G[2]],
        mode='markers+text',
        marker=dict(size=12, color='darkgreen'),
        text=['G'],
        name='Point G'
    ))
    
    # End-effector
    fig.add_trace(go.Scatter3d(
        x=[end_effector[0]],
        y=[end_effector[1]],
        z=[end_effector[2]],
        mode='markers+text',
        marker=dict(size=12, color=ROBOT_COLORS['end_effector']),
        text=['End-Effector'],
        name='End-Effector'
    ))
    
    # Final coordinate frame U''V''W'' at end-effector
    scale = 15.0
    # U'' axis (red)
    u_end = np.array(frame_origin) + np.array(frame_u) * scale
    add_vector_arrow(fig, frame_origin, u_end, color='red', name="U'' axis", width=6)
    
    # V'' axis (green)
    v_end = np.array(frame_origin) + np.array(frame_v) * scale
    add_vector_arrow(fig, frame_origin, v_end, color='green', name="V'' axis", width=6)
    
    # W'' axis (blue)
    w_end = np.array(frame_origin) + np.array(frame_w) * scale
    add_vector_arrow(fig, frame_origin, w_end, color='blue', name="W'' axis", width=6)
    
    # H → G line
    fig.add_trace(go.Scatter3d(
        x=[point_H[0], point_G[0]],
        y=[point_H[1], point_G[1]],
        z=[point_H[2], point_G[2]],
        mode='lines',
        line=dict(color='darkblue', width=4),
        name='H→G'
    ))
    
    # Lines from base to fermat calculated points
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
    print(f"\n=== Orientation Block Results ===")
    print(f"End-Effector: ({end_effector[0]:.3f}, {end_effector[1]:.3f}, {end_effector[2]:.3f})")
    print(f"Point G: ({point_G[0]:.3f}, {point_G[1]:.3f}, {point_G[2]:.3f})")
    print(f"Final Frame Origin: ({frame_origin[0]:.3f}, {frame_origin[1]:.3f}, {frame_origin[2]:.3f})")
    print(f"Final Frame U'': ({frame_u[0]:.3f}, {frame_u[1]:.3f}, {frame_u[2]:.3f})")
    print(f"Final Frame V'': ({frame_v[0]:.3f}, {frame_v[1]:.3f}, {frame_v[2]:.3f})")
    print(f"Final Frame W'': ({frame_w[0]:.3f}, {frame_w[1]:.3f}, {frame_w[2]:.3f})")
    print(f"Transformation Matrix:\n{transformation_matrix}")
    print(f"Calculation Time: {calculation_time_ms:.3f}ms")


def calculate_orientation(x: float, y: float, z: float, debug: bool = False):
    """
    Calculate orientation from direction vector using C++ implementation
    
    Args:
        x, y, z: Input direction vector components (z must be positive)
        debug: If True, show visualization
        
    Returns:
        Tuple of (end_effector, point_G, final_frame, transformation_matrix, calculation_time_ms, kinematics_data)
        
    Raises:
        ValueError: If input constraints are violated
    """
    
    try:
        # Call C++ implementation - validation happens in C++
        end_effector, point_G, final_frame, transformation_matrix, calculation_time_ms, kinematics_data = delta_robot_cpp.calculate_orientation(x, y, z)
        
        # Optionally visualize
        if debug:
            # We need kinematics_data for visualization, which is returned from C++
            visualize_orientation_result(kinematics_data, end_effector, point_G, final_frame, 
                                       transformation_matrix, calculation_time_ms)
        
        return end_effector, point_G, final_frame, transformation_matrix, calculation_time_ms, kinematics_data
        
    except Exception as e:
        raise ValueError(f"Orientation calculation failed: {str(e)}")