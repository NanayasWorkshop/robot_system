"""
Cone Constraint Block Python Wrapper
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

def visualize_cone_constraint_result(desired_direction: np.ndarray, cone_apex: np.ndarray, 
                                   cone_axis: np.ndarray, cone_angle_rad: float,
                                   projected_direction: np.ndarray, constraint_applied: bool, 
                                   calculation_time_ms: float) -> None:
    """Visualize Cone Constraint calculation results from C++"""
    
    # Create base figure
    angle_deg = np.degrees(cone_angle_rad)
    fig = create_base_3d_figure(
        f"Cone Constraint Block Results<br>"
        f"Cone Angle: {angle_deg:.1f}°, Constraint Applied: {constraint_applied}<br>"
        f"Calculation Time: {calculation_time_ms:.3f}ms"
    )
    
    # Cone apex point
    fig.add_trace(go.Scatter3d(
        x=[cone_apex[0]],
        y=[cone_apex[1]],
        z=[cone_apex[2]],
        mode='markers+text',
        marker=dict(size=12, color='red'),
        text=['Apex'],
        name='Cone Apex'
    ))
    
    # Desired direction vector (from apex)
    desired_end = cone_apex + desired_direction
    add_vector_arrow(fig, cone_apex, desired_end, color='orange', name='Desired Direction', width=6)
    
    # Projected direction vector (from apex)
    projected_end = cone_apex + projected_direction
    if constraint_applied:
        add_vector_arrow(fig, cone_apex, projected_end, color='green', name='Projected Direction', width=6)
    else:
        # If no constraint applied, projected = desired, so just mark it differently
        add_vector_arrow(fig, cone_apex, projected_end, color='lightgreen', name='No Constraint Needed', width=6)
    
    # Cone axis vector (from apex)
    axis_length = np.linalg.norm(cone_axis)
    if axis_length > 0:
        cone_axis_normalized = cone_axis / axis_length
        axis_end = cone_apex + cone_axis_normalized * np.linalg.norm(desired_direction)
        add_vector_arrow(fig, cone_apex, axis_end, color='blue', name='Cone Axis', width=4)
    
    # Visualize cone surface (simplified as circle at end of axis)
    if axis_length > 0:
        cone_axis_normalized = cone_axis / axis_length
        cone_height = np.linalg.norm(desired_direction)
        cone_radius = cone_height * np.tan(cone_angle_rad / 2.0)
        
        # Create cone circle at the end of axis direction
        theta = np.linspace(0, 2*np.pi, 50)
        
        # Create orthogonal vectors to cone axis
        if abs(cone_axis_normalized[2]) < 0.9:
            v1 = np.cross(cone_axis_normalized, [0, 0, 1])
        else:
            v1 = np.cross(cone_axis_normalized, [1, 0, 0])
        v1 = v1 / np.linalg.norm(v1)
        v2 = np.cross(cone_axis_normalized, v1)
        v2 = v2 / np.linalg.norm(v2)
        
        # Circle points
        circle_center = cone_apex + cone_axis_normalized * cone_height
        circle_x = circle_center[0] + cone_radius * (np.cos(theta) * v1[0] + np.sin(theta) * v2[0])
        circle_y = circle_center[1] + cone_radius * (np.cos(theta) * v1[1] + np.sin(theta) * v2[1])
        circle_z = circle_center[2] + cone_radius * (np.cos(theta) * v1[2] + np.sin(theta) * v2[2])
        
        fig.add_trace(go.Scatter3d(
            x=circle_x,
            y=circle_y,
            z=circle_z,
            mode='lines',
            line=dict(color='lightblue', width=2, dash='dot'),
            name='Cone Surface',
            showlegend=True
        ))
    
    # Add text annotations
    fig.add_annotation(
        x=0.02, y=0.85,
        xref="paper", yref="paper",
        text=f"<b>Results:</b><br>"
             f"• Desired: ({desired_direction[0]:.2f}, {desired_direction[1]:.2f}, {desired_direction[2]:.2f})<br>"
             f"• Projected: ({projected_direction[0]:.2f}, {projected_direction[1]:.2f}, {projected_direction[2]:.2f})<br>"
             f"• Cone Angle: {angle_deg:.1f}°<br>"
             f"• Constraint Applied: {constraint_applied}",
        showarrow=False,
        align="left",
        bgcolor="rgba(255,255,255,0.8)",
        bordercolor="black",
        borderwidth=1
    )
    
    fig.show()
    
    # Print results
    print(f"\n=== Cone Constraint Block Results ===")
    print(f"Desired Direction: ({desired_direction[0]:.3f}, {desired_direction[1]:.3f}, {desired_direction[2]:.3f})")
    print(f"Projected Direction: ({projected_direction[0]:.3f}, {projected_direction[1]:.3f}, {projected_direction[2]:.3f})")
    print(f"Cone Apex: ({cone_apex[0]:.3f}, {cone_apex[1]:.3f}, {cone_apex[2]:.3f})")
    print(f"Cone Axis: ({cone_axis[0]:.3f}, {cone_axis[1]:.3f}, {cone_axis[2]:.3f})")
    print(f"Cone Angle: {np.degrees(cone_angle_rad):.1f}°")
    print(f"Constraint Applied: {constraint_applied}")
    print(f"Calculation Time: {calculation_time_ms:.3f}ms")


def calculate_cone_constraint(desired_direction: np.ndarray, cone_apex: np.ndarray, 
                            cone_axis: np.ndarray, cone_angle_rad: float, debug: bool = False):
    """
    Calculate cone constraint projection using C++ implementation
    
    Args:
        desired_direction: 3D direction vector to be constrained
        cone_apex: 3D position of cone apex (joint position)
        cone_axis: 3D direction of cone axis (will be normalized)
        cone_angle_rad: Full cone angle in radians
        debug: If True, show visualization
        
    Returns:
        Tuple of (projected_direction, constraint_applied, calculation_time_ms)
        
    Raises:
        ValueError: If input constraints are violated
    """
    
    try:
        # Call C++ implementation - pass numpy arrays directly
        projected_direction, constraint_applied, calculation_time_ms = delta_robot_cpp.calculate_cone_constraint(
            desired_direction, cone_apex, cone_axis, cone_angle_rad
        )
        
        # Optionally visualize
        if debug and VISUALIZATION_AVAILABLE:
            visualize_cone_constraint_result(desired_direction, cone_apex, cone_axis, cone_angle_rad,
                                           projected_direction, constraint_applied, calculation_time_ms)
        elif debug:
            print("Visualization not available, showing text results only")
            print(f"Input: desired_direction = {desired_direction}")
            print(f"Input: cone_apex = {cone_apex}")
            print(f"Input: cone_axis = {cone_axis}")
            print(f"Input: cone_angle = {np.degrees(cone_angle_rad):.1f}°")
            print(f"Output: projected_direction = {projected_direction}")
            print(f"Output: constraint_applied = {constraint_applied}")
            print(f"Output: calculation_time = {calculation_time_ms:.3f}ms")
        
        return projected_direction, constraint_applied, calculation_time_ms
        
    except Exception as e:
        raise ValueError(f"Cone constraint calculation failed: {str(e)}")


def calculate_cone_constraint_spherical_120(desired_direction: np.ndarray, cone_apex: np.ndarray, 
                                           cone_axis: np.ndarray, debug: bool = False):
    """
    Calculate 120° spherical joint cone constraint projection using C++ implementation
    
    Args:
        desired_direction: 3D direction vector to be constrained
        cone_apex: 3D position of cone apex (joint position)
        cone_axis: 3D direction of cone axis (will be normalized)
        debug: If True, show visualization
        
    Returns:
        Tuple of (projected_direction, constraint_applied, calculation_time_ms)
        
    Raises:
        ValueError: If input constraints are violated
    """
    
    try:
        # Call C++ implementation using 120° spherical joint constraint
        projected_direction, constraint_applied, calculation_time_ms = delta_robot_cpp.calculate_cone_constraint_spherical_120(
            desired_direction[0], desired_direction[1], desired_direction[2],
            cone_apex[0], cone_apex[1], cone_apex[2],
            cone_axis[0], cone_axis[1], cone_axis[2]
        )
        
        # Optionally visualize (using the 120° angle for visualization)
        if debug:
            cone_angle_rad = delta_robot_cpp.SPHERICAL_JOINT_CONE_ANGLE_RAD
            visualize_cone_constraint_result(desired_direction, cone_apex, cone_axis, cone_angle_rad,
                                           projected_direction, constraint_applied, calculation_time_ms)
        
        return projected_direction, constraint_applied, calculation_time_ms
        
    except Exception as e:
        raise ValueError(f"Spherical 120° cone constraint calculation failed: {str(e)}")