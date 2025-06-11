"""
Capsule Creation Block Python Wrapper
Pure visualization wrapper - calls C++ and plots results
"""

import numpy as np
from base_plotter import create_base_3d_figure, add_robot_base_circle, ROBOT_COLORS
import plotly.graph_objects as go

import delta_robot_cpp

def visualize_capsule_chain(s_points: list, capsules: list, robot_radius: float) -> None:
    """Visualize S-points and created capsule chain"""
    
    # Create base figure with coordinate system
    fig = create_base_3d_figure(f"Capsule Creation Block - {len(capsules)} Capsules from {len(s_points)} S-Points")
    
    # Add robot base circle for reference
    add_robot_base_circle(fig, delta_robot_cpp.ROBOT_RADIUS)
    
    # Plot S-points
    s_x = [pos[0] for pos in s_points]
    s_y = [pos[1] for pos in s_points]
    s_z = [pos[2] for pos in s_points]
    
    fig.add_trace(go.Scatter3d(
        x=s_x,
        y=s_y,
        z=s_z,
        mode='markers+text+lines',
        marker=dict(size=10, color=ROBOT_COLORS['segment_points'], symbol='diamond'),
        line=dict(color=ROBOT_COLORS['segment_chain'], width=3, dash='dot'),
        text=[f'S{i}' for i in range(len(s_points))],
        textposition='top center',
        name='S-Point Chain'
    ))
    
    # Plot capsules as cylinders
    for i, capsule in enumerate(capsules):
        start = capsule['start_point']
        end = capsule['end_point']
        radius = capsule['radius']
        
        # Create cylinder representation
        # Direction vector
        direction = np.array(end) - np.array(start)
        length = np.linalg.norm(direction)
        
        if length > 0:
            # Normalized direction
            dir_norm = direction / length
            
            # Create cylinder points along the capsule
            t_values = np.linspace(0, 1, 20)
            cylinder_points = []
            
            for t in t_values:
                center = np.array(start) + t * direction
                
                # Create circle perpendicular to direction
                # Find two perpendicular vectors to direction
                if abs(dir_norm[2]) < 0.9:
                    v1 = np.cross(dir_norm, [0, 0, 1])
                else:
                    v1 = np.cross(dir_norm, [1, 0, 0])
                v1 = v1 / np.linalg.norm(v1)
                v2 = np.cross(dir_norm, v1)
                v2 = v2 / np.linalg.norm(v2)
                
                # Create circle points
                circle_angles = np.linspace(0, 2*np.pi, 12)
                for angle in circle_angles:
                    point = center + radius * (np.cos(angle) * v1 + np.sin(angle) * v2)
                    cylinder_points.append(point)
            
            # Plot cylinder as surface
            if cylinder_points:
                points_array = np.array(cylinder_points)
                
                # Plot as scatter points for simplicity
                fig.add_trace(go.Scatter3d(
                    x=points_array[:, 0],
                    y=points_array[:, 1],
                    z=points_array[:, 2],
                    mode='markers',
                    marker=dict(size=2, color=f'rgba(255, 165, 0, 0.3)'),
                    name=f'Capsule {i+1}' if i == 0 else None,
                    showlegend=(i == 0)
                ))
        
        # Plot capsule centerline
        fig.add_trace(go.Scatter3d(
            x=[start[0], end[0]],
            y=[start[1], end[1]],
            z=[start[2], end[2]],
            mode='lines',
            line=dict(color='orange', width=8),
            name=f'Capsule {i+1} Center' if i == 0 else None,
            showlegend=(i == 0)
        ))
        
        # Plot capsule endpoints
        fig.add_trace(go.Scatter3d(
            x=[start[0], end[0]],
            y=[start[1], end[1]],
            z=[start[2], end[2]],
            mode='markers',
            marker=dict(size=8, color=['red', 'blue'], symbol='circle'),
            name=f'Capsule {i+1} Endpoints' if i == 0 else None,
            showlegend=(i == 0)
        ))
    
    fig.show()
    
    # Print results
    print(f"\n=== Capsule Creation Block Results ===")
    print(f"Input S-Points: {len(s_points)}")
    print(f"Created Capsules: {len(capsules)}")
    print(f"Robot Radius: {robot_radius:.3f}mm")
    
    total_length = 0.0
    for i, capsule in enumerate(capsules):
        length = capsule['length']
        total_length += length
        print(f"Capsule {i+1}: Length={length:.3f}mm, Radius={capsule['radius']:.3f}mm")
    
    print(f"Total Chain Length: {total_length:.3f}mm")


def create_capsule_chain(s_points: list, robot_radius: float, debug: bool = False):
    """
    Create capsule chain from S-points using C++ implementation
    
    Args:
        s_points: List of S-point positions as [x, y, z] arrays or tuples
        robot_radius: Radius for all capsules in millimeters
        debug: If True, show visualization
        
    Returns:
        Tuple of (capsules, total_chain_length, calculation_time_ms, creation_successful, error_message)
        
    Raises:
        ValueError: If input constraints are violated
    """
    
    try:
        # Convert to list of Eigen vectors for C++
        s_point_vectors = [np.array(pos, dtype=float) for pos in s_points]
        
        # Call C++ implementation - validation happens in C++
        result = delta_robot_cpp.create_capsule_chain(s_point_vectors, robot_radius)
        capsules, total_chain_length, calculation_time_ms, creation_successful, error_message = result
        
        # Convert capsules to Python-friendly format
        capsule_list = []
        if creation_successful and capsules:
            for capsule in capsules:
                capsule_dict = {
                    'start_point': np.array(capsule.start_point),
                    'end_point': np.array(capsule.end_point),
                    'radius': capsule.radius,
                    'length': capsule.length
                }
                capsule_list.append(capsule_dict)
        
        # Optionally visualize
        if debug and creation_successful:
            visualize_capsule_chain(s_points, capsule_list, robot_radius)
        
        # Print results if debug mode
        if debug:
            print(f"\n=== Capsule Creation Block Results ===")
            print(f"Input S-Points: {len(s_points)}")
            print(f"Robot Radius: {robot_radius:.3f}mm")
            print(f"Created Capsules: {len(capsule_list)}")
            print(f"Total Chain Length: {total_chain_length:.3f}mm")
            print(f"Calculation Time: {calculation_time_ms:.3f}ms")
            print(f"Status: {'SUCCESS' if creation_successful else 'FAILED'}")
            if not creation_successful:
                print(f"Error: {error_message}")
        
        return capsule_list, total_chain_length, calculation_time_ms, creation_successful, error_message
        
    except Exception as e:
        raise ValueError(f"Capsule creation failed: {str(e)}")


def update_capsule_positions(existing_capsules: list, new_s_points: list, debug: bool = False):
    """
    Update existing capsule positions with new S-points using C++ implementation
    
    Args:
        existing_capsules: List of existing capsule dictionaries
        new_s_points: List of new S-point positions as [x, y, z] arrays or tuples
        debug: If True, show visualization
        
    Returns:
        Tuple of (updated_capsules, total_chain_length, calculation_time_ms, creation_successful, error_message)
        
    Raises:
        ValueError: If input constraints are violated
    """
    
    try:
        # Convert existing capsules to C++ format
        cpp_capsules = []
        for capsule in existing_capsules:
            cpp_capsule = delta_robot_cpp.CapsuleData()
            cpp_capsule.start_point = capsule['start_point']
            cpp_capsule.end_point = capsule['end_point']
            cpp_capsule.radius = capsule['radius']
            cpp_capsule.length = capsule['length']
            cpp_capsules.append(cpp_capsule)
        
        # Convert new S-points to Eigen vectors
        s_point_vectors = [np.array(pos, dtype=float) for pos in new_s_points]
        
        # Call C++ implementation
        result = delta_robot_cpp.update_capsule_positions(cpp_capsules, s_point_vectors)
        capsules, total_chain_length, calculation_time_ms, creation_successful, error_message = result
        
        # Convert updated capsules to Python-friendly format
        updated_capsule_list = []
        if creation_successful and capsules:
            for capsule in capsules:
                capsule_dict = {
                    'start_point': np.array(capsule.start_point),
                    'end_point': np.array(capsule.end_point),
                    'radius': capsule.radius,
                    'length': capsule.length
                }
                updated_capsule_list.append(capsule_dict)
        
        # Optionally visualize
        if debug and creation_successful:
            visualize_capsule_chain(new_s_points, updated_capsule_list, updated_capsule_list[0]['radius'])
        
        # Print results if debug mode
        if debug:
            print(f"\n=== Capsule Update Block Results ===")
            print(f"Input Capsules: {len(existing_capsules)}")
            print(f"New S-Points: {len(new_s_points)}")
            print(f"Updated Capsules: {len(updated_capsule_list)}")
            print(f"Total Chain Length: {total_chain_length:.3f}mm")
            print(f"Calculation Time: {calculation_time_ms:.3f}ms")
            print(f"Status: {'SUCCESS' if creation_successful else 'FAILED'}")
            if not creation_successful:
                print(f"Error: {error_message}")
        
        return updated_capsule_list, total_chain_length, calculation_time_ms, creation_successful, error_message
        
    except Exception as e:
        raise ValueError(f"Capsule update failed: {str(e)}")