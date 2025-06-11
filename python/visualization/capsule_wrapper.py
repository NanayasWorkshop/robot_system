"""
Capsule Creation Block Python Wrapper - Clean Version
Real capsule visualization using cylinders with hemispherical caps
"""

import numpy as np
from base_plotter import create_base_3d_figure, add_robot_base_circle, ROBOT_COLORS
import plotly.graph_objects as go
import delta_robot_cpp


def create_capsule_mesh(start_point, end_point, radius, resolution=12):
    """Create a proper capsule mesh with cylindrical body and hemispherical caps"""
    start = np.array(start_point)
    end = np.array(end_point)
    
    # Calculate direction and length
    direction = end - start
    length = np.linalg.norm(direction)
    
    if length < 1e-6:
        return None, None, None, None  # Degenerate capsule
    
    # Normalize direction
    axis = direction / length
    
    # Find perpendicular vectors
    if abs(axis[2]) < 0.9:
        v1 = np.cross(axis, [0, 0, 1])
    else:
        v1 = np.cross(axis, [1, 0, 0])
    v1 = v1 / np.linalg.norm(v1)
    v2 = np.cross(axis, v1)
    
    vertices = []
    faces = []
    
    # 1. Create cylinder body vertices
    for i in range(resolution):
        angle = 2 * np.pi * i / resolution
        
        # Bottom circle
        bottom = start + radius * (np.cos(angle) * v1 + np.sin(angle) * v2)
        vertices.append(bottom)
        
        # Top circle  
        top = end + radius * (np.cos(angle) * v1 + np.sin(angle) * v2)
        vertices.append(top)
    
    # 2. Create hemisphere vertices
    hemisphere_res = resolution // 2
    
    # Bottom hemisphere (pointing towards start)
    for j in range(1, hemisphere_res + 1):
        phi = np.pi * j / (hemisphere_res + 1)  # 0 to pi/2
        
        for i in range(resolution):
            angle = 2 * np.pi * i / resolution
            
            # Sphere coordinates
            x = radius * np.sin(phi) * np.cos(angle)
            y = radius * np.sin(phi) * np.sin(angle)
            z = -radius * np.cos(phi)  # Negative for bottom hemisphere
            
            # Transform to world coordinates
            point = start + x * v1 + y * v2 + z * axis
            vertices.append(point)
    
    # Top hemisphere (pointing towards end)
    for j in range(1, hemisphere_res + 1):
        phi = np.pi * j / (hemisphere_res + 1)  # 0 to pi/2
        
        for i in range(resolution):
            angle = 2 * np.pi * i / resolution
            
            # Sphere coordinates
            x = radius * np.sin(phi) * np.cos(angle)
            y = radius * np.sin(phi) * np.sin(angle)
            z = radius * np.cos(phi)  # Positive for top hemisphere
            
            # Transform to world coordinates
            point = end + x * v1 + y * v2 + z * axis
            vertices.append(point)
    
    # Add center points for hemisphere caps
    vertices.append(start)  # Bottom center
    vertices.append(end)    # Top center
    
    vertices = np.array(vertices)
    
    # 3. Create faces
    
    # Cylinder side faces
    for i in range(resolution):
        next_i = (i + 1) % resolution
        
        # Two triangles per side face
        faces.append([2*i, 2*i+1, 2*next_i])
        faces.append([2*next_i, 2*i+1, 2*next_i+1])
    
    # Bottom hemisphere faces
    bottom_center_idx = len(vertices) - 2
    bottom_ring_start = 2 * resolution
    
    # Connect bottom circle to first hemisphere ring
    for i in range(resolution):
        next_i = (i + 1) % resolution
        faces.append([2*i, bottom_ring_start + i, 2*next_i])
        faces.append([2*next_i, bottom_ring_start + i, bottom_ring_start + next_i])
    
    # Hemisphere rings
    for j in range(hemisphere_res - 1):
        for i in range(resolution):
            next_i = (i + 1) % resolution
            
            curr_ring = bottom_ring_start + j * resolution
            next_ring = bottom_ring_start + (j + 1) * resolution
            
            faces.append([curr_ring + i, next_ring + i, curr_ring + next_i])
            faces.append([curr_ring + next_i, next_ring + i, next_ring + next_i])
    
    # Connect last hemisphere ring to center
    last_ring = bottom_ring_start + (hemisphere_res - 1) * resolution
    for i in range(resolution):
        next_i = (i + 1) % resolution
        faces.append([last_ring + i, bottom_center_idx, last_ring + next_i])
    
    # Top hemisphere faces (similar pattern)
    top_center_idx = len(vertices) - 1
    top_ring_start = bottom_ring_start + hemisphere_res * resolution
    
    # Connect top circle to first hemisphere ring
    for i in range(resolution):
        next_i = (i + 1) % resolution
        faces.append([2*i+1, 2*next_i+1, top_ring_start + i])
        faces.append([2*next_i+1, top_ring_start + next_i, top_ring_start + i])
    
    # Top hemisphere rings
    for j in range(hemisphere_res - 1):
        for i in range(resolution):
            next_i = (i + 1) % resolution
            
            curr_ring = top_ring_start + j * resolution
            next_ring = top_ring_start + (j + 1) * resolution
            
            faces.append([curr_ring + i, curr_ring + next_i, next_ring + i])
            faces.append([curr_ring + next_i, next_ring + next_i, next_ring + i])
    
    # Connect last top hemisphere ring to center
    last_top_ring = top_ring_start + (hemisphere_res - 1) * resolution
    for i in range(resolution):
        next_i = (i + 1) % resolution
        faces.append([last_top_ring + i, last_top_ring + next_i, top_center_idx])
    
    faces = np.array(faces)
    
    x = vertices[:, 0]
    y = vertices[:, 1]
    z = vertices[:, 2]
    i = faces[:, 0]
    j = faces[:, 1]
    k = faces[:, 2]
    
    return x, y, z, (i, j, k)


def visualize_capsule_chain(s_points, capsules, robot_radius):
    """Visualize S-points and capsule chain using proper cylinder meshes"""
    
    fig = create_base_3d_figure(f"Capsule Chain - {len(capsules)} Capsules")
    add_robot_base_circle(fig, delta_robot_cpp.ROBOT_RADIUS)
    
    # Plot S-points
    s_x = [pos[0] for pos in s_points]
    s_y = [pos[1] for pos in s_points]
    s_z = [pos[2] for pos in s_points]
    
    fig.add_trace(go.Scatter3d(
        x=s_x, y=s_y, z=s_z,
        mode='markers+text+lines',
        marker=dict(size=8, color='red'),
        line=dict(color='gray', width=2, dash='dot'),
        text=[f'S{i}' for i in range(len(s_points))],
        textposition='top center',
        name='S-Points'
    ))
    
    # Plot capsules as proper cylinder meshes
    for i, capsule in enumerate(capsules):
        start = capsule['start_point']
        end = capsule['end_point']
        radius = capsule['radius']
        
        # Create capsule mesh with rounded ends
        mesh_data = create_capsule_mesh(start, end, radius)
        
        if mesh_data[0] is not None:
            x, y, z, faces = mesh_data
            face_i, face_j, face_k = faces
            
            # Add the mesh
            fig.add_trace(go.Mesh3d(
                x=x, y=y, z=z,
                i=face_i, j=face_j, k=face_k,
                color='lightblue',
                opacity=0.7,
                name=f'Capsule {i+1}' if i == 0 else None,
                showlegend=(i == 0)
            ))
        
        # Add centerline for reference
        fig.add_trace(go.Scatter3d(
            x=[start[0], end[0]],
            y=[start[1], end[1]], 
            z=[start[2], end[2]],
            mode='lines',
            line=dict(color='orange', width=4),
            name='Centerlines' if i == 0 else None,
            showlegend=(i == 0)
        ))
    
    fig.show()


def create_capsule_chain(s_points, robot_radius, debug=False):
    """Create capsule chain from S-points using C++ implementation"""
    
    try:
        # Convert to numpy arrays
        s_point_vectors = [np.array(pos, dtype=np.float64) for pos in s_points]
        
        # Call C++ implementation
        result = delta_robot_cpp.create_capsule_chain(s_point_vectors, float(robot_radius))
        capsules_cpp, total_length, calc_time, success, error_msg = result
        
        # Convert to Python format
        capsules = []
        if success and capsules_cpp:
            for capsule in capsules_cpp:
                capsules.append({
                    'start_point': np.array(capsule.start_point),
                    'end_point': np.array(capsule.end_point),
                    'radius': capsule.radius,
                    'length': capsule.length
                })
        
        if debug:
            print(f"Created {len(capsules)} capsules, Total length: {total_length:.1f}mm")
        
        return capsules, total_length, calc_time, success, error_msg
        
    except Exception as e:
        return [], 0.0, 0.0, False, str(e)


def update_capsule_positions(existing_capsules, new_s_points, debug=False):
    """Update existing capsule positions with new S-points"""
    
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
        
        # Convert new S-points
        s_point_vectors = [np.array(pos, dtype=np.float64) for pos in new_s_points]
        
        # Call C++ implementation
        result = delta_robot_cpp.update_capsule_positions(cpp_capsules, s_point_vectors)
        capsules_cpp, total_length, calc_time, success, error_msg = result
        
        # Convert to Python format
        updated_capsules = []
        if success and capsules_cpp:
            for capsule in capsules_cpp:
                updated_capsules.append({
                    'start_point': np.array(capsule.start_point),
                    'end_point': np.array(capsule.end_point),
                    'radius': capsule.radius,
                    'length': capsule.length
                })
        
        if debug:
            print(f"Updated {len(updated_capsules)} capsules, Total length: {total_length:.1f}mm")
        
        return updated_capsules, total_length, calc_time, success, error_msg
        
    except Exception as e:
        return [], 0.0, 0.0, False, str(e)