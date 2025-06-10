"""
Segment Block Python Wrapper
Pure visualization wrapper - calls C++ and plots results
"""

import numpy as np
from base_plotter import create_base_3d_figure, add_robot_base_circle, add_vector_arrow, ROBOT_COLORS
import plotly.graph_objects as go

import delta_robot_cpp

def visualize_all_segments_and_joints(joint_positions: list) -> None:
    """Visualize all J points and calculated S points in a single comprehensive plot"""
    
    # Get constants from C++
    ROBOT_RADIUS = delta_robot_cpp.ROBOT_RADIUS
    
    # Calculate maximum possible segments (num_joints - 1)
    max_possible_segments = len(joint_positions) - 1
    
    # Create base figure with coordinate system
    fig = create_base_3d_figure(f"Complete J→S Conversion Visualization - {len(joint_positions)} Joints, {max_possible_segments} Segments")
    
    # Add robot base circle for reference
    add_robot_base_circle(fig, ROBOT_RADIUS)
    
    # Plot all joint positions
    joint_x = [pos[0] for pos in joint_positions]
    joint_y = [pos[1] for pos in joint_positions]
    joint_z = [pos[2] for pos in joint_positions]
    
    fig.add_trace(go.Scatter3d(
        x=joint_x,
        y=joint_y,
        z=joint_z,
        mode='markers+text+lines',
        marker=dict(size=8, color=ROBOT_COLORS['joint_points']),
        line=dict(color=ROBOT_COLORS['joint_chain'], width=3, dash='dot'),
        text=[f'J{i}' for i in range(len(joint_positions))],
        textposition='top center',
        name='Joint Chain'
    ))
    
    # Add S0 base point at J0 (start of S-point chain)
    s0_base_point = joint_positions[0]  # S0 is at J0 position
    
    # Plot S0 base point
    fig.add_trace(go.Scatter3d(
        x=[s0_base_point[0]],
        y=[s0_base_point[1]],
        z=[s0_base_point[2]],
        mode='markers+text',
        marker=dict(size=12, color='darkred', symbol='diamond'),
        text=['S0'],
        textposition='top center',
        name='S0 Base'
    ))
    
    # Calculate and plot S points for ALL possible segments
    s_points = [s0_base_point]  # Start with S0 base point
    s_directions = []
    prismatic_lengths = []
    joint_calculations = []  # Store joint state data
    kinematic_calculations = []  # Store kinematic data
    
    for segment_idx in range(max_possible_segments):
        try:
            # Get segment result with J→S conversion
            result = delta_robot_cpp.calculate_segment_essential_from_joints(
                [np.array(pos, dtype=float) for pos in joint_positions], 
                segment_idx
            )
            prismatic_length, calc_time, success, error_msg, calc_seg_pos, calc_dir = result
            
            if success and calc_seg_pos is not None:
                s_points.append(calc_seg_pos)
                s_directions.append(calc_dir)
                prismatic_lengths.append(prismatic_length)
                
                # Calculate joint states from the calculated direction
                try:
                    joint_result = delta_robot_cpp.calculate_joint_state(calc_dir[0], calc_dir[1], calc_dir[2])
                    joint_prismatic, joint_roll, joint_pitch, joint_calc_time, joint_fermat_data = joint_result
                    joint_calculations.append({
                        'prismatic': joint_prismatic,
                        'roll': joint_roll,
                        'pitch': joint_pitch
                    })
                except:
                    joint_calculations.append({
                        'prismatic': 0.0,
                        'roll': 0.0,
                        'pitch': 0.0
                    })
                
                # Calculate kinematics (end-effector) from the calculated direction
                try:
                    kinematic_result = delta_robot_cpp.calculate_kinematics(calc_dir[0], calc_dir[1], calc_dir[2])
                    point_H, point_G, HG_length, end_effector, kinematic_calc_time, kinematic_fermat_data = kinematic_result
                    kinematic_calculations.append({
                        'end_effector': end_effector,
                        'point_H': point_H,
                        'point_G': point_G,
                        'HG_length': HG_length
                    })
                except:
                    kinematic_calculations.append({
                        'end_effector': [0.0, 0.0, 0.0],
                        'point_H': [0.0, 0.0, 0.0],
                        'point_G': [0.0, 0.0, 0.0],
                        'HG_length': 0.0
                    })
                
                # Plot individual S point - FIXED NUMBERING (S1, S2, S3...)
                fig.add_trace(go.Scatter3d(
                    x=[calc_seg_pos[0]],
                    y=[calc_seg_pos[1]],
                    z=[calc_seg_pos[2]],
                    mode='markers+text',
                    marker=dict(size=12, color=ROBOT_COLORS['segment_points'], symbol='diamond'),
                    text=[f'S{segment_idx+1}'],  # FIXED: segment_idx 0 → S1, 1 → S2, etc.
                    textposition='top center',
                    name=f'Segment S{segment_idx+1}'
                ))
                
                # Direction vector from S point
                normalized_direction = np.array(calc_dir) / np.linalg.norm(calc_dir)
                direction_end = np.array(calc_seg_pos) + normalized_direction * 30
                
                # Use color from segment_directions array, cycling if needed
                direction_color = ROBOT_COLORS['segment_directions'][segment_idx % len(ROBOT_COLORS['segment_directions'])]
                add_vector_arrow(fig, calc_seg_pos, direction_end, 
                               color=direction_color, 
                               name=f'Dir S{segment_idx+1}')  # FIXED: S1, S2, S3...
                
            else:
                print(f"Failed to calculate segment {segment_idx}: {error_msg}")
                
        except Exception as e:
            print(f"Error calculating segment {segment_idx}: {e}")
    
    # Connect ALL S points (S0 → S1 → S2 → ... → S8) for complete chain
    if len(s_points) > 1:
        s_x = [pos[0] for pos in s_points]
        s_y = [pos[1] for pos in s_points]
        s_z = [pos[2] for pos in s_points]
        
        fig.add_trace(go.Scatter3d(
            x=s_x,
            y=s_y,
            z=s_z,
            mode='lines',
            line=dict(color=ROBOT_COLORS['segment_chain'], width=5, dash='dash'),
            name='Complete S-Chain (S0→S8)',
            showlegend=True
        ))
    
    fig.show()
    
    # Print comprehensive results
    print(f"\n=== Complete J→S Conversion Results ===")
    print(f"Total Joints: {len(joint_positions)}")
    print(f"Maximum Possible Segments: {max_possible_segments}")
    print(f"Segments Calculated: {len(s_points)-1}")  # Subtract 1 for S0 base point
    print(f"Success Rate: {len(s_points)-1}/{max_possible_segments} ({100*(len(s_points)-1)/max_possible_segments:.1f}%)")
    print(f"S0 Base Point: ({s_points[0][0]:.3f}, {s_points[0][1]:.3f}, {s_points[0][2]:.3f}) [at J0]")
    
    for i, (s_point, direction, seg_length, joint_calc, kinematic_calc) in enumerate(zip(s_points[1:], s_directions, prismatic_lengths, joint_calculations, kinematic_calculations)):
        print(f"Segment S{i+1}:")  # FIXED: S1, S2, S3...
        print(f"  S-Point: ({s_point[0]:.3f}, {s_point[1]:.3f}, {s_point[2]:.3f})")
        print(f"  End-Effector: ({kinematic_calc['end_effector'][0]:.3f}, {kinematic_calc['end_effector'][1]:.3f}, {kinematic_calc['end_effector'][2]:.3f})")
        print(f"  Direction: ({direction[0]:.3f}, {direction[1]:.3f}, {direction[2]:.3f})")
        print(f"  Segment Prismatic Length: {seg_length:.3f}")
        print(f"  Joint State - Prismatic: {joint_calc['prismatic']:.3f}, Roll: {np.degrees(joint_calc['roll']):.1f}°, Pitch: {np.degrees(joint_calc['pitch']):.1f}°")


def calculate_segment_essential(x: float, y: float, z: float, 
                              prev_x: float = 0.0, prev_y: float = 0.0, prev_z: float = 1.0,
                              debug: bool = False):
    """
    Calculate segment prismatic length using C++ implementation (direction-based)
    
    Args:
        x, y, z: Current direction vector components (z must be positive after transformation)
        prev_x, prev_y, prev_z: Previous direction vector components
        debug: If True, show visualization
        
    Returns:
        Tuple of (prismatic_length, calculation_time_ms, calculation_successful, error_message)
        
    Raises:
        ValueError: If input constraints are violated
    """
    
    try:
        # Call C++ implementation - validation happens in C++
        prismatic_length, calculation_time_ms, calculation_successful, error_message = delta_robot_cpp.calculate_segment_essential(x, y, z, prev_x, prev_y, prev_z)
        
        # Print results if debug mode
        if debug:
            print(f"\n=== Segment Block Results (Essential) ===")
            print(f"Current Direction: ({x:.3f}, {y:.3f}, {z:.3f})")
            print(f"Previous Direction: ({prev_x:.3f}, {prev_y:.3f}, {prev_z:.3f})")
            print(f"Prismatic Length: {prismatic_length:.3f}")
            print(f"Calculation Time: {calculation_time_ms:.3f}ms")
            print(f"Status: {'SUCCESS' if calculation_successful else 'FAILED'}")
            if not calculation_successful:
                print(f"Error: {error_message}")
        
        return prismatic_length, calculation_time_ms, calculation_successful, error_message
        
    except Exception as e:
        raise ValueError(f"Segment essential calculation failed: {str(e)}")


def calculate_segment_essential_from_joints(joint_positions: list, segment_index: int, debug: bool = False):
    """
    Calculate segment prismatic length from FABRIK joints using C++ implementation
    
    Args:
        joint_positions: List of joint positions as [x, y, z] arrays or tuples
        segment_index: Which segment to calculate (0=S1, 1=S2, etc.)
        debug: If True, show visualization
        
    Returns:
        Tuple of (prismatic_length, calculation_time_ms, calculation_successful, error_message,
                 calculated_segment_position, calculated_direction)
        
    Raises:
        ValueError: If input constraints are violated
    """
    
    try:
        # Convert to list of Eigen vectors for C++
        joint_vectors = [np.array(pos, dtype=float) for pos in joint_positions]
        
        # Call C++ implementation - validation happens in C++
        result = delta_robot_cpp.calculate_segment_essential_from_joints(joint_vectors, segment_index)
        prismatic_length, calculation_time_ms, calculation_successful, error_message, calc_seg_pos, calc_dir = result
        
        # Convert None values from C++ to actual None in Python
        calculated_segment_position = np.array(calc_seg_pos) if calc_seg_pos is not None else None
        calculated_direction = np.array(calc_dir) if calc_dir is not None else None
        
        # Print results if debug mode
        if debug:
            print(f"\n=== Segment Block Results (From Joints) ===")
            print(f"Segment Index: {segment_index}")
            print(f"Number of Joints: {len(joint_positions)}")
            print(f"Prismatic Length: {prismatic_length:.3f}")
            if calculation_successful and calculated_segment_position is not None:
                print(f"Calculated S-Point: ({calculated_segment_position[0]:.3f}, {calculated_segment_position[1]:.3f}, {calculated_segment_position[2]:.3f})")
                print(f"Calculated Direction: ({calculated_direction[0]:.3f}, {calculated_direction[1]:.3f}, {calculated_direction[2]:.3f})")
            print(f"Calculation Time: {calculation_time_ms:.3f}ms")
            print(f"Status: {'SUCCESS' if calculation_successful else 'FAILED'}")
            if not calculation_successful:
                print(f"Error: {error_message}")
        
        return prismatic_length, calculation_time_ms, calculation_successful, error_message, calculated_segment_position, calculated_direction
        
    except Exception as e:
        raise ValueError(f"Segment essential from joints calculation failed: {str(e)}")


def calculate_segment_complete(x: float, y: float, z: float,
                             prev_x: float = 0.0, prev_y: float = 0.0, prev_z: float = 1.0,
                             debug: bool = False):
    """
    Calculate segment with full analysis using C++ implementation (direction-based)
    
    Args:
        x, y, z: Current direction vector components (z must be positive after transformation)
        prev_x, prev_y, prev_z: Previous direction vector components
        debug: If True, show visualization
        
    Returns:
        Tuple of (prismatic_length, calculation_time_ms, calculation_successful, error_message,
                 has_kinematics_data, has_joint_state_data, has_orientation_data)
        
    Raises:
        ValueError: If input constraints are violated
    """
    
    try:
        # Call C++ implementation - validation happens in C++
        result = delta_robot_cpp.calculate_segment_complete(x, y, z, prev_x, prev_y, prev_z)
        prismatic_length, calculation_time_ms, calculation_successful, error_message, has_kinematics, has_joint_state, has_orientation = result
        
        # Print results if debug mode
        if debug:
            print(f"\n=== Segment Block Results (Complete) ===")
            print(f"Current Direction: ({x:.3f}, {y:.3f}, {z:.3f})")
            print(f"Previous Direction: ({prev_x:.3f}, {prev_y:.3f}, {prev_z:.3f})")
            print(f"Prismatic Length: {prismatic_length:.3f}")
            print(f"Calculation Time: {calculation_time_ms:.3f}ms")
            print(f"Success: {calculation_successful}")
            print(f"Has Kinematics Data: {has_kinematics}")
            print(f"Has Joint State Data: {has_joint_state}")
            print(f"Has Orientation Data: {has_orientation}")
            if not calculation_successful:
                print(f"Error: {error_message}")
        
        return prismatic_length, calculation_time_ms, calculation_successful, error_message, has_kinematics, has_joint_state, has_orientation
        
    except Exception as e:
        raise ValueError(f"Segment complete calculation failed: {str(e)}")


def calculate_segment_complete_from_joints(joint_positions: list, segment_index: int, debug: bool = False):
    """
    Calculate segment with full analysis from FABRIK joints using C++ implementation
    
    Args:
        joint_positions: List of joint positions as [x, y, z] arrays or tuples
        segment_index: Which segment to calculate (0=S1, 1=S2, etc.)
        debug: If True, show visualization
        
    Returns:
        Tuple of (prismatic_length, calculation_time_ms, calculation_successful, error_message,
                 calculated_segment_position, calculated_direction,
                 has_kinematics_data, has_joint_state_data, has_orientation_data)
        
    Raises:
        ValueError: If input constraints are violated
    """
    
    try:
        # Convert to list of Eigen vectors for C++
        joint_vectors = [np.array(pos, dtype=float) for pos in joint_positions]
        
        # Call C++ implementation - validation happens in C++
        result = delta_robot_cpp.calculate_segment_complete_from_joints(joint_vectors, segment_index)
        prismatic_length, calculation_time_ms, calculation_successful, error_message, calc_seg_pos, calc_dir, has_kinematics, has_joint_state, has_orientation = result
        
        # Convert None values from C++ to actual None in Python
        calculated_segment_position = np.array(calc_seg_pos) if calc_seg_pos is not None else None
        calculated_direction = np.array(calc_dir) if calc_dir is not None else None
        
        # Print results if debug mode
        if debug:
            print(f"\n=== Segment Block Results (Complete from Joints) ===")
            print(f"Segment Index: {segment_index}")
            print(f"Number of Joints: {len(joint_positions)}")
            print(f"Prismatic Length: {prismatic_length:.3f}")
            print(f"Calculation Time: {calculation_time_ms:.3f}ms")
            print(f"Success: {calculation_successful}")
            print(f"Has Kinematics Data: {has_kinematics}")
            print(f"Has Joint State Data: {has_joint_state}")
            print(f"Has Orientation Data: {has_orientation}")
            if calculation_successful and calculated_segment_position is not None:
                print(f"Calculated S-Point: ({calculated_segment_position[0]:.3f}, {calculated_segment_position[1]:.3f}, {calculated_segment_position[2]:.3f})")
                print(f"Calculated Direction: ({calculated_direction[0]:.3f}, {calculated_direction[1]:.3f}, {calculated_direction[2]:.3f})")
            if not calculation_successful:
                print(f"Error: {error_message}")
        
        return prismatic_length, calculation_time_ms, calculation_successful, error_message, calculated_segment_position, calculated_direction, has_kinematics, has_joint_state, has_orientation
        
    except Exception as e:
        raise ValueError(f"Segment complete from joints calculation failed: {str(e)}")