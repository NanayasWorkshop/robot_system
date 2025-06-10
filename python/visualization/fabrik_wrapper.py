"""
FABRIK Python Wrapper
Complete FABRIK solving with visualization
"""

import numpy as np
from base_plotter import create_base_3d_figure, add_robot_base_circle, add_vector_arrow, ROBOT_COLORS
import plotly.graph_objects as go

import delta_robot_cpp

def visualize_fabrik_result(target_position: np.ndarray, final_joints: list, 
                           initial_joints: list, iterations_used: int, 
                           converged: bool, final_distance: float, 
                           calculation_time_ms: float) -> None:
    """Visualize complete FABRIK solving results"""
    
    # Get constants from C++
    ROBOT_RADIUS = delta_robot_cpp.ROBOT_RADIUS
    
    # Create base figure with coordinate system
    title = f"FABRIK Solver Results - Target: ({target_position[0]:.0f}, {target_position[1]:.0f}, {target_position[2]:.0f})<br>"
    title += f"Iterations: {iterations_used}, Converged: {converged}, Distance: {final_distance:.3f}mm, Time: {calculation_time_ms:.1f}ms"
    fig = create_base_3d_figure(title)
    
    # Add robot base circle for reference
    add_robot_base_circle(fig, ROBOT_RADIUS)
    
    # Target position
    fig.add_trace(go.Scatter3d(
        x=[target_position[0]],
        y=[target_position[1]],
        z=[target_position[2]],
        mode='markers+text',
        marker=dict(size=15, color='red', symbol='x'),
        text=['TARGET'],
        textposition='top center',
        name='Target Position'
    ))
    
    # Initial joint chain
    initial_x = [pos[0] for pos in initial_joints]
    initial_y = [pos[1] for pos in initial_joints]
    initial_z = [pos[2] for pos in initial_joints]
    
    fig.add_trace(go.Scatter3d(
        x=initial_x,
        y=initial_y,
        z=initial_z,
        mode='lines+markers',
        line=dict(color='lightgray', width=3, dash='dot'),
        marker=dict(size=6, color='lightgray'),
        name='Initial Chain'
    ))
    
    # Final joint chain
    final_x = [pos[0] for pos in final_joints]
    final_y = [pos[1] for pos in final_joints]
    final_z = [pos[2] for pos in final_joints]
    
    fig.add_trace(go.Scatter3d(
        x=final_x,
        y=final_y,
        z=final_z,
        mode='lines+markers+text',
        line=dict(color=ROBOT_COLORS['joint_chain'], width=5),
        marker=dict(size=8, color=ROBOT_COLORS['joint_points']),
        text=[f'J{i}' for i in range(len(final_joints))],
        textposition='top center',
        name='Final Chain'
    ))
    
    # Highlight base and end-effector
    fig.add_trace(go.Scatter3d(
        x=[final_joints[0][0]],
        y=[final_joints[0][1]],
        z=[final_joints[0][2]],
        mode='markers+text',
        marker=dict(size=12, color='darkblue'),
        text=['BASE'],
        name='Base Joint'
    ))
    
    fig.add_trace(go.Scatter3d(
        x=[final_joints[-1][0]],
        y=[final_joints[-1][1]],
        z=[final_joints[-1][2]],
        mode='markers+text',
        marker=dict(size=12, color=ROBOT_COLORS['end_effector']),
        text=['END-EFFECTOR'],
        name='End-Effector'
    ))
    
    # Vector from end-effector to target (error vector)
    if final_distance > 0.1:  # Only show if significant error
        add_vector_arrow(fig, final_joints[-1], target_position, 
                        color='red', name=f'Error Vector ({final_distance:.1f}mm)', width=3)
    
    fig.show()
    
    # Print comprehensive results
    print(f"\n=== FABRIK Solver Results ===")
    print(f"Target Position: ({target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f})")
    print(f"Initial End-Effector: ({initial_joints[-1][0]:.3f}, {initial_joints[-1][1]:.3f}, {initial_joints[-1][2]:.3f})")
    print(f"Final End-Effector: ({final_joints[-1][0]:.3f}, {final_joints[-1][1]:.3f}, {final_joints[-1][2]:.3f})")
    print(f"Iterations Used: {iterations_used}")
    print(f"Converged: {'Yes' if converged else 'No'}")
    print(f"Final Distance to Target: {final_distance:.3f}mm")
    print(f"Total Calculation Time: {calculation_time_ms:.3f}ms")
    
    # Calculate initial vs final improvement
    initial_distance = np.linalg.norm(np.array(initial_joints[-1]) - target_position)
    improvement = initial_distance - final_distance
    improvement_percent = (improvement / initial_distance) * 100 if initial_distance > 0 else 0
    
    print(f"Distance Improvement: {initial_distance:.3f}mm → {final_distance:.3f}mm")
    print(f"Improvement: {improvement:.3f}mm ({improvement_percent:.1f}%)")


def test_fabrik_solver(target_x: float, target_y: float, target_z: float, 
                      num_segments: int = 7, tolerance: float = 0.01, 
                      max_iterations: int = 100, debug: bool = False):
    """
    Test complete FABRIK solver using C++ implementation
    
    Args:
        target_x, target_y, target_z: Target position components
        num_segments: Number of robot segments (default: 7)
        tolerance: Convergence tolerance (default: 0.01)
        max_iterations: Maximum iterations (default: 100)
        debug: If True, show visualization
        
    Returns:
        Tuple of (final_joints, iterations_used, final_distance_to_target, converged,
                 calculation_time_ms, solving_successful, error_message, initialization_data, initial_joints)
        
    Raises:
        ValueError: If solving fails
    """
    
    try:
        # Call C++ implementation
        target_position = np.array([target_x, target_y, target_z], dtype=float)
        
        result = delta_robot_cpp.fabrik_solve(
            target_position, num_segments, tolerance, max_iterations
        )
        
        final_joints, iterations_used, final_distance_to_target, converged, calculation_time_ms, solving_successful, error_message, initialization_data, initial_joints = result
        
        if not solving_successful:
            raise ValueError(f"FABRIK solving failed: {error_message}")
        
        # Optionally visualize
        if debug:
            visualize_fabrik_result(target_position, final_joints, initial_joints,
                                  iterations_used, converged, final_distance_to_target,
                                  calculation_time_ms)
        
        return final_joints, iterations_used, final_distance_to_target, converged, calculation_time_ms, solving_successful, error_message, initialization_data, initial_joints
        
    except Exception as e:
        raise ValueError(f"FABRIK solver test failed: {str(e)}")


def test_fabrik_initialization(num_segments: int = 7, debug: bool = False):
    """
    Test FABRIK initialization using C++ implementation
    
    Args:
        num_segments: Number of robot segments (default: 7)
        debug: If True, show detailed output
        
    Returns:
        Tuple of (initial_joints, joint_distances, num_segments, num_joints,
                 calculation_time_ms, initialization_successful, error_message)
        
    Raises:
        ValueError: If initialization fails
    """
    
    try:
        # Call C++ implementation
        result = delta_robot_cpp.create_fabrik_straight_chain(num_segments)
        initial_joints, joint_distances, num_segments_out, num_joints, calculation_time_ms, initialization_successful, error_message = result
        
        if not initialization_successful:
            raise ValueError(f"FABRIK initialization failed: {error_message}")
        
        # Print results if debug mode
        if debug:
            print(f"\n=== FABRIK Initialization Results ===")
            print(f"Number of Segments: {num_segments_out}")
            print(f"Number of Joints: {num_joints}")
            print(f"Joint Distances: {[f'{d:.1f}mm' for d in joint_distances]}")
            print(f"Calculation Time: {calculation_time_ms:.3f}ms")
            print(f"First Joint: ({initial_joints[0][0]:.1f}, {initial_joints[0][1]:.1f}, {initial_joints[0][2]:.1f})")
            print(f"Last Joint: ({initial_joints[-1][0]:.1f}, {initial_joints[-1][1]:.1f}, {initial_joints[-1][2]:.1f})")
        
        return initial_joints, joint_distances, num_segments_out, num_joints, calculation_time_ms, initialization_successful, error_message
        
    except Exception as e:
        raise ValueError(f"FABRIK initialization test failed: {str(e)}")


def test_fabrik_iteration(current_joints: list, target_x: float, target_y: float, target_z: float,
                         joint_distances: list, debug: bool = False):
    """
    Test single FABRIK iteration using C++ implementation
    
    Args:
        current_joints: Current joint positions as list of [x,y,z]
        target_x, target_y, target_z: Target position components
        joint_distances: Distance constraints between joints
        debug: If True, show detailed output
        
    Returns:
        Tuple of (updated_joints, distance_to_target, calculation_time_ms, 
                 iteration_successful, error_message)
        
    Raises:
        ValueError: If iteration fails
    """
    
    try:
        # Convert inputs to proper format
        joint_vectors = [np.array(pos, dtype=float) for pos in current_joints]
        target_position = np.array([target_x, target_y, target_z], dtype=float)
        
        # Call C++ implementation
        result = delta_robot_cpp.fabrik_iterate(joint_vectors, target_position, joint_distances)
        updated_joints, distance_to_target, calculation_time_ms, iteration_successful, error_message = result
        
        if not iteration_successful:
            raise ValueError(f"FABRIK iteration failed: {error_message}")
        
        # Print results if debug mode
        if debug:
            print(f"\n=== FABRIK Iteration Results ===")
            print(f"Target Position: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
            print(f"Distance to Target: {distance_to_target:.3f}mm")
            print(f"Calculation Time: {calculation_time_ms:.3f}ms")
            print(f"Initial End-Effector: ({current_joints[-1][0]:.3f}, {current_joints[-1][1]:.3f}, {current_joints[-1][2]:.3f})")
            print(f"Updated End-Effector: ({updated_joints[-1][0]:.3f}, {updated_joints[-1][1]:.3f}, {updated_joints[-1][2]:.3f})")
            
            # Calculate improvement
            initial_distance = np.linalg.norm(np.array(current_joints[-1]) - target_position)
            improvement = initial_distance - distance_to_target
            print(f"Distance Improvement: {initial_distance:.3f}mm → {distance_to_target:.3f}mm ({improvement:.3f}mm)")
        
        return updated_joints, distance_to_target, calculation_time_ms, iteration_successful, error_message
        
    except Exception as e:
        raise ValueError(f"FABRIK iteration test failed: {str(e)}")


def compare_with_python_fabrik(target_x: float, target_y: float, target_z: float):
    """
    Compare C++ FABRIK results with Python FABRIK implementation
    
    Args:
        target_x, target_y, target_z: Target position components
    """
    
    print(f"\n=== FABRIK Implementation Comparison ===")
    print(f"Target: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})")
    
    try:
        # Test C++ implementation
        print(f"\n--- C++ Implementation ---")
        cpp_result = test_fabrik_solver(target_x, target_y, target_z, debug=False)
        cpp_final_joints, cpp_iterations, cpp_distance, cpp_converged, cpp_time, cpp_success, _, _, _ = cpp_result
        
        if cpp_success:
            print(f"✓ C++ FABRIK: {cpp_iterations} iterations, distance {cpp_distance:.3f}mm, time {cpp_time:.1f}ms")
            print(f"  End-effector: ({cpp_final_joints[-1][0]:.3f}, {cpp_final_joints[-1][1]:.3f}, {cpp_final_joints[-1][2]:.3f})")
            print(f"  Converged: {cpp_converged}")
        else:
            print(f"✗ C++ FABRIK failed")
        
        # Test Python implementation (if available)
        print(f"\n--- Python Implementation Comparison ---")
        try:
            # Import your Python FABRIK if available
            import sys
            sys.path.append('.')
            from fabrik import FABRIKSolver
            
            python_solver = FABRIKSolver()
            python_result = python_solver.solve([target_x, target_y, target_z])
            
            py_final_joints = python_result['final_joints']
            py_iterations = python_result['iterations']
            py_distance = python_result['remaining_tolerance']
            py_converged = python_result['converged']
            
            print(f"✓ Python FABRIK: {py_iterations} iterations, distance {py_distance:.3f}mm")
            print(f"  End-effector: ({py_final_joints[-1][0]:.3f}, {py_final_joints[-1][1]:.3f}, {py_final_joints[-1][2]:.3f})")
            print(f"  Converged: {py_converged}")
            
            # Compare results
            if cpp_success:
                cpp_end = np.array(cpp_final_joints[-1])
                py_end = np.array(py_final_joints[-1])
                end_effector_diff = np.linalg.norm(cpp_end - py_end)
                
                print(f"\n--- Comparison ---")
                print(f"End-effector difference: {end_effector_diff:.3f}mm")
                print(f"Iteration difference: {abs(cpp_iterations - py_iterations)}")
                print(f"Distance difference: {abs(cpp_distance - py_distance):.3f}mm")
                
                if end_effector_diff < 1.0:  # Within 1mm
                    print(f"✓ Results are very similar!")
                elif end_effector_diff < 10.0:  # Within 1cm
                    print(f"✓ Results are reasonably similar")
                else:
                    print(f"⚠️  Results differ significantly")
            
        except ImportError:
            print(f"Python FABRIK not available for comparison")
        except Exception as e:
            print(f"Python FABRIK comparison failed: {e}")
    
    except Exception as e:
        print(f"Comparison failed: {e}")


# Example usage and testing functions
if __name__ == "__main__":
    print("FABRIK Python Wrapper Test")
    print("=" * 50)
    
    # Test cases matching your Python examples
    test_targets = [
        [80, 30, 400],    # Your main example
        [0, 0, 1000],     # Straight up
        [100, 50, 300],   # Moderate challenge
    ]
    
    for target in test_targets:
        print(f"\nTesting target: {target}")
        try:
            # Test complete solver with visualization
            result = test_fabrik_solver(target[0], target[1], target[2], debug=True)
            
            # Compare with Python implementation
            compare_with_python_fabrik(target[0], target[1], target[2])
            
        except Exception as e:
            print(f"Test failed: {e}")
    
    print(f"\n" + "=" * 50)
    print("FABRIK Wrapper Tests Completed")