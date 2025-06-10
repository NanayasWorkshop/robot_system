import numpy as np

class FABRIKSolver:
    def __init__(self):
        # Initial joint positions
        self.initial_joints = np.array([
            [0, 0, 0],      # J0 - Base
            [0, 0, 73],     # J1
            [0, 0, 219],    # J2
            [0, 0, 365],    # J3
            [0, 0, 511],    # J4
            [0, 0, 657],    # J5
            [0, 0, 803],    # J6
            [0, 0, 949],    # J7
            [0, 0, 1022]    # J8 - End Effector
        ])
        
        # Calculate segment lengths
        self.segment_lengths = []
        for i in range(len(self.initial_joints) - 1):
            length = self.vector_distance(self.initial_joints[i], self.initial_joints[i + 1])
            self.segment_lengths.append(length)
        
        # Parameters
        self.cone_angle = 120.0  # degrees (total opening angle)
        self.max_iterations = 50
        self.tolerance = 0.01
    
    def vector_subtract(self, a, b):
        return np.array(a) - np.array(b)
    
    def vector_magnitude(self, v):
        return np.linalg.norm(v)
    
    def vector_distance(self, a, b):
        return self.vector_magnitude(self.vector_subtract(a, b))
    
    def normalize_vector(self, v):
        mag = self.vector_magnitude(v)
        if mag == 0:
            return np.array([0, 0, 0])
        return np.array(v) / mag
    
    def vector_dot(self, a, b):
        return np.dot(a, b)
    
    def angle_between_vectors(self, v1, v2):
        """Calculate angle between two vectors in degrees"""
        v1_norm = self.normalize_vector(v1)
        v2_norm = self.normalize_vector(v2)
        dot_product = np.clip(self.vector_dot(v1_norm, v2_norm), -1.0, 1.0)
        angle_rad = np.arccos(dot_product)
        return np.degrees(angle_rad)
    
    def is_inside_cone(self, direction, cone_axis, cone_half_angle):
        """Check if direction is within cone constraints"""
        angle = self.angle_between_vectors(direction, cone_axis)
        return angle <= cone_half_angle
    
    def project_onto_cone(self, direction, cone_axis, cone_half_angle):
        """Project direction onto cone surface if outside"""
        if self.is_inside_cone(direction, cone_axis, cone_half_angle):
            return direction
        
        # Project direction onto plane perpendicular to cone axis
        direction_norm = self.normalize_vector(direction)
        cone_axis_norm = self.normalize_vector(cone_axis)
        
        # Component along cone axis
        axis_component = self.vector_dot(direction_norm, cone_axis_norm) * cone_axis_norm
        
        # Component perpendicular to cone axis
        perp_component = direction_norm - axis_component
        perp_component_norm = self.normalize_vector(perp_component)
        
        # Calculate the corrected direction on cone surface
        cone_half_angle_rad = np.radians(cone_half_angle)
        corrected_direction = (np.cos(cone_half_angle_rad) * cone_axis_norm + 
                             np.sin(cone_half_angle_rad) * perp_component_norm)
        
        return corrected_direction
    
    def backward_pass(self, joints, target):
        """Perform backward iteration from end effector to base"""
        b_joints = joints.copy()
        
        # Step 1: Move end effector to target
        b_joints[-1] = np.array(target)
        
        # Step 2: Position remaining joints with cone constraints
        for i in range(len(b_joints) - 2, -1, -1):
            if i == len(b_joints) - 2:
                # First joint back (no cone constraint)
                direction = self.vector_subtract(joints[i], b_joints[i + 1])
                direction_norm = self.normalize_vector(direction)
                b_joints[i] = b_joints[i + 1] + direction_norm * self.segment_lengths[i]
            else:
                # Apply cone constraint
                # Cone apex is at the previously positioned joint
                cone_apex = b_joints[i + 1]
                # Cone axis points from joint two positions forward to apex
                cone_axis = self.vector_subtract(cone_apex, b_joints[i + 2])
                
                # Desired direction
                desired_direction = self.vector_subtract(joints[i], cone_apex)
                
                # Apply cone constraint
                cone_half_angle = self.cone_angle / 2.0
                corrected_direction = self.project_onto_cone(desired_direction, cone_axis, cone_half_angle)
                corrected_direction_norm = self.normalize_vector(corrected_direction)
                
                # Position joint at correct distance
                b_joints[i] = cone_apex + corrected_direction_norm * self.segment_lengths[i]
        
        return b_joints
    
    def forward_pass(self, backward_joints):
        """Perform forward iteration from base to end effector"""
        f_joints = backward_joints.copy()
        
        # Step 1: Fix base position
        f_joints[0] = self.initial_joints[0].copy()
        
        # Step 2: Position remaining joints with cone constraints
        for i in range(1, len(f_joints)):
            if i == 1:
                # First joint from base (no cone constraint)
                direction = self.vector_subtract(backward_joints[i], f_joints[i - 1])
                direction_norm = self.normalize_vector(direction)
                f_joints[i] = f_joints[i - 1] + direction_norm * self.segment_lengths[i - 1]
            else:
                # Apply cone constraint
                # Cone apex is at the previously positioned joint
                cone_apex = f_joints[i - 1]
                # Cone axis points from joint two positions back toward apex
                cone_axis = self.vector_subtract(cone_apex, f_joints[i - 2])
                
                # Desired direction
                desired_direction = self.vector_subtract(backward_joints[i], cone_apex)
                
                # Apply cone constraint
                cone_half_angle = self.cone_angle / 2.0
                corrected_direction = self.project_onto_cone(desired_direction, cone_axis, cone_half_angle)
                corrected_direction_norm = self.normalize_vector(corrected_direction)
                
                # Position joint at correct distance
                f_joints[i] = cone_apex + corrected_direction_norm * self.segment_lengths[i - 1]
        
        return f_joints
    
    def solve(self, target):
        """Main FABRIK solving function"""
        current_joints = self.initial_joints.copy()
        
        for iteration in range(self.max_iterations):
            # Check if we're close enough to target
            end_effector_pos = current_joints[-1]
            distance_to_target = self.vector_distance(end_effector_pos, target)
            
            if distance_to_target <= self.tolerance:
                return {
                    'final_joints': current_joints,
                    'iterations': iteration + 1,
                    'remaining_tolerance': distance_to_target,
                    'converged': True
                }
            
            # Backward pass
            backward_joints = self.backward_pass(current_joints, target)
            
            # Forward pass
            current_joints = self.forward_pass(backward_joints)
        
        # Max iterations reached
        final_distance = self.vector_distance(current_joints[-1], target)
        return {
            'final_joints': current_joints,
            'iterations': self.max_iterations,
            'remaining_tolerance': final_distance,
            'converged': False
        }


# Example usage
if __name__ == "__main__":
    # Create solver
    solver = FABRIKSolver()
    
    # Define target position
    target = [80, 30, 400]
    
    print(f"FABRIK Solver")
    print(f"Target: {target}")
    print(f"Initial end effector: {solver.initial_joints[-1]}")
    print("-" * 50)
    
    # Solve
    result = solver.solve(target)
    
    # Print results
    print(f"Final joint positions:")
    for i, joint in enumerate(result['final_joints']):
        print(f"  J{i}: [{joint[0]:8.3f}, {joint[1]:8.3f}, {joint[2]:8.3f}]")
    
    print(f"\nIterations used: {result['iterations']}")
    print(f"Remaining tolerance: {result['remaining_tolerance']:.6f}")
    print(f"Converged: {'Yes' if result['converged'] else 'No (max iterations reached)'}")
    print(f"Final end effector: {result['final_joints'][-1]}")