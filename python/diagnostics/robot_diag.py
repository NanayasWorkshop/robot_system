#!/usr/bin/env python3
"""
FABRIK → Capsule Bridge Diagnostic
Extracts exact data formats for bridge design
"""

import sys
sys.path.append('python/visualization')

from capsule_wrapper import create_capsule_chain
import delta_robot_cpp
import numpy as np


def diagnostic_fabrik_output():
    """Extract and print exact FABRIK output format"""
    print("\n" + "="*60)
    print("FABRIK OUTPUT DIAGNOSTIC")
    print("="*60)
    
    # Test case
    target = [80.0, 30.0, 400.0]
    print(f"Input Target: [{target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f}]")
    
    # FABRIK solve
    fabrik_result = delta_robot_cpp.fabrik_solve_with_refinement(target)
    
    print(f"\nFABRIK Result Structure:")
    print(f"  Type: {type(fabrik_result)}")
    print(f"  Length: {len(fabrik_result)}")
    print(f"  Success: {fabrik_result[8]}")
    if not fabrik_result[8]:
        print(f"  Error: {fabrik_result[9]}")
        return None
    
    # Extract joint positions
    joints = fabrik_result[0]
    print(f"\nJoint Positions ({len(joints)} joints):")
    for i, joint in enumerate(joints):
        print(f"  Joint[{i}]: ({joint[0]:8.3f}, {joint[1]:8.3f}, {joint[2]:8.3f})")
    
    print(f"\nFABRIK Metadata:")
    print(f"  Final distance to target: {fabrik_result[4]:.6f}")
    print(f"  Converged: {fabrik_result[5]}")
    print(f"  Calculation time: {fabrik_result[7]:.3f}ms")
    print(f"  Iterations used: {fabrik_result[2]}")
    
    return joints


def diagnostic_s_points_extraction(joints):
    """Extract and print S-points derivation process"""
    print("\n" + "="*60)
    print("S-POINTS EXTRACTION DIAGNOSTIC")
    print("="*60)
    
    print(f"Input: {len(joints)} joint positions")
    num_segments = len(joints) - 1
    print(f"Processing {num_segments} segments...")
    
    s_points = []
    
    for segment_idx in range(num_segments):
        print(f"\nSegment {segment_idx}:")
        try:
            result = delta_robot_cpp.calculate_segment_essential_from_joints(joints, segment_idx)
            prismatic_length, calc_time, success, error_msg, calc_seg_pos, calc_dir = result
            
            print(f"  Prismatic length: {prismatic_length:.3f}")
            print(f"  Calculation time: {calc_time:.3f}ms")
            print(f"  Success: {success}")
            
            if success and calc_seg_pos is not None:
                s_point = [float(calc_seg_pos[0]), float(calc_seg_pos[1]), float(calc_seg_pos[2])]
                print(f"  S-point: ({s_point[0]:8.3f}, {s_point[1]:8.3f}, {s_point[2]:8.3f})")
                
                # Check for duplicates
                is_duplicate = False
                if len(s_points) > 0:
                    distance = np.linalg.norm(np.array(s_points[-1]) - np.array(s_point))
                    print(f"  Distance to previous: {distance:.6f}")
                    if distance <= 1e-6:
                        print(f"  → Skipped (duplicate)")
                        is_duplicate = True
                
                if not is_duplicate:
                    s_points.append(s_point)
                    print(f"  → Added as S-point[{len(s_points)-1}]")
            else:
                print(f"  → Failed: {error_msg}")
                
        except Exception as e:
            print(f"  → Exception: {e}")
    
    print(f"\nFinal S-points ({len(s_points)} points):")
    for i, point in enumerate(s_points):
        print(f"  S[{i}]: ({point[0]:8.3f}, {point[1]:8.3f}, {point[2]:8.3f})")
    
    return s_points


def diagnostic_capsule_creation(s_points):
    """Extract and print capsule creation output format"""
    print("\n" + "="*60)
    print("CAPSULE CREATION DIAGNOSTIC")
    print("="*60)
    
    if len(s_points) < 2:
        print(f"❌ Insufficient S-points: {len(s_points)}")
        return None
    
    robot_radius = delta_robot_cpp.ROBOT_RADIUS
    print(f"Robot radius: {robot_radius:.1f}mm")
    print(f"Input S-points: {len(s_points)}")
    
    # Create capsules
    capsules, total_length, calc_time, success, error_msg = create_capsule_chain(
        s_points, robot_radius, debug=False
    )
    
    print(f"\nCapsule Creation Result:")
    print(f"  Success: {success}")
    print(f"  Calculation time: {calc_time:.3f}ms")
    print(f"  Total length: {total_length:.3f}mm")
    
    if not success:
        print(f"  Error: {error_msg}")
        return None
    
    print(f"\nGenerated Capsules ({len(capsules)}):")
    for i, capsule in enumerate(capsules):
        start = capsule['start_point']
        end = capsule['end_point']
        radius = capsule['radius']
        length = capsule['length']
        print(f"  Capsule[{i}]:")
        print(f"    Start:  ({start[0]:8.3f}, {start[1]:8.3f}, {start[2]:8.3f})")
        print(f"    End:    ({end[0]:8.3f}, {end[1]:8.3f}, {end[2]:8.3f})")
        print(f"    Radius: {radius:8.3f}")
        print(f"    Length: {length:8.3f}")
    
    return capsules


def diagnostic_bridge_requirements():
    """Print what the bridge needs to implement"""
    print("\n" + "="*60)
    print("BRIDGE REQUIREMENTS SUMMARY")
    print("="*60)
    
    print("\nINPUT (from FABRIK):")
    print("  - fabrik_result[0]: List of joint positions [(x,y,z), ...]")
    print("  - fabrik_result[8]: success boolean")
    print("  - fabrik_result[9]: error message string")
    
    print("\nPROCESSING STEPS:")
    print("  1. Extract S-points from joints using calculate_segment_essential_from_joints()")
    print("  2. Filter duplicate consecutive S-points (distance < 1e-6)")
    print("  3. Convert S-points to capsule chain using robot_radius")
    
    print("\nOUTPUT (for Collision):")
    print("  - Vector of CapsuleData structs")
    print("  - Each capsule: start_point, end_point, radius, length")
    print("  - Coordinate system: Same as FABRIK joints")
    
    print("\nBRIDGE FUNCTION SIGNATURE:")
    print("  std::vector<CapsuleData> convert_fabrik_to_capsules(")
    print("    const FabrikSolverResult& fabrik_result,")
    print("    double robot_radius = ROBOT_RADIUS)")


def main():
    """Run complete diagnostic"""
    print("FABRIK → CAPSULE BRIDGE DIAGNOSTIC")
    print("Extracting exact data formats for bridge design")
    
    # Step 1: FABRIK output
    joints = diagnostic_fabrik_output()
    if joints is None:
        return
    
    # Step 2: S-points extraction
    s_points = diagnostic_s_points_extraction(joints)
    if not s_points:
        return
    
    # Step 3: Capsule creation
    capsules = diagnostic_capsule_creation(s_points)
    if capsules is None:
        return
    
    # Step 4: Bridge requirements
    diagnostic_bridge_requirements()
    
    print("\n" + "="*60)
    print("DIAGNOSTIC COMPLETE")
    print("Use this output to design the C++ bridge")
    print("="*60)


if __name__ == "__main__":
    main()