#!/usr/bin/env python3
"""
Capsule Creation Block Test - Minimal Pipeline Version
Target → FABRIK → S-points → Real Capsules → Visualization
"""

import sys
sys.path.append('python/visualization')

from capsule_wrapper import create_capsule_chain, visualize_capsule_chain
import delta_robot_cpp
import numpy as np


def extract_s_points_from_joints(joints):
    """Extract unique S-points from FABRIK joint positions"""
    s_points = []
    num_segments = len(joints) - 1
    
    for segment_idx in range(num_segments):
        try:
            result = delta_robot_cpp.calculate_segment_essential_from_joints(joints, segment_idx)
            prismatic_length, calc_time, success, error_msg, calc_seg_pos, calc_dir = result
            
            if success and calc_seg_pos is not None:
                s_point = [float(calc_seg_pos[0]), float(calc_seg_pos[1]), float(calc_seg_pos[2])]
                
                # Skip duplicate consecutive points
                if len(s_points) == 0 or np.linalg.norm(np.array(s_points[-1]) - np.array(s_point)) > 1e-6:
                    s_points.append(s_point)
                    
        except Exception as e:
            print(f"Segment {segment_idx} failed: {e}")
    
    return s_points


def test_capsule_pipeline():
    """Complete pipeline: Target → FABRIK → S-points → Capsules → Visualization"""
    
    print("CAPSULE PIPELINE TEST")
    print("=" * 50)
    
    # Step 1: FABRIK solution
    target = [80.0, 30.0, 400.0]
    print(f"Target: ({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})")
    
    fabrik_result = delta_robot_cpp.fabrik_solve_with_refinement(target)
    if not fabrik_result[8]:  # solving_successful
        print(f"❌ FABRIK failed: {fabrik_result[9]}")
        return
    
    joints = fabrik_result[0]
    print(f"✓ FABRIK solved with {len(joints)} joints")
    
    # Step 2: Extract S-points
    s_points = extract_s_points_from_joints(joints)
    if len(s_points) < 2:
        print(f"❌ Insufficient S-points: {len(s_points)}")
        return
    
    print(f"✓ Extracted {len(s_points)} S-points")
    
    # DEBUG: Print all S-points
    print("\nDEBUG - S-points:")
    for i, sp in enumerate(s_points):
        print(f"  S{i+1}: ({sp[0]:.3f}, {sp[1]:.3f}, {sp[2]:.3f})")
    
    # DEBUG: Check distances between consecutive S-points
    print("\nDEBUG - Distances between consecutive S-points:")
    for i in range(len(s_points) - 1):
        dist = np.linalg.norm(np.array(s_points[i+1]) - np.array(s_points[i]))
        print(f"  S{i+1} → S{i+2}: {dist:.6f}mm")
        if dist < 1e-6:
            print(f"    ⚠️  DEGENERATE SEGMENT!")
    
    # DEBUG: What C++ sees after S0 injection
    print(f"\nDEBUG - After S0 injection at (0,0,0):")
    complete_s_points = [[0.0, 0.0, 0.0]] + s_points
    for i in range(len(complete_s_points) - 1):
        dist = np.linalg.norm(np.array(complete_s_points[i+1]) - np.array(complete_s_points[i]))
        print(f"  S{i} → S{i+1}: {dist:.6f}mm")
        if dist < 1e-6:
            print(f"    ⚠️  DEGENERATE SEGMENT!")
    
    # Step 3: Create capsules
    robot_radius = delta_robot_cpp.ROBOT_RADIUS
    capsules, total_length, calc_time, success, error_msg = create_capsule_chain(
        s_points, robot_radius, debug=True
    )
    
    if not success:
        print(f"❌ Capsule creation failed: {error_msg}")
        return
    
    print(f"✓ Created {len(capsules)} capsules")
    print(f"  Total length: {total_length:.1f}mm")
    print(f"  Calculation time: {calc_time:.3f}ms")
    
    # Step 4: Visualize real capsules
    print("✓ Visualizing real capsules...")
    visualize_capsule_chain(s_points, capsules, robot_radius)
    
    print("=" * 50)
    print("PIPELINE COMPLETED SUCCESSFULLY")


def main():
    """Run minimal test"""
    
    print("CAPSULE CREATION BLOCK - MINIMAL TEST")
    print("=" * 60)
    
    # Just run the basic pipeline
    test_capsule_pipeline()
    
    print("\n" + "=" * 60)
    print("TEST COMPLETED")


if __name__ == "__main__":
    main()