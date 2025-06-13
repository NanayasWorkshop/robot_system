#!/usr/bin/env python3
"""
STAR Interface Bridge Diagnostic
Extracts exact STAR output formats for collision bridge design
"""

import numpy as np
import torch

try:
    from star.pytorch.star import STAR
    STAR_AVAILABLE = True
except ImportError:
    print("❌ STAR not available")
    STAR_AVAILABLE = False


def diagnostic_star_interface():
    """Extract and print exact STAR interface format"""
    print("\n" + "="*60)
    print("STAR INTERFACE DIAGNOSTIC")
    print("="*60)
    
    if not STAR_AVAILABLE:
        print("❌ STAR model not available")
        return None
    
    # Initialize STAR model
    print("Initializing STAR model...")
    try:
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"Using device: {device}")
        
        model = STAR(gender='neutral')
        model.to(device)
        model.eval()
        print("✅ STAR model initialized successfully")
        
    except Exception as e:
        print(f"❌ Failed to initialize STAR: {e}")
        return None
    
    return model


def diagnostic_neutral_pose(model):
    """Extract neutral pose data format"""
    print("\n" + "="*60)
    print("NEUTRAL POSE DIAGNOSTIC")
    print("="*60)
    
    # Get device from model (try different methods)
    try:
        device = next(model.parameters()).device
    except StopIteration:
        # Fallback: check if model has device attribute or use cuda/cpu
        if hasattr(model, 'device'):
            device = model.device
        elif torch.cuda.is_available():
            device = torch.device('cuda')
        else:
            device = torch.device('cpu')
    
    print(f"Using device: {device}")
    batch_size = 1
    
    pose_params = torch.zeros(batch_size, 72, device=device)
    shape_params = torch.zeros(batch_size, 10, device=device)
    trans = torch.zeros(batch_size, 3, device=device)
    
    print(f"Input Parameters:")
    print(f"  Pose params shape: {pose_params.shape}")
    print(f"  Shape params shape: {shape_params.shape}")
    print(f"  Translation shape: {trans.shape}")
    
    # Forward pass
    with torch.no_grad():
        try:
            result = model(pose_params, shape_params, trans)
            
            if isinstance(result, tuple):
                vertices, joints = result
                print(f"\nSTAR Output (tuple):")
                print(f"  Vertices shape: {vertices.shape}")
                print(f"  Joints shape: {joints.shape}")
            else:
                vertices = result
                joints = None
                print(f"\nSTAR Output (vertices only):")
                print(f"  Vertices shape: {vertices.shape}")
                
                # Try to get joints via J_regressor
                if hasattr(model, 'J_regressor'):
                    joints = torch.matmul(model.J_regressor, vertices)
                    print(f"  Joints from J_regressor: {joints.shape}")
            
            # Convert to numpy
            vertices_np = vertices[0].cpu().numpy()
            joints_np = joints[0].cpu().numpy() if joints is not None else None
            
            return vertices_np, joints_np
            
        except Exception as e:
            print(f"❌ STAR forward pass failed: {e}")
            return None, None


def diagnostic_mesh_vertices(vertices):
    """Analyze mesh vertex format"""
    print("\n" + "="*60)
    print("MESH VERTICES DIAGNOSTIC")
    print("="*60)
    
    if vertices is None:
        print("❌ No vertices data")
        return
    
    print(f"Vertex Data Format:")
    print(f"  Shape: {vertices.shape}")
    print(f"  Type: {type(vertices)}")
    print(f"  Dtype: {vertices.dtype}")
    
    print(f"\nVertex Statistics:")
    print(f"  Total vertices: {len(vertices)}")
    print(f"  Min coordinates: [{vertices[:, 0].min():.3f}, {vertices[:, 1].min():.3f}, {vertices[:, 2].min():.3f}]")
    print(f"  Max coordinates: [{vertices[:, 0].max():.3f}, {vertices[:, 1].max():.3f}, {vertices[:, 2].max():.3f}]")
    print(f"  Mean coordinates: [{vertices[:, 0].mean():.3f}, {vertices[:, 1].mean():.3f}, {vertices[:, 2].mean():.3f}]")
    
    # Sample vertices
    print(f"\nSample Vertices:")
    sample_indices = [0, len(vertices)//4, len(vertices)//2, 3*len(vertices)//4, len(vertices)-1]
    for i in sample_indices:
        print(f"  Vertex[{i:4d}]: ({vertices[i, 0]:8.3f}, {vertices[i, 1]:8.3f}, {vertices[i, 2]:8.3f})")


def diagnostic_joint_positions(joints):
    """Analyze joint position format"""
    print("\n" + "="*60)
    print("JOINT POSITIONS DIAGNOSTIC")
    print("="*60)
    
    if joints is None:
        print("❌ No joint data")
        return
    
    print(f"Joint Data Format:")
    print(f"  Shape: {joints.shape}")
    print(f"  Type: {type(joints)}")
    print(f"  Dtype: {joints.dtype}")
    
    # STAR joint names (standard 24 joints)
    joint_names = [
        'pelvis', 'left_hip', 'right_hip', 'spine1', 'left_knee', 'right_knee',
        'spine2', 'left_ankle', 'right_ankle', 'spine3', 'left_foot', 'right_foot',
        'neck', 'left_collar', 'right_collar', 'head', 'left_shoulder', 'right_shoulder',
        'left_elbow', 'right_elbow', 'left_wrist', 'right_wrist', 'left_hand', 'right_hand'
    ]
    
    print(f"\nJoint Statistics:")
    print(f"  Total joints: {len(joints)}")
    print(f"  Expected joints: 24")
    if len(joints) != 24:
        print(f"  ⚠️  WARNING: Joint count mismatch!")
    
    print(f"  Min coordinates: [{joints[:, 0].min():.3f}, {joints[:, 1].min():.3f}, {joints[:, 2].min():.3f}]")
    print(f"  Max coordinates: [{joints[:, 0].max():.3f}, {joints[:, 1].max():.3f}, {joints[:, 2].max():.3f}]")
    print(f"  Mean coordinates: [{joints[:, 0].mean():.3f}, {joints[:, 1].mean():.3f}, {joints[:, 2].mean():.3f}]")
    
    print(f"\nJoint Positions:")
    for i, joint_pos in enumerate(joints):
        joint_name = joint_names[i] if i < len(joint_names) else f"joint_{i}"
        print(f"  Joint[{i:2d}] {joint_name:15s}: ({joint_pos[0]:8.3f}, {joint_pos[1]:8.3f}, {joint_pos[2]:8.3f})")


def diagnostic_pose_updates(model):
    """Test pose parameter updates"""
    print("\n" + "="*60)
    print("POSE UPDATE DIAGNOSTIC")
    print("="*60)
    
    # Get device safely
    try:
        device = next(model.parameters()).device
    except StopIteration:
        if hasattr(model, 'device'):
            device = model.device
        elif torch.cuda.is_available():
            device = torch.device('cuda')
        else:
            device = torch.device('cpu')
    
    batch_size = 1
    
    # Test different poses
    test_poses = [
        ("neutral", torch.zeros(72)),
        ("spine_bend", torch.zeros(72)),
        ("arm_raise", torch.zeros(72)),
    ]
    
    # Modify poses
    test_poses[1][1][6:9] = torch.tensor([0.5, 0, 0])  # Bend spine forward
    test_poses[2][1][48:51] = torch.tensor([0, 0, 1.57])  # Raise left arm
    
    print("Testing pose parameter updates:")
    
    for pose_name, pose_params in test_poses:
        print(f"\n  Testing {pose_name} pose...")
        
        pose_batch = pose_params.unsqueeze(0).to(device)
        shape_params = torch.zeros(batch_size, 10, device=device)
        trans = torch.zeros(batch_size, 3, device=device)
        
        with torch.no_grad():
            try:
                result = model(pose_batch, shape_params, trans)
                
                if isinstance(result, tuple):
                    vertices, joints = result
                else:
                    vertices = result
                    if hasattr(model, 'J_regressor'):
                        joints = torch.matmul(model.J_regressor, vertices)
                    else:
                        joints = None
                
                vertices_np = vertices[0].cpu().numpy()
                joints_np = joints[0].cpu().numpy() if joints is not None else None
                
                print(f"    ✅ Success: {len(vertices_np)} vertices, {len(joints_np) if joints_np is not None else 0} joints")
                
                if joints_np is not None:
                    # Show key joint changes
                    key_joints = [0, 3, 6, 9, 15, 16, 17]  # pelvis, spine joints, head, shoulders
                    for joint_idx in key_joints:
                        if joint_idx < len(joints_np):
                            joint_pos = joints_np[joint_idx]
                            print(f"      Joint[{joint_idx}]: ({joint_pos[0]:6.3f}, {joint_pos[1]:6.3f}, {joint_pos[2]:6.3f})")
                
            except Exception as e:
                print(f"    ❌ Failed: {e}")


def diagnostic_bridge_requirements():
    """Print what the STAR-collision bridge needs"""
    print("\n" + "="*60)
    print("STAR-COLLISION BRIDGE REQUIREMENTS")
    print("="*60)
    
    print("\nINPUT (from STAR):")
    print("  - vertices: numpy array (N, 3) - mesh vertices in world space")
    print("  - joints: numpy array (24, 3) - bone positions for 24 STAR joints")
    print("  - coordinate system: STAR default (Y-up, person lying on back)")
    
    print("\nPROCESSING STEPS:")
    print("  1. Transform STAR joint positions to collision engine coordinate system")
    print("  2. Update LayerManager with new bone positions")
    print("  3. Update collision primitives (Layer 3, 2, 1 hierarchies)")
    print("  4. Handle mesh vertex updates if needed")
    
    print("\nOUTPUT (for Collision Engine):")
    print("  - Updated bone_positions: vector<Eigen::Vector3d> (24 joints)")
    print("  - Coordinate system: Same as collision detection expects")
    print("  - Success/failure status")
    
    print("\nBRIDGE FUNCTION SIGNATURE:")
    print("  bool update_collision_from_star(")
    print("    CollisionDetectionEngine& engine,")
    print("    const std::vector<Eigen::Vector3d>& star_bone_positions)")
    
    print("\nCOORDINATE SYSTEM NOTES:")
    print("  - STAR: Y-up, person lying on back")
    print("  - Robot: Need to verify coordinate system")
    print("  - May need transformation matrix between systems")


def main():
    """Run complete STAR diagnostic"""
    print("STAR → COLLISION BRIDGE DIAGNOSTIC")
    print("Extracting exact data formats for bridge design")
    
    # Step 1: Initialize STAR
    model = diagnostic_star_interface()
    if model is None:
        return
    
    # Step 2: Get neutral pose data
    vertices, joints = diagnostic_neutral_pose(model)
    if vertices is None:
        return
    
    # Step 3: Analyze mesh vertices
    diagnostic_mesh_vertices(vertices)
    
    # Step 4: Analyze joint positions
    diagnostic_joint_positions(joints)
    
    # Step 5: Test pose updates
    diagnostic_pose_updates(model)
    
    # Step 6: Bridge requirements
    diagnostic_bridge_requirements()
    
    print("\n" + "="*60)
    print("STAR DIAGNOSTIC COMPLETE")
    print("Use this output to design the C++ STAR-collision bridge")
    print("="*60)


if __name__ == "__main__":
    main()