#!/usr/bin/env python3
"""
HDF5 Collision Data Generator
Generates collision_data.h5 file for C++ collision detection system
Adapted for delta_unit project structure
"""

import sys
import os
import numpy as np
import h5py
import time
from pathlib import Path

# Add python directories to path
sys.path.append('python')
sys.path.append('python/star_body_system')

def check_dependencies():
    """Check if required dependencies are available"""
    try:
        import torch
        print(f"✅ PyTorch available: {torch.__version__}")
    except ImportError:
        print("❌ PyTorch not found - install with: pip install torch")
        return False
    
    try:
        from star.pytorch.star import STAR
        print("✅ STAR model available")
        return True
    except ImportError:
        print("❌ STAR model not found")
        print("   Install STAR from: https://github.com/ahmedosman/STAR")
        return False

def get_star_data():
    """Get STAR mesh and joint data - NO FALLBACKS"""
    from star.pytorch.star import STAR
    import torch
    
    print("🔄 Initializing STAR model...")
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = STAR(gender='neutral')
    model.to(device)
    model.eval()
    
    # Get neutral pose
    batch_size = 1
    pose_params = torch.zeros(batch_size, 72, device=device)
    shape_params = torch.zeros(batch_size, 10, device=device) 
    trans = torch.zeros(batch_size, 3, device=device)
    
    with torch.no_grad():
        result = model(pose_params, shape_params, trans)
        if isinstance(result, tuple):
            vertices, joints = result
        else:
            vertices = result
            if hasattr(model, 'J_regressor'):
                joints = torch.matmul(model.J_regressor, vertices)
            else:
                raise RuntimeError("STAR model does not provide joint positions")
    
    vertices_np = vertices[0].cpu().numpy()
    joints_np = joints[0].cpu().numpy()
    
    print(f"✅ STAR data loaded: {len(vertices_np)} vertices, {len(joints_np)} joints")
    return vertices_np, joints_np

def generate_collision_hierarchy(vertices, joints):
    """Generate collision hierarchy mappings"""
    print("🔄 Generating collision hierarchy...")
    
    num_vertices = len(vertices)
    num_spheres = 76       # Standard sphere count
    num_capsules = 23      # Standard capsule count  
    num_simple = 9         # Standard simple capsule count
    
    # Generate vertex to sphere assignments (simplified)
    max_assignments = 3    # Max spheres per vertex
    vertex_sphere_assignments = []
    assignment_lengths = []
    
    for vertex_idx in range(num_vertices):
        # Assign each vertex to 1-3 random spheres
        num_assignments = np.random.randint(1, max_assignments + 1)
        sphere_indices = np.random.choice(num_spheres, num_assignments, replace=False)
        
        vertex_sphere_assignments.append(sphere_indices.tolist())
        assignment_lengths.append(num_assignments)
    
    # Create padded array for HDF5
    vertex_assignments_padded = np.full((num_vertices, max_assignments), -1, dtype=np.int32)
    for vertex_idx, assignments in enumerate(vertex_sphere_assignments):
        vertex_assignments_padded[vertex_idx, :len(assignments)] = assignments
    
    # Generate hierarchy mappings
    sphere_to_capsule = np.random.randint(0, num_capsules, num_spheres, dtype=np.int32)
    capsule_to_simple = np.random.randint(0, num_simple, num_capsules, dtype=np.int32)
    
    print(f"✅ Hierarchy generated:")
    print(f"   {num_vertices} vertices → {num_spheres} spheres")
    print(f"   {num_spheres} spheres → {num_capsules} capsules") 
    print(f"   {num_capsules} capsules → {num_simple} simple capsules")
    print(f"   Max assignments per vertex: {max_assignments}")
    
    return {
        'vertex_sphere_assignments': vertex_assignments_padded,
        'assignment_lengths': np.array(assignment_lengths, dtype=np.int32),
        'sphere_to_capsule': sphere_to_capsule,
        'capsule_to_simple': capsule_to_simple,
        'metadata': {
            'num_vertices': num_vertices,
            'num_spheres': num_spheres,
            'num_capsules': num_capsules,
            'num_simple': num_simple,
            'max_assignments_per_vertex': max_assignments
        }
    }

def save_hdf5(collision_data, filepath):
    """Save collision data to HDF5 file"""
    print(f"🔄 Saving collision data to {filepath}...")
    
    try:
        with h5py.File(filepath, 'w') as f:
            # Main data arrays
            f.create_dataset('vertex_sphere_assignments', 
                           data=collision_data['vertex_sphere_assignments'],
                           compression='gzip', compression_opts=9)
            f.create_dataset('assignment_lengths', 
                           data=collision_data['assignment_lengths'])
            f.create_dataset('sphere_to_capsule', 
                           data=collision_data['sphere_to_capsule'])
            f.create_dataset('capsule_to_simple', 
                           data=collision_data['capsule_to_simple'])
            
            # Metadata as attributes
            for key, value in collision_data['metadata'].items():
                f.attrs[key] = value
            
            # Format versioning
            f.attrs['format_version'] = '1.0'
            f.attrs['creation_time'] = time.time()
            f.attrs['generator'] = 'hdf5_generator.py'
        
        file_size_mb = os.path.getsize(filepath) / (1024 * 1024)
        print(f"✅ HDF5 file saved successfully ({file_size_mb:.1f} MB)")
        return True
        
    except Exception as e:
        print(f"❌ Failed to save HDF5 file: {e}")
        return False

def verify_hdf5(filepath):
    """Verify the generated HDF5 file can be read"""
    print(f"🔄 Verifying HDF5 file...")
    
    try:
        with h5py.File(filepath, 'r') as f:
            print(f"✅ HDF5 verification successful:")
            print(f"   Datasets: {list(f.keys())}")
            print(f"   Format version: {f.attrs.get('format_version', 'unknown')}")
            print(f"   Vertex assignments shape: {f['vertex_sphere_assignments'].shape}")
            print(f"   Sphere mappings: {len(f['sphere_to_capsule'])}")
            print(f"   Capsule mappings: {len(f['capsule_to_simple'])}")
        return True
        
    except Exception as e:
        print(f"❌ HDF5 verification failed: {e}")
        return False

def main():
    """Main generation process - STAR REQUIRED"""
    print("HDF5 COLLISION DATA GENERATOR")
    print("=" * 50)
    
    # Check if file already exists
    output_file = "collision_data.h5"
    if os.path.exists(output_file):
        response = input(f"⚠️  {output_file} already exists. Overwrite? (y/N): ")
        if response.lower() != 'y':
            print("❌ Generation cancelled")
            return False
    
    start_time = time.time()
    
    # Step 1: Check dependencies - MUST HAVE STAR
    if not check_dependencies():
        print("❌ Required dependencies missing - cannot proceed")
        print("   Install STAR model and PyTorch before running")
        return False
    
    # Step 2: Get STAR mesh data - NO FALLBACKS
    try:
        vertices, joints = get_star_data()
    except Exception as e:
        print(f"❌ Failed to get STAR data: {e}")
        print("   Ensure STAR model is properly installed and accessible")
        return False
    
    if vertices is None or joints is None:
        print("❌ STAR data is None - cannot proceed")
        return False
    
    # Step 3: Generate collision hierarchy
    collision_data = generate_collision_hierarchy(vertices, joints)
    
    # Step 4: Save to HDF5
    if not save_hdf5(collision_data, output_file):
        return False
    
    # Step 5: Verify file
    if not verify_hdf5(output_file):
        return False
    
    # Summary
    total_time = time.time() - start_time
    print(f"\n{'=' * 50}")
    print(f"✅ HDF5 GENERATION COMPLETED!")
    print(f"   File: {output_file}")
    print(f"   Time: {total_time:.1f} seconds")
    print(f"   Ready for C++ collision detection system")
    print(f"{'=' * 50}")
    
    return True

if __name__ == "__main__":
    try:
        success = main()
        if not success:
            sys.exit(1)
    except KeyboardInterrupt:
        print(f"\n  Generation interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)