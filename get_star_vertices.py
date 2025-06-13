"""
STAR Vertex Extractor
Extracts T-pose vertices from STAR model and saves to binary file for C++ consumption
"""

import sys
import os
import numpy as np
import struct
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
        print(f"   CUDA available: {torch.cuda.is_available()}")
        if torch.cuda.is_available():
            print(f"   GPU: {torch.cuda.get_device_name(0)}")
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

def extract_star_t_pose_vertices(gender='neutral'):
    """
    Extract T-pose vertices from STAR model
    
    Args:
        gender: 'neutral', 'male', or 'female'
        
    Returns:
        numpy array: vertices (6890, 3) in meters
    """
    import torch
    from star.pytorch.star import STAR
    
    print(f"🔄 Initializing STAR model (gender: {gender})...")
    
    # Use GPU for consistency and speed
    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    print(f"   Using device: {device}")
    
    model = STAR(gender=gender)
    model = model.to(device)
    model.eval()
    
    # Create T-pose parameters (all zeros = neutral/T-pose)
    batch_size = 1
    pose_params = torch.zeros(batch_size, 72, device=device, dtype=torch.float32)  # T-pose
    shape_params = torch.zeros(batch_size, 10, device=device, dtype=torch.float32) # Neutral shape
    trans = torch.zeros(batch_size, 3, device=device, dtype=torch.float32)         # No translation
    
    print("🔄 Generating T-pose mesh...")
    with torch.no_grad():
        result = model(pose_params, shape_params, trans)
        
        if isinstance(result, tuple):
            vertices, joints = result
        else:
            vertices = result
            joints = None
    
    # Convert to numpy and extract first batch
    vertices_np = vertices[0].cpu().numpy().astype(np.float64)
    
    print(f"✅ Generated {len(vertices_np)} vertices")
    print(f"   Vertex shape: {vertices_np.shape}")
    print(f"   Data type: {vertices_np.dtype}")
    print(f"   Bounds: X[{vertices_np[:, 0].min():.3f}, {vertices_np[:, 0].max():.3f}] "
          f"Y[{vertices_np[:, 1].min():.3f}, {vertices_np[:, 1].max():.3f}] "
          f"Z[{vertices_np[:, 2].min():.3f}, {vertices_np[:, 2].max():.3f}]")
    
    # Validate vertex count
    if len(vertices_np) != 6890:
        print(f"⚠️  WARNING: Expected 6890 vertices, got {len(vertices_np)}")
        print("   This might cause issues with collision_data.h5")
    
    return vertices_np

def save_vertices_binary(vertices, filepath):
    """
    Save vertices to binary file for fast C++ loading
    
    Format:
    - uint32_t: vertex_count
    - double[vertex_count * 3]: vertex data (x, y, z, x, y, z, ...)
    
    Args:
        vertices: numpy array (N, 3)
        filepath: output file path
    """
    print(f"🔄 Saving vertices to binary file: {filepath}")
    
    vertex_count = len(vertices)
    
    try:
        with open(filepath, 'wb') as f:
            # Write header: vertex count
            f.write(struct.pack('<I', vertex_count))  # Little-endian uint32
            
            # Write vertex data: flatten to (x,y,z,x,y,z,...)
            vertex_data = vertices.flatten()
            for value in vertex_data:
                f.write(struct.pack('<d', value))  # Little-endian double
        
        file_size_mb = os.path.getsize(filepath) / (1024 * 1024)
        print(f"✅ Binary file saved successfully ({file_size_mb:.1f} MB)")
        print(f"   Vertex count: {vertex_count}")
        print(f"   Data size: {len(vertex_data)} doubles")
        return True
        
    except Exception as e:
        print(f"❌ Failed to save binary file: {e}")
        return False

def verify_binary_file(filepath):
    """Verify the binary file can be read correctly"""
    print(f"🔄 Verifying binary file: {filepath}")
    
    try:
        with open(filepath, 'rb') as f:
            # Read header
            vertex_count_bytes = f.read(4)
            if len(vertex_count_bytes) != 4:
                raise ValueError("Could not read vertex count")
            
            vertex_count = struct.unpack('<I', vertex_count_bytes)[0]
            print(f"   Read vertex count: {vertex_count}")
            
            # Read first few vertices for verification
            for i in range(min(3, vertex_count)):
                x_bytes = f.read(8)
                y_bytes = f.read(8)
                z_bytes = f.read(8)
                
                if len(x_bytes) != 8 or len(y_bytes) != 8 or len(z_bytes) != 8:
                    raise ValueError(f"Could not read vertex {i}")
                
                x = struct.unpack('<d', x_bytes)[0]
                y = struct.unpack('<d', y_bytes)[0]
                z = struct.unpack('<d', z_bytes)[0]
                
                print(f"   Vertex {i}: ({x:.6f}, {y:.6f}, {z:.6f})")
        
        print("✅ Binary file verification successful")
        return True
        
    except Exception as e:
        print(f"❌ Binary file verification failed: {e}")
        return False

def save_vertices_text(vertices, filepath):
    """
    Save vertices to text file for debugging (optional)
    
    Args:
        vertices: numpy array (N, 3)
        filepath: output file path
    """
    print(f"🔄 Saving vertices to text file: {filepath}")
    
    try:
        with open(filepath, 'w') as f:
            f.write(f"# STAR T-pose vertices\n")
            f.write(f"# Vertex count: {len(vertices)}\n")
            f.write(f"# Format: x y z\n")
            
            for i, vertex in enumerate(vertices):
                f.write(f"{vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}\n")
        
        print(f"✅ Text file saved successfully")
        return True
        
    except Exception as e:
        print(f"❌ Failed to save text file: {e}")
        return False

def main():
    """Main extraction process"""
    print("STAR VERTEX EXTRACTOR")
    print("=" * 50)
    
    start_time = time.time()
    
    # Check dependencies
    if not check_dependencies():
        print("❌ Required dependencies missing")
        return False
    
    # Extract vertices
    try:
        vertices = extract_star_t_pose_vertices(gender='neutral')
    except Exception as e:
        print(f"❌ Failed to extract STAR vertices: {e}")
        print("   Make sure STAR model files are downloaded and paths are correct")
        return False
    
    # Save binary file (primary format for C++)
    binary_filepath = "star_vertices.bin"
    if not save_vertices_binary(vertices, binary_filepath):
        return False
    
    # Verify binary file
    if not verify_binary_file(binary_filepath):
        return False
    
    # Save text file (optional, for debugging)
    text_filepath = "star_vertices.txt"
    save_vertices_text(vertices, text_filepath)
    
    # Summary
    total_time = time.time() - start_time
    print(f"\n{'=' * 50}")
    print(f"✅ VERTEX EXTRACTION COMPLETED!")
    print(f"   Binary file: {binary_filepath}")
    print(f"   Text file: {text_filepath}")
    print(f"   Vertex count: {len(vertices)}")
    print(f"   Time: {total_time:.1f} seconds")
    print(f"   Ready for C++ collision system")
    print(f"{'=' * 50}")
    
    return True

if __name__ == "__main__":
    try:
        success = main()
        if not success:
            sys.exit(1)
    except KeyboardInterrupt:
        print(f"\n❌ Extraction interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)