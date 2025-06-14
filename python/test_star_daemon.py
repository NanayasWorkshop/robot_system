#!/usr/bin/env python3
"""
Test STAR Daemon
Simple client to test if the STAR daemon works correctly
"""

import socket
import struct
import numpy as np
import time
import sys


def test_star_daemon(socket_path="/tmp/star_daemon.sock"):
    """Test the STAR daemon with various poses"""
    
    print("🧪 Testing STAR Daemon")
    print("=" * 40)
    
    # Connect to daemon
    print(f"1. Connecting to daemon at {socket_path}...")
    try:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.connect(socket_path)
        print("   ✅ Connected successfully")
    except Exception as e:
        print(f"   ❌ Connection failed: {e}")
        print("   Make sure the daemon is running: python3 star_daemon.py")
        return False
    
    try:
        # Test 1: Neutral pose
        print("\n2. Testing neutral pose...")
        pose_params = np.zeros(72, dtype=np.float32)
        
        vertices, joints = send_pose_request(sock, pose_params)
        if vertices is not None and joints is not None:
            print(f"   ✅ Success: {len(vertices)} vertices, {len(joints)} joints")
            print(f"   Vertex bounds: X[{vertices[:, 0].min():.3f}, {vertices[:, 0].max():.3f}]")
            print(f"                  Y[{vertices[:, 1].min():.3f}, {vertices[:, 1].max():.3f}]")
            print(f"                  Z[{vertices[:, 2].min():.3f}, {vertices[:, 2].max():.3f}]")
        else:
            print("   ❌ Failed to get neutral pose")
            return False
        
        # Test 2: Spine bend
        print("\n3. Testing spine bend...")
        pose_params = np.zeros(72, dtype=np.float32)
        pose_params[6:9] = [0.3, 0, 0]  # Bend spine forward
        
        vertices_bent, joints_bent = send_pose_request(sock, pose_params)
        if vertices_bent is not None and joints_bent is not None:
            print(f"   ✅ Success: spine bent pose")
            
            # Compare with neutral pose
            vertex_diff = np.linalg.norm(vertices_bent - vertices, axis=1).mean()
            joint_diff = np.linalg.norm(joints_bent - joints, axis=1).mean()
            print(f"   Average vertex movement: {vertex_diff:.4f}m")
            print(f"   Average joint movement: {joint_diff:.4f}m")
            
            if vertex_diff > 0.001:  # Vertices should move significantly
                print("   ✅ Mesh deformation detected")
            else:
                print("   ⚠️  Warning: Very small mesh deformation")
        else:
            print("   ❌ Failed to get bent pose")
        
        # Test 3: Arm movements
        print("\n4. Testing arm movement...")
        pose_params = np.zeros(72, dtype=np.float32)
        pose_params[48] = 0.5  # Left shoulder rotation
        pose_params[51] = -0.5  # Right shoulder rotation (opposite)
        
        vertices_arms, joints_arms = send_pose_request(sock, pose_params)
        if vertices_arms is not None and joints_arms is not None:
            arm_vertex_diff = np.linalg.norm(vertices_arms - vertices, axis=1).mean()
            print(f"   ✅ Success: arm movement, avg vertex diff: {arm_vertex_diff:.4f}m")
        else:
            print("   ❌ Failed to get arm pose")
        
        # Test 4: Performance test
        print("\n5. Performance test (10 requests)...")
        times = []
        
        for i in range(10):
            # Random small pose variations
            pose_params = np.random.normal(0, 0.1, 72).astype(np.float32)
            
            start_time = time.time()
            vertices_perf, joints_perf = send_pose_request(sock, pose_params)
            elapsed = time.time() - start_time
            
            if vertices_perf is not None:
                times.append(elapsed * 1000)  # Convert to ms
                print(f"   Request {i+1}: {elapsed*1000:.1f}ms")
            else:
                print(f"   Request {i+1}: FAILED")
        
        if times:
            avg_time = np.mean(times)
            max_time = np.max(times)
            min_time = np.min(times)
            print(f"   ✅ Performance: avg={avg_time:.1f}ms, min={min_time:.1f}ms, max={max_time:.1f}ms")
            
            if avg_time < 10:
                print("   ✅ Excellent performance (<10ms average)")
            elif avg_time < 20:
                print("   ✅ Good performance (<20ms average)")
            else:
                print("   ⚠️  Slow performance (>20ms average)")
        
        print("\n6. Testing error handling...")
        
        # Test invalid pose size
        try:
            invalid_pose = np.zeros(50, dtype=np.float32)  # Wrong size
            sock.send(invalid_pose.tobytes())
            print("   ⚠️  Daemon should reject invalid pose size")
        except Exception as e:
            print(f"   ✅ Proper error handling: {e}")
        
        print("\n" + "=" * 40)
        print("🎉 STAR Daemon test complete!")
        
        return True
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        return False
    finally:
        sock.close()


def send_pose_request(sock, pose_params):
    """Send pose parameters and receive mesh data"""
    try:
        # Send pose parameters (72 floats = 288 bytes)
        pose_data = pose_params.tobytes()
        if len(pose_data) != 288:
            print(f"   ❌ Invalid pose data size: {len(pose_data)} bytes")
            return None, None
        
        sock.send(pose_data)
        
        # Receive vertices
        vertex_size_data = sock.recv(4)
        if len(vertex_size_data) != 4:
            print("   ❌ Failed to receive vertex size header")
            return None, None
        
        vertex_size = struct.unpack('I', vertex_size_data)[0]
        expected_vertex_size = 6890 * 3 * 4  # 6890 vertices * 3 coords * 4 bytes
        
        if vertex_size != expected_vertex_size:
            print(f"   ❌ Unexpected vertex size: {vertex_size}, expected {expected_vertex_size}")
            return None, None
        
        # Receive vertex data
        vertex_data = b''
        while len(vertex_data) < vertex_size:
            chunk = sock.recv(vertex_size - len(vertex_data))
            if not chunk:
                print("   ❌ Connection closed while receiving vertices")
                return None, None
            vertex_data += chunk
        
        # Receive joints
        joint_size_data = sock.recv(4)
        if len(joint_size_data) != 4:
            print("   ❌ Failed to receive joint size header")
            return None, None
        
        joint_size = struct.unpack('I', joint_size_data)[0]
        expected_joint_size = 24 * 3 * 4  # 24 joints * 3 coords * 4 bytes
        
        if joint_size != expected_joint_size:
            print(f"   ❌ Unexpected joint size: {joint_size}, expected {expected_joint_size}")
            return None, None
        
        # Receive joint data
        joint_data = b''
        while len(joint_data) < joint_size:
            chunk = sock.recv(joint_size - len(joint_data))
            if not chunk:
                print("   ❌ Connection closed while receiving joints")
                return None, None
            joint_data += chunk
        
        # Convert to numpy arrays
        vertices = np.frombuffer(vertex_data, dtype=np.float32).reshape(6890, 3)
        joints = np.frombuffer(joint_data, dtype=np.float32).reshape(24, 3)
        
        return vertices, joints
        
    except Exception as e:
        print(f"   ❌ Request failed: {e}")
        return None, None


if __name__ == "__main__":
    socket_path = "/tmp/star_daemon.sock"
    
    if len(sys.argv) > 1:
        socket_path = sys.argv[1]
    
    print(f"Testing daemon at: {socket_path}")
    print("Make sure to start the daemon first:")
    print("  python3 star_daemon.py")
    print()
    
    success = test_star_daemon(socket_path)
    sys.exit(0 if success else 1)