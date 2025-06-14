#!/usr/bin/env python3
"""
STAR Daemon - Real-time STAR mesh deformation service
Persistent Python process that loads STAR model once and serves mesh deformation requests
"""

import sys
import struct
import socket
import numpy as np
import time
import signal
import os
from pathlib import Path

# Add paths for STAR and star_body_system
current_dir = Path(__file__).parent
sys.path.append(str(current_dir))
sys.path.append(str(current_dir / "STAR"))
sys.path.append(str(current_dir / "star_body_system"))

try:
    from core.star_interface import STARInterface
    STAR_AVAILABLE = True
    print("✅ STAR interface loaded successfully")
except ImportError as e:
    print(f"ERROR: STAR model not available: {e}")
    print("Available paths:")
    for path in sys.path[-3:]:
        print(f"  {path}")
    try:
        import star
        print("✅ STAR module found")
    except ImportError:
        print("❌ STAR module not found")
    STAR_AVAILABLE = False


class STARDaemon:
    """STAR mesh deformation daemon with Unix socket IPC"""
    
    def __init__(self, socket_path="/tmp/star_daemon.sock", gender='neutral'):
        self.socket_path = socket_path
        self.gender = gender
        self.star_interface = None
        self.socket = None
        self.running = False
        
        # Performance tracking
        self.total_requests = 0
        self.total_computation_time = 0.0
        self.last_stats_time = time.time()
        
        # Expected data sizes
        self.POSE_PARAM_SIZE = 72 * 4  # 72 floats, 4 bytes each
        self.VERTEX_DATA_SIZE = 6890 * 3 * 4  # 6890 vertices * 3 coords * 4 bytes
        self.JOINT_DATA_SIZE = 24 * 3 * 4  # 24 joints * 3 coords * 4 bytes
        
    def initialize(self):
        """Initialize STAR model and socket"""
        print("🚀 Initializing STAR Daemon...")
        
        if not STAR_AVAILABLE:
            print("❌ STAR not available")
            return False
        
        try:
            # Load STAR model (expensive operation done once)
            print("   Loading STAR model...")
            start_time = time.time()
            self.star_interface = STARInterface(gender=self.gender)
            load_time = time.time() - start_time
            print(f"   ✅ STAR model loaded in {load_time:.2f}s")
            
            # Test STAR with neutral pose
            print("   Testing STAR interface...")
            vertices, joints = self.star_interface.get_neutral_pose()
            if vertices is None or joints is None:
                print("❌ STAR interface test failed")
                return False
            
            print(f"   ✅ STAR test successful: {len(vertices)} vertices, {len(joints)} joints")
            
            # Initialize socket
            if not self._init_socket():
                return False
            
            print("✅ STAR Daemon ready!")
            print(f"   Socket: {self.socket_path}")
            print(f"   Gender: {self.gender}")
            print(f"   Expected vertex count: 6890")
            print(f"   Expected joint count: 24")
            
            return True
            
        except Exception as e:
            print(f"❌ Initialization failed: {e}")
            return False
    
    def _init_socket(self):
        """Initialize Unix domain socket"""
        try:
            # Remove existing socket file
            if os.path.exists(self.socket_path):
                os.unlink(self.socket_path)
            
            # Create socket
            self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.socket.bind(self.socket_path)
            self.socket.listen(1)
            
            print(f"   ✅ Socket created: {self.socket_path}")
            return True
            
        except Exception as e:
            print(f"❌ Socket creation failed: {e}")
            return False
    
    def run(self):
        """Main daemon loop"""
        print("🎬 Starting STAR Daemon service...")
        print("   Waiting for connections...")
        print("   Press Ctrl+C to stop")
        
        self.running = True
        
        try:
            while self.running:
                # Accept connection
                try:
                    client_socket, addr = self.socket.accept()
                    print(f"📡 Client connected")
                    
                    # Handle client requests
                    self._handle_client(client_socket)
                    
                except socket.error as e:
                    if self.running:  # Only log if not shutting down
                        print(f"⚠️  Socket error: {e}")
                    break
                
        except KeyboardInterrupt:
            print("\n🛑 Shutdown requested...")
        finally:
            self._cleanup()
    
    def _handle_client(self, client_socket):
        """Handle requests from a single client"""
        try:
            while self.running:
                # Receive pose parameters (72 floats = 288 bytes)
                pose_data = self._recv_exactly(client_socket, self.POSE_PARAM_SIZE)
                if not pose_data:
                    break
                
                # Parse pose parameters
                pose_params = np.frombuffer(pose_data, dtype=np.float32).reshape(72)
                
                # Generate mesh
                start_time = time.time()
                vertices, joints = self._generate_mesh(pose_params)
                computation_time = time.time() - start_time
                
                if vertices is None or joints is None:
                    print("⚠️  Mesh generation failed")
                    break
                
                # Send response
                if not self._send_mesh_data(client_socket, vertices, joints):
                    break
                
                # Update statistics
                self._update_stats(computation_time)
                
        except Exception as e:
            print(f"⚠️  Client handling error: {e}")
        finally:
            client_socket.close()
            print("📡 Client disconnected")
    
    def _recv_exactly(self, sock, size):
        """Receive exactly 'size' bytes from socket"""
        data = b''
        while len(data) < size:
            packet = sock.recv(size - len(data))
            if not packet:
                return None
            data += packet
        return data
    
    def _generate_mesh(self, pose_params):
        """Generate deformed mesh using STAR"""
        try:
            # Call STAR forward kinematics
            vertices, joints = self.star_interface.get_mesh_and_joints(pose_params)
            
            if vertices is None or joints is None:
                return None, None
            
            # Validate sizes
            if len(vertices) != 6890 or len(joints) != 24:
                print(f"⚠️  Unexpected sizes: {len(vertices)} vertices, {len(joints)} joints")
                return None, None
            
            return vertices.astype(np.float32), joints.astype(np.float32)
            
        except Exception as e:
            print(f"⚠️  Mesh generation error: {e}")
            return None, None
    
    def _send_mesh_data(self, sock, vertices, joints):
        """Send mesh data to client"""
        try:
            # Send vertices (6890 * 3 * 4 = 82,680 bytes)
            vertex_data = vertices.flatten().tobytes()
            sock.sendall(struct.pack('I', len(vertex_data)))  # Size header
            sock.sendall(vertex_data)
            
            # Send joints (24 * 3 * 4 = 288 bytes)
            joint_data = joints.flatten().tobytes()
            sock.sendall(struct.pack('I', len(joint_data)))  # Size header
            sock.sendall(joint_data)
            
            return True
            
        except Exception as e:
            print(f"⚠️  Send error: {e}")
            return False
    
    def _update_stats(self, computation_time):
        """Update performance statistics"""
        self.total_requests += 1
        self.total_computation_time += computation_time
        
        # Print stats every 100 requests
        if self.total_requests % 100 == 0:
            avg_time = self.total_computation_time / self.total_requests
            current_time = time.time()
            elapsed = current_time - self.last_stats_time
            fps = 100 / elapsed if elapsed > 0 else 0
            
            print(f"📊 Stats: {self.total_requests} requests | "
                  f"Avg time: {avg_time*1000:.1f}ms | "
                  f"Current FPS: {fps:.1f}")
            
            self.last_stats_time = current_time
    
    def _cleanup(self):
        """Clean shutdown"""
        print("🔄 Cleaning up...")
        
        self.running = False
        
        if self.socket:
            self.socket.close()
        
        if os.path.exists(self.socket_path):
            os.unlink(self.socket_path)
        
        # Final stats
        if self.total_requests > 0:
            avg_time = self.total_computation_time / self.total_requests
            print(f"📊 Final stats: {self.total_requests} requests | "
                  f"Avg computation time: {avg_time*1000:.1f}ms")
        
        print("✅ STAR Daemon shutdown complete")


def signal_handler(signum, frame):
    """Handle shutdown signals"""
    print(f"\n🛑 Received signal {signum}")
    sys.exit(0)


def main():
    """Main entry point"""
    print("🎯 STAR MESH DEFORMATION DAEMON")
    print("=" * 50)
    
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Parse command line arguments
    gender = 'neutral'
    socket_path = "/tmp/star_daemon.sock"
    
    if len(sys.argv) > 1:
        gender = sys.argv[1]
    if len(sys.argv) > 2:
        socket_path = sys.argv[2]
    
    print(f"Gender: {gender}")
    print(f"Socket: {socket_path}")
    print()
    
    # Create and run daemon
    daemon = STARDaemon(socket_path, gender)
    
    if not daemon.initialize():
        print("❌ Failed to initialize daemon")
        return 1
    
    try:
        daemon.run()
    except Exception as e:
        print(f"❌ Daemon error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())