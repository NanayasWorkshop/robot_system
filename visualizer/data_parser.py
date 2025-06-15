#!/usr/bin/env python3
"""
Data Parser - Pure C++ collision data display
Clean rewrite: All features, no coordinate transformation baggage
UPDATED: Fixed joint visualization with proper coordinates, colors, and skeleton
NO TRANSFORMATION NEEDED - joints already in STAR coordinates
"""

import struct
import numpy as np
import open3d as o3d
from typing import Dict, Optional, Tuple
from dataclasses import dataclass
from network_receiver import PacketType, PacketBuffer

# Import STAR body definitions for skeleton
try:
    import sys
    import os
    # Add the parent directory to path to find python module
    parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, parent_dir)
    from python.star_body_system.core.body_definitions import BodyDefinitions
    print("✅ BodyDefinitions imported successfully")
except ImportError as e:
    print(f"⚠️  Warning: Could not import BodyDefinitions ({e}), skeleton lines disabled")
    BodyDefinitions = None


@dataclass
class CapsuleData:
    start: np.ndarray
    end: np.ndarray
    radius: float


@dataclass
class VertexBatchAccumulator:
    frame_id: int
    total_batches: int
    received_batches: Dict[int, np.ndarray]
    complete: bool = False
    
    def add_batch(self, batch_index: int, vertices: np.ndarray) -> bool:
        self.received_batches[batch_index] = vertices
        self.complete = len(self.received_batches) == self.total_batches
        return self.complete
    
    def get_complete_vertices(self) -> Optional[np.ndarray]:
        if not self.complete:
            return None
        
        all_vertices = []
        for batch_idx in range(self.total_batches):
            if batch_idx in self.received_batches:
                all_vertices.extend(self.received_batches[batch_idx])
            else:
                return None
        
        return np.array(all_vertices) if all_vertices else None


class DataParser:
    """Parse C++ collision packets into Open3D geometries"""
    
    def __init__(self):
        # Colors
        self.ROBOT_COLOR = np.array([0.27, 0.51, 0.71])      # Steel blue
        self.HUMAN_COLOR = np.array([0.96, 0.87, 0.70])      # Warm beige (vertices)
        self.JOINT_COLOR = np.array([1.0, 0.2, 0.2])        # Bright red (joints)
        self.BONE_COLOR = np.array([0.8, 0.8, 0.2])         # Yellow-green (skeleton)
        
        self.COLLISION_COLORS = {
            'low': np.array([0.0, 0.8, 0.0]),      # Green
            'med': np.array([1.0, 1.0, 0.0]),      # Yellow
            'high': np.array([1.0, 0.5, 0.0]),     # Orange
            'max': np.array([1.0, 0.0, 0.0])       # Red
        }
        
        # Point generation
        self.CAPSULE_POINTS = 25
        self.CIRCLE_POINTS = 8
        
        # Joint sizing
        self.JOINT_SPHERE_RADIUS = 0.01  # 1cm sphere radius for joints
        
        # Vertex batching
        self.vertex_accumulators: Dict[int, VertexBatchAccumulator] = {}
        self.max_cached_frames = 5
    
    def create_joint_spheres(self, joint_positions: np.ndarray) -> o3d.geometry.TriangleMesh:
        """Create sphere meshes for joints"""
        if len(joint_positions) == 0:
            return o3d.geometry.TriangleMesh()
        
        # Create combined mesh for all joints
        combined_mesh = o3d.geometry.TriangleMesh()
        
        for joint_pos in joint_positions:
            # Create sphere at joint position
            sphere = o3d.geometry.TriangleMesh.create_sphere(
                radius=self.JOINT_SPHERE_RADIUS,
                resolution=10
            )
            sphere.translate(joint_pos)
            sphere.paint_uniform_color(self.JOINT_COLOR)
            
            # Add to combined mesh
            combined_mesh += sphere
        
        return combined_mesh
    
    def create_skeleton_lines(self, joint_positions: np.ndarray) -> Optional[o3d.geometry.LineSet]:
        """Create skeleton line connections between joints"""
        if BodyDefinitions is None or len(joint_positions) != 24:
            return None
        
        lines = []
        colors = []
        
        # Use DETAILED_BONES for 24 bone connections
        for joint1_idx, joint2_idx, bone_name in BodyDefinitions.DETAILED_BONES:
            if joint1_idx < len(joint_positions) and joint2_idx < len(joint_positions):
                lines.append([joint1_idx, joint2_idx])
                colors.append(self.BONE_COLOR)
        
        if not lines:
            return None
        
        # Create LineSet
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(joint_positions)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors)
        
        return line_set
    
    def sample_capsule_points(self, capsule: CapsuleData, num_points: int) -> np.ndarray:
        """Generate points on capsule surface"""
        points = []
        axis = capsule.end - capsule.start
        length = np.linalg.norm(axis)
        
        if length < 1e-6:
            return np.array([capsule.start])
        
        axis_norm = axis / length
        
        # Create perpendicular vectors
        if abs(axis_norm[2]) < 0.9:
            perp1 = np.cross(axis_norm, np.array([0, 0, 1]))
        else:
            perp1 = np.cross(axis_norm, np.array([1, 0, 0]))
        perp1 = perp1 / np.linalg.norm(perp1)
        perp2 = np.cross(axis_norm, perp1)
        
        # Cylinder body
        body_points = max(1, num_points // 2)
        for i in range(body_points):
            t = i / max(1, body_points - 1)
            center = capsule.start + t * axis
            angle = 2 * np.pi * (i % self.CIRCLE_POINTS) / self.CIRCLE_POINTS
            offset = capsule.radius * (np.cos(angle) * perp1 + np.sin(angle) * perp2)
            points.append(center + offset)
        
        # End caps
        cap_points = max(1, (num_points - body_points) // 2)
        for i in range(cap_points):
            angle = np.pi * i / max(1, cap_points - 1)
            phi = 2 * np.pi * (i % 4) / 4
            cap_offset = capsule.radius * np.sin(angle)
            cap_height = capsule.radius * np.cos(angle)
            
            # Start cap
            offset = (cap_offset * (np.cos(phi) * perp1 + np.sin(phi) * perp2) - 
                     cap_height * axis_norm)
            points.append(capsule.start + offset)
            
            # End cap
            offset = (cap_offset * (np.cos(phi) * perp1 + np.sin(phi) * perp2) + 
                     cap_height * axis_norm)
            points.append(capsule.end + offset)
        
        return np.array(points) if points else np.array([capsule.start])
    
    def sample_sphere_points(self, center: np.ndarray, radius: float, num_points: int) -> np.ndarray:
        """Generate points on sphere surface"""
        points = []
        
        # Use spherical coordinates to generate points uniformly on sphere
        for i in range(num_points):
            # Uniform distribution on sphere using golden ratio
            theta = 2 * np.pi * i / num_points
            phi = np.arccos(1 - 2 * (i + 0.5) / num_points)
            
            # Convert to Cartesian coordinates
            x = radius * np.sin(phi) * np.cos(theta)
            y = radius * np.sin(phi) * np.sin(theta)
            z = radius * np.cos(phi)
            
            point = center + np.array([x, y, z])
            points.append(point)
        
        return np.array(points) if points else np.array([center])
    
    def get_collision_color(self, depth: float) -> np.ndarray:
        """Color based on penetration depth"""
        if depth <= 0.002:    # 2mm
            return self.COLLISION_COLORS['low']
        elif depth <= 0.005:  # 5mm
            return self.COLLISION_COLORS['med']
        elif depth <= 0.010:  # 10mm
            return self.COLLISION_COLORS['high']
        else:
            return self.COLLISION_COLORS['max']
    
    def parse_robot_capsules(self, data: bytes) -> Optional[o3d.geometry.PointCloud]:
        """Parse robot capsule data"""
        try:
            if len(data) < 4:
                return None
            
            count = struct.unpack('I', data[:4])[0]
            if count == 0:
                return None
            
            capsules = []
            offset = 4
            for _ in range(count):
                if offset + 28 > len(data):
                    break
                
                values = struct.unpack('fffffff', data[offset:offset+28])
                capsule = CapsuleData(
                    start=np.array([values[0], values[1], values[2]]),
                    end=np.array([values[3], values[4], values[5]]),
                    radius=values[6]
                )
                capsules.append(capsule)
                offset += 28
            
            # Generate points
            all_points = []
            for capsule in capsules:
                points = self.sample_capsule_points(capsule, self.CAPSULE_POINTS)
                all_points.extend(points)
            
            if not all_points:
                return None
            
            # Create point cloud
            pc = o3d.geometry.PointCloud()
            pc.points = o3d.utility.Vector3dVector(np.array(all_points))
            colors = np.tile(self.ROBOT_COLOR, (len(all_points), 1))
            pc.colors = o3d.utility.Vector3dVector(colors)
            
            return pc
            
        except Exception:
            return None
    
    def parse_human_pose(self, data: bytes) -> Tuple[Optional[o3d.geometry.TriangleMesh], Optional[o3d.geometry.LineSet]]:
        """Parse human joint data and return joint spheres + skeleton lines"""
        try:
            if len(data) < 4:
                return None, None
            
            count = struct.unpack('I', data[:4])[0]
            if count == 0:
                return None, None
            
            joints = []
            offset = 4
            for _ in range(min(count, 24)):  # STAR has 24 joints
                if offset + 12 > len(data):
                    break
                
                values = struct.unpack('fff', data[offset:offset+12])
                joint = np.array([values[0], values[1], values[2]])
                joints.append(joint)
                offset += 12
            
            if not joints:
                return None, None
            
            joint_positions = np.array(joints)
            
            # No transformation needed - joints already in STAR coordinates!
            
            # Create joint spheres
            joint_spheres = self.create_joint_spheres(joint_positions)
            
            # Create skeleton lines
            skeleton_lines = self.create_skeleton_lines(joint_positions)
            
            return joint_spheres, skeleton_lines
            
        except Exception as e:
            print(f"⚠️  Error parsing joints: {e}")
            return None, None
    
    def parse_human_vertices_batch(self, data: bytes, frame_id: int) -> Optional[int]:
        """Parse human vertex batch"""
        try:
            if len(data) < 12:
                return None
            
            header = struct.unpack('III', data[:12])
            vertex_count, batch_index, total_batches = header
            
            if vertex_count == 0:
                return 0
            
            # Parse vertices
            vertices = []
            offset = 12
            for _ in range(vertex_count):
                if offset + 12 > len(data):
                    break
                
                try:
                    values = struct.unpack('fff', data[offset:offset+12])
                    vertex = np.array([values[0], values[1], values[2]])
                    
                    if np.isfinite(vertex).all():
                        vertices.append(vertex)
                    
                    offset += 12
                except struct.error:
                    break
            
            if not vertices:
                return 0
            
            # Add to accumulator
            if frame_id not in self.vertex_accumulators:
                self.vertex_accumulators[frame_id] = VertexBatchAccumulator(
                    frame_id=frame_id,
                    total_batches=total_batches,
                    received_batches={}
                )
            
            self.vertex_accumulators[frame_id].add_batch(batch_index, vertices)
            self._cleanup_old_accumulators(frame_id)
            
            return len(vertices)
            
        except Exception:
            return None
    
    def get_complete_human_vertices(self, frame_id: int) -> Optional[o3d.geometry.PointCloud]:
        """Get complete human mesh if ready"""
        if frame_id not in self.vertex_accumulators:
            return None
        
        accumulator = self.vertex_accumulators[frame_id]
        if not accumulator.complete:
            return None
        
        all_vertices = accumulator.get_complete_vertices()
        if all_vertices is None:
            return None
        
        # Create point cloud
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(all_vertices)
        colors = np.tile(self.HUMAN_COLOR, (len(all_vertices), 1))
        pc.colors = o3d.utility.Vector3dVector(colors)
        
        # Clean up
        del self.vertex_accumulators[frame_id]
        
        return pc
    
    def _cleanup_old_accumulators(self, current_frame_id: int):
        """Remove old accumulators"""
        frames_to_remove = [
            fid for fid in self.vertex_accumulators 
            if fid < current_frame_id - self.max_cached_frames
        ]
        for frame_id in frames_to_remove:
            del self.vertex_accumulators[frame_id]
    
    def parse_collision_layers(self, data: bytes) -> Dict[str, o3d.geometry.PointCloud]:
        """Parse collision layer data (Layer 3, Layer 2, Layer 1)"""
        try:
            layer_geometries = {}
            offset = 0
            
            # Parse Layer 3 count
            if offset + 4 > len(data):
                return {}
            
            layer3_count = struct.unpack('I', data[offset:offset+4])[0]
            offset += 4
            
            # Parse Layer 3 primitives (capsules)
            layer3_points = []
            layer3_colors = []
            for i in range(min(layer3_count, 16)):  # Max 16 from C++
                if offset + 29 > len(data):  # 7 floats + 1 byte
                    break
                
                values = struct.unpack('fffffffB', data[offset:offset+29])
                start = np.array([values[0], values[1], values[2]])
                end = np.array([values[3], values[4], values[5]])
                radius = values[6]
                is_active = values[7]
                
                # Create capsule points
                if radius > 0:
                    capsule = CapsuleData(start, end, radius)
                    points = self.sample_capsule_points(capsule, 50)  # Increased from 15 to 50
                    layer3_points.extend(points)
                    
                    # Color based on activity
                    color = np.array([0.0, 1.0, 0.0]) if is_active else np.array([0.5, 0.5, 0.5])
                    colors = np.tile(color, (len(points), 1))
                    layer3_colors.extend(colors)
                
                offset += 29
            
            if layer3_points:
                pc = o3d.geometry.PointCloud()
                pc.points = o3d.utility.Vector3dVector(np.array(layer3_points))
                pc.colors = o3d.utility.Vector3dVector(np.array(layer3_colors))
                layer_geometries['layer3'] = pc
            
            # Parse Layer 2 count
            if offset + 4 > len(data):
                return layer_geometries
            
            layer2_count = struct.unpack('I', data[offset:offset+4])[0]
            offset += 4
            
            # Parse Layer 2 primitives (capsules)
            layer2_points = []
            layer2_colors = []
            for i in range(min(layer2_count, 32)):  # Max 32 from C++
                if offset + 29 > len(data):
                    break
                
                values = struct.unpack('fffffffB', data[offset:offset+29])
                start = np.array([values[0], values[1], values[2]])
                end = np.array([values[3], values[4], values[5]])
                radius = values[6]
                is_active = values[7]
                
                if radius > 0:
                    capsule = CapsuleData(start, end, radius)
                    points = self.sample_capsule_points(capsule, 40)  # Increased from 10 to 40
                    layer2_points.extend(points)
                    
                    color = np.array([0.0, 0.0, 1.0]) if is_active else np.array([0.3, 0.3, 0.3])
                    colors = np.tile(color, (len(points), 1))
                    layer2_colors.extend(colors)
                
                offset += 29
            
            if layer2_points:
                pc = o3d.geometry.PointCloud()
                pc.points = o3d.utility.Vector3dVector(np.array(layer2_points))
                pc.colors = o3d.utility.Vector3dVector(np.array(layer2_colors))
                layer_geometries['layer2'] = pc
            
            # Parse Layer 1 count
            if offset + 4 > len(data):
                return layer_geometries
            
            layer1_count = struct.unpack('I', data[offset:offset+4])[0]
            offset += 4
            
            # Parse Layer 1 primitives (spheres)
            layer1_points = []
            layer1_colors = []
            for i in range(min(layer1_count, 128)):  # Max 128 from C++
                if offset + 17 > len(data):  # 4 floats + 1 byte
                    break
                
                values = struct.unpack('ffffB', data[offset:offset+17])
                center = np.array([values[0], values[1], values[2]])
                radius = values[3]
                is_active = values[4]
                
                if radius > 0:
                    # Create sphere points (many points on sphere surface)
                    sphere_points = self.sample_sphere_points(center, radius, 30)  # 30 points per sphere
                    layer1_points.extend(sphere_points)
                    
                    color = np.array([1.0, 0.0, 1.0]) if is_active else np.array([0.2, 0.2, 0.2])
                    colors = np.tile(color, (len(sphere_points), 1))
                    layer1_colors.extend(colors)
                
                offset += 17
            
            if layer1_points:
                pc = o3d.geometry.PointCloud()
                pc.points = o3d.utility.Vector3dVector(np.array(layer1_points))
                pc.colors = o3d.utility.Vector3dVector(np.array(layer1_colors))
                layer_geometries['layer1'] = pc
            
            return layer_geometries
            
        except Exception as e:
            print(f"⚠️  Error parsing collision layers: {e}")
            return {}
    
    def parse_collision_contacts(self, data: bytes) -> Optional[o3d.geometry.PointCloud]:
        """Parse collision contact data"""
        try:
            if len(data) < 12:
                return None
            
            try:
                header = struct.unpack('IBBf', data[:10])
                contact_count, has_collision, _, max_penetration = header
                
                if has_collision and contact_count > 0:
                    # Simple collision indicator at origin
                    points = np.array([[0, 0, 0]])
                    pc = o3d.geometry.PointCloud()
                    pc.points = o3d.utility.Vector3dVector(points)
                    color = self.get_collision_color(max_penetration)
                    pc.colors = o3d.utility.Vector3dVector([color])
                    return pc
                    
            except struct.error:
                pass
            
            return None
            
        except Exception:
            return None
    
    def parse_frame(self, frame: PacketBuffer) -> Dict[str, o3d.geometry.PointCloud]:
        """Parse complete frame"""
        geometries = {}
        
        # Helper to get packets
        def get_packets(packet_type):
            if hasattr(frame, 'packets') and packet_type in frame.packets:
                data = frame.packets[packet_type]
                return data if isinstance(data, list) else [data]
            return []
        
        # Robot capsules
        robot_packets = get_packets(PacketType.ROBOT_CAPSULES)
        if robot_packets and len(robot_packets[-1]) > 4:
            robot_pc = self.parse_robot_capsules(robot_packets[-1])
            if robot_pc:
                geometries['robot'] = robot_pc
        
        # Human data
        human_packets = get_packets(PacketType.HUMAN_POSE)
        if human_packets:
            # Separate joints and vertices by size - joints are ~296 bytes, vertices are ~6012 bytes
            joint_packets = [p for p in human_packets if len(p) < 1000]  # Changed threshold
            vertex_packets = [p for p in human_packets if len(p) >= 1000]
            
            # Process joints first (smaller packets)
            if joint_packets:
                joint_spheres, skeleton_lines = self.parse_human_pose(joint_packets[-1])
                if joint_spheres:
                    geometries['joints'] = joint_spheres
                if skeleton_lines:
                    geometries['skeleton'] = skeleton_lines
            
            # Process vertices (larger packets)
            if vertex_packets:
                # Process vertex batches
                for vertex_data in vertex_packets:
                    self.parse_human_vertices_batch(vertex_data, frame.frame_id)
                
                # Check for complete mesh
                complete_pc = self.get_complete_human_vertices(frame.frame_id)
                if complete_pc:
                    geometries['human'] = complete_pc
                else:
                    # Partial mesh fallback
                    if frame.frame_id in self.vertex_accumulators:
                        acc = self.vertex_accumulators[frame.frame_id]
                        if acc.received_batches:
                            partial_vertices = []
                            for batch in acc.received_batches.values():
                                partial_vertices.extend(batch)
                            
                            if partial_vertices:
                                pc = o3d.geometry.PointCloud()
                                pc.points = o3d.utility.Vector3dVector(np.array(partial_vertices))
                                colors = np.tile(self.HUMAN_COLOR, (len(partial_vertices), 1))
                                pc.colors = o3d.utility.Vector3dVector(colors)
                                geometries['human'] = pc
        
        # Collision contacts
        collision_packets = get_packets(PacketType.COLLISION_CONTACTS)
        if collision_packets and len(collision_packets[-1]) > 12:
            collision_pc = self.parse_collision_contacts(collision_packets[-1])
            if collision_pc:
                geometries['collision'] = collision_pc
        
        # Collision layers (NEW)
        layer_packets = get_packets(PacketType.COLLISION_LAYERS)
        if layer_packets and len(layer_packets[-1]) > 100:
            layer_geometries = self.parse_collision_layers(layer_packets[-1])
            for layer_name, layer_geom in layer_geometries.items():
                geometries[layer_name] = layer_geom
        
        return geometries


# Test
if __name__ == "__main__":
    print("🧪 Testing DataParser (Clean Version - No Transform)")
    
    parser = DataParser()
    
    # Test capsule sampling
    capsule = CapsuleData(
        start=np.array([0, 0, 0]),
        end=np.array([0.1, 0, 0]),
        radius=0.025
    )
    points = parser.sample_capsule_points(capsule, 25)
    print(f"Generated {len(points)} capsule points")
    
    # Test skeleton creation
    if BodyDefinitions:
        fake_joints = np.random.rand(24, 3)
        skeleton = parser.create_skeleton_lines(fake_joints)
        if skeleton:
            print(f"Created skeleton with {len(skeleton.lines)} bones")
        else:
            print("Failed to create skeleton")
    
    print("✅ Tests complete - No coordinate transformation needed!")