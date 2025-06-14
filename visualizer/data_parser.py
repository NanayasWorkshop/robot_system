#!/usr/bin/env python3
"""
Data Parser - Pure C++ collision data display
Clean rewrite: All features, no coordinate transformation baggage
"""

import struct
import numpy as np
import open3d as o3d
from typing import Dict, Optional
from dataclasses import dataclass
from network_receiver import PacketType, PacketBuffer


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
        self.HUMAN_COLOR = np.array([0.96, 0.87, 0.70])      # Warm beige
        self.COLLISION_COLORS = {
            'low': np.array([0.0, 0.8, 0.0]),      # Green
            'med': np.array([1.0, 1.0, 0.0]),      # Yellow
            'high': np.array([1.0, 0.5, 0.0]),     # Orange
            'max': np.array([1.0, 0.0, 0.0])       # Red
        }
        
        # Point generation
        self.CAPSULE_POINTS = 25
        self.CIRCLE_POINTS = 8
        
        # Vertex batching
        self.vertex_accumulators: Dict[int, VertexBatchAccumulator] = {}
        self.max_cached_frames = 5
    
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
    
    def parse_human_pose(self, data: bytes) -> Optional[o3d.geometry.PointCloud]:
        """Parse human joint data"""
        try:
            if len(data) < 4:
                return None
            
            count = struct.unpack('I', data[:4])[0]
            if count == 0:
                return None
            
            joints = []
            offset = 4
            for _ in range(min(count, 24)):
                if offset + 12 > len(data):
                    break
                
                values = struct.unpack('fff', data[offset:offset+12])
                joint = np.array([values[0], values[1], values[2]])
                joints.append(joint)
                offset += 12
            
            if not joints:
                return None
            
            # Create point cloud
            pc = o3d.geometry.PointCloud()
            pc.points = o3d.utility.Vector3dVector(np.array(joints))
            colors = np.tile(self.HUMAN_COLOR, (len(joints), 1))
            pc.colors = o3d.utility.Vector3dVector(colors)
            
            return pc
            
        except Exception:
            return None
    
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
            # Separate joints and vertices
            joint_packets = [p for p in human_packets if len(p) <= 200]
            vertex_packets = [p for p in human_packets if len(p) > 200]
            
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
            
            elif joint_packets:
                # Joint-only visualization
                human_pc = self.parse_human_pose(joint_packets[-1])
                if human_pc:
                    geometries['human'] = human_pc
        
        # Collision contacts
        collision_packets = get_packets(PacketType.COLLISION_CONTACTS)
        if collision_packets and len(collision_packets[-1]) > 12:
            collision_pc = self.parse_collision_contacts(collision_packets[-1])
            if collision_pc:
                geometries['collision'] = collision_pc
        
        return geometries


# Test
if __name__ == "__main__":
    print("🧪 Testing DataParser (Clean Version)")
    
    parser = DataParser()
    
    # Test capsule sampling
    capsule = CapsuleData(
        start=np.array([0, 0, 0]),
        end=np.array([0.1, 0, 0]),
        radius=0.025
    )
    points = parser.sample_capsule_points(capsule, 25)
    print(f"Generated {len(points)} capsule points")
    
    print("✅ Tests complete")