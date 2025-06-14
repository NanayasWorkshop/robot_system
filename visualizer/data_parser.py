#!/usr/bin/env python3
"""
Data Parser - Transform C++ collision data to Open3D geometries
Handles coordinate transformation and point cloud generation
OPTIMIZED: Reduced verbose logging while preserving functionality
"""

import struct
import numpy as np
import open3d as o3d
from typing import List, Optional, Tuple, Dict
from dataclasses import dataclass
from network_receiver import PacketType, PacketBuffer


@dataclass
class CapsuleData:
    """Capsule data structure matching C++"""
    start: np.ndarray  # 3D point
    end: np.ndarray    # 3D point
    radius: float


@dataclass
class ContactData:
    """Collision contact data matching C++"""
    contact_point: np.ndarray  # 3D contact position
    normal: np.ndarray         # 3D contact normal
    penetration_depth: float
    robot_capsule_index: int


@dataclass
class VertexBatchAccumulator:
    """Accumulates vertex batches for complete mesh reconstruction"""
    frame_id: int
    total_batches: int
    received_batches: Dict[int, np.ndarray]  # batch_index -> vertices
    complete: bool = False
    
    def add_batch(self, batch_index: int, vertices: np.ndarray) -> bool:
        """Add a vertex batch and check if complete"""
        self.received_batches[batch_index] = vertices
        self.complete = len(self.received_batches) == self.total_batches
        return self.complete
    
    def get_complete_vertices(self) -> Optional[np.ndarray]:
        """Get all vertices combined in correct order"""
        if not self.complete:
            return None
        
        # Combine batches in order
        all_vertices = []
        for batch_idx in range(self.total_batches):
            if batch_idx in self.received_batches:
                all_vertices.extend(self.received_batches[batch_idx])
            else:
                print(f"⚠️ Missing batch {batch_idx} in frame {self.frame_id}")
                return None
        
        return np.array(all_vertices) if all_vertices else None


class DataParser:
    """Parse binary packets and create Open3D geometries"""
    
    def __init__(self):
        # Constants matching C++ coordinate system
        self.METERS_TO_MM = 1000.0
        
        # Point sampling parameters
        self.CAPSULE_POINTS_PER_SEGMENT = 25
        self.POINTS_PER_CIRCLE = 8
        
        # Colors (RGB normalized)
        self.ROBOT_COLOR = np.array([0.27, 0.51, 0.71])      # Steel blue
        self.HUMAN_COLOR = np.array([0.96, 0.87, 0.70])      # Warm beige
        self.COLLISION_COLORS = {
            'green': np.array([0.0, 0.8, 0.0]),     # 0-2mm
            'yellow': np.array([1.0, 1.0, 0.0]),    # 2-5mm
            'orange': np.array([1.0, 0.5, 0.0]),    # 5-10mm
            'red': np.array([1.0, 0.0, 0.0])        # >10mm
        }
        
        # Vertex batch accumulation
        self.vertex_accumulators: Dict[int, VertexBatchAccumulator] = {}
        self.max_cached_frames = 5  # Keep last 5 frames to handle out-of-order packets
    
    def transform_star_to_collision(self, star_point: np.ndarray) -> np.ndarray:
        """
        Transform STAR coordinates to collision coordinates
        Exact copy of C++ transform_point_star_to_collision()
        
        STAR: Y-up, meters, person lying on back
        Collision: Z-up, millimeters, person standing upright
        """
        x_star, y_star, z_star = star_point[0], star_point[1], star_point[2]
        
        return np.array([
            x_star * self.METERS_TO_MM,      # X unchanged, meters → mm
            -z_star * self.METERS_TO_MM,     # Y = -Z (flip to face forward)
            y_star * self.METERS_TO_MM       # Z = Y (Y-up → Z-up)
        ])
    
    def sample_capsule_points(self, capsule: CapsuleData, num_points: int) -> np.ndarray:
        """Generate point samples on capsule surface"""
        points = []
        
        # Capsule axis
        axis = capsule.end - capsule.start
        length = np.linalg.norm(axis)
        
        if length < 1e-6:  # Degenerate capsule
            return np.array([capsule.start])
        
        axis_normalized = axis / length
        
        # Create perpendicular vectors for cylinder sampling
        if abs(axis_normalized[2]) < 0.9:
            perp1 = np.cross(axis_normalized, np.array([0, 0, 1]))
        else:
            perp1 = np.cross(axis_normalized, np.array([1, 0, 0]))
        perp1 = perp1 / np.linalg.norm(perp1)
        perp2 = np.cross(axis_normalized, perp1)
        
        # Sample points along cylinder body
        body_points = max(1, num_points // 2)
        for i in range(body_points):
            # Position along axis
            t = i / max(1, body_points - 1)
            center = capsule.start + t * axis
            
            # Points around circumference
            angle = 2 * np.pi * (i % self.POINTS_PER_CIRCLE) / self.POINTS_PER_CIRCLE
            offset = capsule.radius * (np.cos(angle) * perp1 + np.sin(angle) * perp2)
            points.append(center + offset)
        
        # Sample points on spherical caps
        cap_points = max(1, (num_points - body_points) // 2)
        
        # Start cap
        for i in range(cap_points):
            angle = np.pi * i / max(1, cap_points - 1)  # 0 to π
            phi = 2 * np.pi * (i % 4) / 4  # Azimuth
            
            # Sphere point in cap direction
            cap_offset = capsule.radius * np.sin(angle)
            cap_height = -capsule.radius * np.cos(angle)
            
            offset = (cap_offset * (np.cos(phi) * perp1 + np.sin(phi) * perp2) + 
                     cap_height * axis_normalized)
            points.append(capsule.start + offset)
        
        # End cap
        for i in range(cap_points):
            angle = np.pi * i / max(1, cap_points - 1)  # 0 to π
            phi = 2 * np.pi * (i % 4) / 4  # Azimuth
            
            # Sphere point in cap direction
            cap_offset = capsule.radius * np.sin(angle)
            cap_height = capsule.radius * np.cos(angle)
            
            offset = (cap_offset * (np.cos(phi) * perp1 + np.sin(phi) * perp2) + 
                     cap_height * axis_normalized)
            points.append(capsule.end + offset)
        
        return np.array(points) if points else np.array([capsule.start])
    
    def get_collision_color(self, penetration_depth: float) -> np.ndarray:
        """Get color based on collision penetration depth (heatmap)"""
        if penetration_depth <= 2.0:
            return self.COLLISION_COLORS['green']
        elif penetration_depth <= 5.0:
            return self.COLLISION_COLORS['yellow']
        elif penetration_depth <= 10.0:
            return self.COLLISION_COLORS['orange']
        else:
            return self.COLLISION_COLORS['red']
    
    def parse_robot_capsules(self, packet_data: bytes) -> Optional[o3d.geometry.PointCloud]:
        """Parse robot capsules packet and create point cloud"""
        try:
            # Parse header: uint32_t capsule_count
            if len(packet_data) < 4:
                return None
            
            capsule_count = struct.unpack('I', packet_data[:4])[0]
            if capsule_count == 0:
                return None
            
            # Parse capsules (7 floats each: start_xyz, end_xyz, radius)
            capsules = []
            offset = 4
            for i in range(capsule_count):
                if offset + 28 > len(packet_data):  # 7 * 4 bytes
                    break
                
                capsule_data = struct.unpack('fffffff', packet_data[offset:offset+28])
                capsule = CapsuleData(
                    start=np.array([capsule_data[0], capsule_data[1], capsule_data[2]]),
                    end=np.array([capsule_data[3], capsule_data[4], capsule_data[5]]),
                    radius=capsule_data[6]
                )
                
                capsules.append(capsule)
                offset += 28
            
            # Generate point cloud from capsules
            all_points = []
            for capsule in capsules:
                points = self.sample_capsule_points(capsule, self.CAPSULE_POINTS_PER_SEGMENT)
                all_points.extend(points)
            
            if not all_points:
                return None
            
            # Create Open3D point cloud
            point_cloud = o3d.geometry.PointCloud()
            point_cloud.points = o3d.utility.Vector3dVector(np.array(all_points))
            
            # Apply robot color
            colors = np.tile(self.ROBOT_COLOR, (len(all_points), 1))
            point_cloud.colors = o3d.utility.Vector3dVector(colors)
            
            return point_cloud
            
        except Exception as e:
            print(f"⚠️  Robot capsules parse error: {e}")
            return None
    
    def parse_human_pose(self, packet_data: bytes) -> Optional[o3d.geometry.PointCloud]:
        """Parse human pose packet and create point cloud"""
        try:
            # Parse header: uint32_t joint_count
            if len(packet_data) < 4:
                return None
            
            joint_count = struct.unpack('I', packet_data[:4])[0]
            if joint_count == 0:
                return None
            
            # Parse joints (3 floats each: x, y, z)
            joints = []
            offset = 4
            for i in range(min(joint_count, 24)):  # Max 24 joints
                if offset + 12 > len(packet_data):  # 3 * 4 bytes
                    break
                
                joint_data = struct.unpack('fff', packet_data[offset:offset+12])
                joint_star = np.array([joint_data[0], joint_data[1], joint_data[2]])
                
                # Transform from STAR to collision coordinates
                joint_collision = self.transform_star_to_collision(joint_star)
                joints.append(joint_collision)
                offset += 12
            
            if not joints:
                return None
            
            # Create Open3D point cloud for joints
            point_cloud = o3d.geometry.PointCloud()
            point_cloud.points = o3d.utility.Vector3dVector(np.array(joints))
            
            # Apply human color
            colors = np.tile(self.HUMAN_COLOR, (len(joints), 1))
            point_cloud.colors = o3d.utility.Vector3dVector(colors)
            
            return point_cloud
            
        except Exception as e:
            print(f"⚠️  Human pose parse error: {e}")
            return None
    
    def parse_human_vertices_batch(self, vertex_data: bytes, frame_id: int) -> Optional[int]:
        """
        Parse human vertex batch and accumulate for complete mesh
        Returns the number of vertices in this batch, or None if parsing failed
        """
        try:
            # Parse HumanVerticesPacket header
            if len(vertex_data) < 12:
                return None
            
            header = struct.unpack('III', vertex_data[:12])  # vertex_count, batch_index, total_batches
            vertex_count = header[0]
            batch_index = header[1]
            total_batches = header[2]
            
            if vertex_count == 0:
                return 0
            
            # Check available data size
            available_data = len(vertex_data) - 12  # Minus header
            expected_data = vertex_count * 12  # 3 floats * 4 bytes each
            
            if available_data < expected_data:
                vertex_count = available_data // 12
            
            # Parse vertices (3 floats each: x, y, z)
            vertices = []
            offset = 12
            skipped_vertices = 0
            
            for i in range(vertex_count):
                if offset + 12 > len(vertex_data):  # 3 * 4 bytes
                    break
                
                try:
                    vertex_data_parsed = struct.unpack('fff', vertex_data[offset:offset+12])
                    vertex_star = np.array([vertex_data_parsed[0], vertex_data_parsed[1], vertex_data_parsed[2]])
                    
                    # Check for valid vertex data before transformation
                    if not np.isfinite(vertex_star).all():
                        skipped_vertices += 1
                        offset += 12
                        continue
                    
                    # Transform from STAR to collision coordinates
                    vertex_collision = self.transform_star_to_collision(vertex_star)
                    
                    # Check for valid transformation result
                    if not np.isfinite(vertex_collision).all():
                        skipped_vertices += 1
                        offset += 12
                        continue
                    
                    vertices.append(vertex_collision)
                    
                except struct.error:
                    break
                
                offset += 12
            
            if not vertices:
                return 0
            
            # Add to accumulator
            if frame_id not in self.vertex_accumulators:
                self.vertex_accumulators[frame_id] = VertexBatchAccumulator(
                    frame_id=frame_id,
                    total_batches=total_batches,
                    received_batches={}
                )
            
            accumulator = self.vertex_accumulators[frame_id]
            batch_complete = accumulator.add_batch(batch_index, vertices)
            
            # OPTIMIZED: Only log completion, not every batch
            if batch_complete:
                print(f"      🎯 All {total_batches} vertex batches received for frame {frame_id}")
            
            # Clean up old accumulators
            self._cleanup_old_accumulators(frame_id)
            
            return len(vertices)
            
        except Exception as e:
            print(f"⚠️  Human vertices batch parse error: {e}")
            return None
    
    def get_complete_human_vertices(self, frame_id: int) -> Optional[o3d.geometry.PointCloud]:
        """
        Get complete human vertex point cloud if all batches received for this frame
        """
        if frame_id not in self.vertex_accumulators:
            return None
        
        accumulator = self.vertex_accumulators[frame_id]
        if not accumulator.complete:
            return None
        
        # Get all vertices combined
        all_vertices = accumulator.get_complete_vertices()
        if all_vertices is None:
            return None
        
        # OPTIMIZED: Only log final result
        print(f"      🎯 Creating complete human mesh: {len(all_vertices)} vertices (frame {frame_id})")
        
        # Create Open3D point cloud
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(all_vertices)
        
        # Apply human color
        colors = np.tile(self.HUMAN_COLOR, (len(all_vertices), 1))
        point_cloud.colors = o3d.utility.Vector3dVector(colors)
        
        # Clean up this accumulator
        del self.vertex_accumulators[frame_id]
        
        return point_cloud
    
    def _cleanup_old_accumulators(self, current_frame_id: int):
        """Remove old vertex accumulators to prevent memory leaks"""
        frames_to_remove = []
        for frame_id in self.vertex_accumulators:
            if frame_id < current_frame_id - self.max_cached_frames:
                frames_to_remove.append(frame_id)
        
        for frame_id in frames_to_remove:
            del self.vertex_accumulators[frame_id]
    
    def parse_collision_contacts(self, packet_data: bytes) -> Optional[o3d.geometry.PointCloud]:
        """Parse collision contacts packet and create point cloud with heatmap"""
        try:
            # Parse header: contact_count, has_collision, max_penetration_depth
            if len(packet_data) < 12:
                return None
            
            # Try different parsing approaches for debugging
            try:
                # Most likely format based on C++ struct
                header = struct.unpack('IBBf', packet_data[:10])  # uint32, uint8, padding, float
                contact_count = header[0]
                has_collision = header[1]
                max_penetration = header[3]
                
                if contact_count == 0:
                    return None
                
                # Parse contacts (would need contact structure from C++)
                # For now, create dummy visualization if collision detected
                if has_collision:
                    # Create a small point cloud at origin to show collision detected
                    points = np.array([[0, 0, 0]])
                    point_cloud = o3d.geometry.PointCloud()
                    point_cloud.points = o3d.utility.Vector3dVector(points)
                    
                    # Color based on penetration depth
                    color = self.get_collision_color(max_penetration)
                    point_cloud.colors = o3d.utility.Vector3dVector([color])
                    
                    return point_cloud
                
            except struct.error:
                pass
            
            return None
            
        except Exception as e:
            print(f"⚠️  Collision contacts parse error: {e}")
            return None
    
    def parse_frame(self, frame: PacketBuffer) -> Dict[str, o3d.geometry.PointCloud]:
        """Parse complete frame and return all geometries"""
        geometries = {}
        
        # OPTIMIZED: Minimal frame logging
        print(f"\n🔍 Frame {frame.frame_id}")
        
        # EMERGENCY COMPATIBILITY: Handle both old and new packet buffer formats
        def safe_get_packets(packet_type):
            """Get packets handling both old (single) and new (list) formats"""
            if hasattr(frame, 'packets') and packet_type in frame.packets:
                data = frame.packets[packet_type]
                if isinstance(data, list):
                    return data  # New format
                else:
                    return [data]  # Old format - wrap in list
            return []
        
        # Parse robot capsules (single packet expected)
        robot_packets = safe_get_packets(PacketType.ROBOT_CAPSULES)
        if robot_packets:
            robot_data = robot_packets[-1]  # Get latest
            if len(robot_data) > 4:  # Need at least header
                robot_pc = self.parse_robot_capsules(robot_data)
                if robot_pc:
                    print(f"   ✅ Robot: {len(robot_pc.points)} points")
                    geometries['robot'] = robot_pc
        
        # Parse human data (handle multiple vertex batch packets)
        human_packets = safe_get_packets(PacketType.HUMAN_POSE)
        if human_packets:
            # Classify packets as joints vs vertex batches
            joint_packets = []
            vertex_packets = []
            
            for packet in human_packets:
                if len(packet) <= 200:
                    joint_packets.append(packet)
                else:
                    vertex_packets.append(packet)
            
            # Process vertex batches if we have them
            if vertex_packets:
                # Clear any existing accumulator for this frame
                if frame.frame_id in self.vertex_accumulators:
                    del self.vertex_accumulators[frame.frame_id]
                
                total_vertices_processed = 0
                
                # Process each vertex batch packet
                for vertex_data in vertex_packets:
                    vertex_count = self.parse_human_vertices_batch(vertex_data, frame.frame_id)
                    if vertex_count is not None and vertex_count > 0:
                        total_vertices_processed += vertex_count
                
                # Check if we have complete mesh now
                complete_vertices_pc = self.get_complete_human_vertices(frame.frame_id)
                if complete_vertices_pc:
                    print(f"   🎯 Complete Human Mesh: {len(complete_vertices_pc.points)} vertices")
                    geometries['human'] = complete_vertices_pc
                else:
                    # Create mesh from whatever we have
                    if frame.frame_id in self.vertex_accumulators:
                        accumulator = self.vertex_accumulators[frame.frame_id]
                        if len(accumulator.received_batches) > 0:
                            partial_vertices = []
                            for batch_vertices in accumulator.received_batches.values():
                                partial_vertices.extend(batch_vertices)
                            
                            if partial_vertices:
                                print(f"   🔧 Partial Human Mesh: {len(partial_vertices)} vertices from {len(accumulator.received_batches)} batches")
                                
                                point_cloud = o3d.geometry.PointCloud()
                                point_cloud.points = o3d.utility.Vector3dVector(np.array(partial_vertices))
                                colors = np.tile(self.HUMAN_COLOR, (len(partial_vertices), 1))
                                point_cloud.colors = o3d.utility.Vector3dVector(colors)
                                
                                geometries['human'] = point_cloud
            
            # Process joint packets if we have them and no vertex data
            elif joint_packets:
                joint_data = joint_packets[-1]  # Use latest joint packet
                human_pc = self.parse_human_pose(joint_data)
                if human_pc:
                    print(f"   ✅ Human Joints: {len(human_pc.points)} points")
                    geometries['human'] = human_pc
            
            # EMERGENCY FALLBACK: Try to show anything we have
            else:
                # Check if we have any stored vertex data from previous frames
                for stored_frame_id in list(self.vertex_accumulators.keys())[-3:]:  # Check last 3 frames
                    accumulator = self.vertex_accumulators[stored_frame_id]
                    if len(accumulator.received_batches) > 0:
                        partial_vertices = []
                        for batch_vertices in accumulator.received_batches.values():
                            partial_vertices.extend(batch_vertices)
                        
                        if partial_vertices:
                            print(f"   🔧 EMERGENCY: Using stored vertices from frame {stored_frame_id}: {len(partial_vertices)} vertices")
                            
                            point_cloud = o3d.geometry.PointCloud()
                            point_cloud.points = o3d.utility.Vector3dVector(np.array(partial_vertices))
                            colors = np.tile(self.HUMAN_COLOR, (len(partial_vertices), 1))
                            point_cloud.colors = o3d.utility.Vector3dVector(colors)
                            
                            geometries['human'] = point_cloud
                            break
        
        # Parse collision contacts (single packet expected)
        collision_packets = safe_get_packets(PacketType.COLLISION_CONTACTS)
        if collision_packets:
            collision_data = collision_packets[-1]  # Get latest
            if len(collision_data) > 12:  # Need at least header
                collision_pc = self.parse_collision_contacts(collision_data)
                if collision_pc:
                    print(f"   ✅ Collision: {len(collision_pc.points)} points")
                    geometries['collision'] = collision_pc
        
        # OPTIMIZED: Simple summary instead of detailed breakdown
        print(f"   📦 Total geometries: {len(geometries)}")
        return geometries


# Test function
if __name__ == "__main__":
    print("🧪 Testing DataParser...")
    
    parser = DataParser()
    
    # Test coordinate transformation
    star_point = np.array([0.5, 1.7, 0.2])  # 0.5m right, 1.7m up, 0.2m forward
    collision_point = parser.transform_star_to_collision(star_point)
    print(f"STAR: {star_point} → Collision: {collision_point}")
    
    # Test capsule point sampling
    capsule = CapsuleData(
        start=np.array([0, 0, 0]),
        end=np.array([100, 0, 0]),
        radius=25.0
    )
    points = parser.sample_capsule_points(capsule, 25)
    print(f"Capsule sampling: {len(points)} points generated")
    
    print("✅ DataParser tests complete")