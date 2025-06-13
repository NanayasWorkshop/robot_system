#!/usr/bin/env python3
"""
Data Parser - Transform C++ collision data to Open3D geometries
Handles coordinate transformation and point cloud generation
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
                
                # DEBUG: Print first capsule coordinates
                if i == 0:
                    print(f"      Robot capsule 0: start=({capsule.start[0]:.1f}, {capsule.start[1]:.1f}, {capsule.start[2]:.1f}) "
                          f"end=({capsule.end[0]:.1f}, {capsule.end[1]:.1f}, {capsule.end[2]:.1f}) radius={capsule.radius:.1f}")
                
                capsules.append(capsule)
                offset += 28
            
            # Generate point cloud from capsules
            all_points = []
            for capsule in capsules:
                points = self.sample_capsule_points(capsule, self.CAPSULE_POINTS_PER_SEGMENT)
                all_points.extend(points)
            
            if not all_points:
                return None
            
            # DEBUG: Print point cloud bounds
            points_array = np.array(all_points)
            print(f"      Robot points range: X=[{points_array[:, 0].min():.1f}, {points_array[:, 0].max():.1f}] "
                  f"Y=[{points_array[:, 1].min():.1f}, {points_array[:, 1].max():.1f}] "
                  f"Z=[{points_array[:, 2].min():.1f}, {points_array[:, 2].max():.1f}]")
            
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
                
                # DEBUG: Print first joint before and after transformation
                if i == 0:
                    print(f"      Human joint 0 STAR: ({joint_star[0]:.3f}, {joint_star[1]:.3f}, {joint_star[2]:.3f})")
                
                # Transform from STAR to collision coordinates
                joint_collision = self.transform_star_to_collision(joint_star)
                
                if i == 0:
                    print(f"      Human joint 0 Collision: ({joint_collision[0]:.1f}, {joint_collision[1]:.1f}, {joint_collision[2]:.1f})")
                
                joints.append(joint_collision)
                offset += 12
            
            if not joints:
                return None
            
            # DEBUG: Print human point cloud bounds
            joints_array = np.array(joints)
            print(f"      Human points range: X=[{joints_array[:, 0].min():.1f}, {joints_array[:, 0].max():.1f}] "
                  f"Y=[{joints_array[:, 1].min():.1f}, {joints_array[:, 1].max():.1f}] "
                  f"Z=[{joints_array[:, 2].min():.1f}, {joints_array[:, 2].max():.1f}]")
            
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
    
    def parse_human_vertices(self, vertex_data: bytes) -> Optional[o3d.geometry.PointCloud]:
        """Parse human vertex data (STAR mesh vertices)"""
        try:
            # This would be called if we receive HumanVerticesPacket
            # For now, we use the joint-based representation
            # Future: Parse actual STAR mesh vertices here
            return None
            
        except Exception as e:
            print(f"⚠️  Human vertices parse error: {e}")
            return None
    
    def parse_collision_contacts(self, packet_data: bytes) -> Optional[o3d.geometry.PointCloud]:
        """Parse collision contacts packet and create point cloud with heatmap"""
        try:
            # Parse header: contact_count, has_collision, max_penetration_depth
            if len(packet_data) < 12:
                return None
            
            header = struct.unpack('BBff', packet_data[:12])
            contact_count = header[0]
            has_collision = header[1]
            max_penetration = header[2]
            
            if not has_collision or contact_count == 0:
                return None
            
            # Parse contacts (7 floats each: contact_xyz, normal_xyz, depth, capsule_index)
            contacts = []
            offset = 12
            for i in range(contact_count):
                if offset + 28 > len(packet_data):  # 7 * 4 bytes
                    break
                
                contact_data = struct.unpack('fffffff', packet_data[offset:offset+28])
                contact = ContactData(
                    contact_point=np.array([contact_data[0], contact_data[1], contact_data[2]]),
                    normal=np.array([contact_data[3], contact_data[4], contact_data[5]]),
                    penetration_depth=contact_data[6],
                    robot_capsule_index=int(contact_data[7])
                )
                contacts.append(contact)
                offset += 28
            
            if not contacts:
                return None
            
            # Create point cloud with heatmap colors
            points = []
            colors = []
            
            for contact in contacts:
                points.append(contact.contact_point)
                color = self.get_collision_color(contact.penetration_depth)
                colors.append(color)
            
            # Create Open3D point cloud
            point_cloud = o3d.geometry.PointCloud()
            point_cloud.points = o3d.utility.Vector3dVector(np.array(points))
            point_cloud.colors = o3d.utility.Vector3dVector(np.array(colors))
            
            return point_cloud
            
        except Exception as e:
            print(f"⚠️  Collision contacts parse error: {e}")
            return None
    
    def parse_frame(self, frame: PacketBuffer) -> Dict[str, o3d.geometry.PointCloud]:
        """Parse complete frame and return all geometries"""
        geometries = {}
        
        print(f"\n🔍 Frame {frame.frame_id} packets: {list(frame.packets.keys())}")
        
        # Parse robot capsules
        if PacketType.ROBOT_CAPSULES in frame.packets:
            print(f"   Parsing robot capsules: {len(frame.packets[PacketType.ROBOT_CAPSULES])} bytes")
            robot_pc = self.parse_robot_capsules(frame.packets[PacketType.ROBOT_CAPSULES])
            if robot_pc:
                print(f"   ✅ Robot: {len(robot_pc.points)} points")
                geometries['robot'] = robot_pc
            else:
                print(f"   ❌ Robot: parsing failed")
        
        # Parse human pose
        if PacketType.HUMAN_POSE in frame.packets:
            print(f"   Parsing human pose: {len(frame.packets[PacketType.HUMAN_POSE])} bytes")
            human_pc = self.parse_human_pose(frame.packets[PacketType.HUMAN_POSE])
            if human_pc:
                print(f"   ✅ Human: {len(human_pc.points)} points")
                geometries['human'] = human_pc
            else:
                print(f"   ❌ Human: parsing failed")
        
        # Parse collision contacts
        if PacketType.COLLISION_CONTACTS in frame.packets:
            print(f"   Parsing collision: {len(frame.packets[PacketType.COLLISION_CONTACTS])} bytes")
            collision_pc = self.parse_collision_contacts(frame.packets[PacketType.COLLISION_CONTACTS])
            if collision_pc:
                print(f"   ✅ Collision: {len(collision_pc.points)} points")
                geometries['collision'] = collision_pc
            else:
                print(f"   ❌ Collision: parsing failed")
        
        print(f"   📦 Total geometries created: {len(geometries)}")
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