#!/usr/bin/env python3
"""
Debug Frame Capture - Plotly 3D Visualization
Captures one frame and visualizes as proper 3D primitives
"""

import plotly.graph_objects as go
import plotly.express as px
import numpy as np
from network_receiver import NetworkReceiver, PacketType
from data_parser import DataParser
import webbrowser
import tempfile
import os


class FrameDebugger:
    """Capture and visualize one frame in Plotly 3D"""
    
    def __init__(self):
        self.receiver = NetworkReceiver()
        self.parser = DataParser()
        self.fig = go.Figure()
        
    def capture_frame(self):
        """Capture first valid complete frame"""
        print("🔍 Capturing frame for debug...")
        
        if not self.receiver.initialize():
            print("❌ Failed to initialize receiver")
            return None
            
        try:
            frames_checked = 0
            while frames_checked < 100:  # Check up to 100 frames
                self.receiver.update()
                frame = self.receiver.get_latest_frame()
                
                if frame:
                    frames_checked += 1
                    print(f"📦 Frame {frame.frame_id}: checking completeness...")
                    
                    # =============================================================================
                    # DEBUG: Check layer packet reception
                    # =============================================================================
                    layer_packets = frame.get_all_packets(PacketType.COLLISION_LAYERS)
                    print(f"   Layer packets received: {len(layer_packets)}")
                    if layer_packets:
                        print(f"   First layer packet size: {len(layer_packets[0])} bytes")
                        if len(layer_packets[0]) > 100:
                            print(f"   ✅ Layer packet looks good (size > 100 bytes)")
                        else:
                            print(f"   ⚠️  Layer packet very small")
                    else:
                        print(f"   ❌ NO layer packets found!")
                    # =============================================================================
                    
                    # Check if frame has good data
                    if (frame.has_packet_type(PacketType.ROBOT_CAPSULES) and
                        frame.has_packet_type(PacketType.HUMAN_POSE) and
                        frame.has_packet_type(PacketType.COLLISION_LAYERS)):
                        
                        print(f"✅ Got complete frame {frame.frame_id}")
                        return frame
                    
        except KeyboardInterrupt:
            print("\n🛑 Interrupted")
        finally:
            self.receiver.shutdown()
            
        print("❌ No complete frame found")
        return None
    
    def create_capsule_mesh(self, start, end, radius, color, name):
        """Create capsule as proper Plotly 3D meshes (cylinder + 2 spheres)"""
        traces = []
        
        # Calculate capsule parameters
        direction = end - start
        length = np.linalg.norm(direction)
        
        if length < 1e-6:
            # Just a sphere if start == end
            traces.append(self.create_sphere_mesh(start, radius, color, f"{name}_sphere"))
            return traces
        
        axis = direction / length
        center = (start + end) / 2
        
        # Create cylinder mesh
        cylinder_trace = self.create_cylinder_mesh(center, axis, length, radius, color, f"{name}_cylinder")
        traces.append(cylinder_trace)
        
        # Create end cap spheres
        start_sphere = self.create_sphere_mesh(start, radius, color, f"{name}_start_sphere")
        end_sphere = self.create_sphere_mesh(end, radius, color, f"{name}_end_sphere")
        traces.append(start_sphere)
        traces.append(end_sphere)
        
        return traces
    
    def create_cylinder_mesh(self, center, axis, length, radius, color, name):
        """Create cylinder mesh using Plotly Mesh3d"""
        # Generate cylinder vertices
        theta = np.linspace(0, 2*np.pi, 16)  # 16 sides
        
        # Create perpendicular vectors
        if abs(axis[2]) < 0.9:
            perp1 = np.cross(axis, np.array([0, 0, 1]))
        else:
            perp1 = np.cross(axis, np.array([1, 0, 0]))
        perp1 = perp1 / np.linalg.norm(perp1)
        perp2 = np.cross(axis, perp1)
        
        vertices = []
        
        # Bottom circle
        for t in theta:
            offset = radius * (np.cos(t) * perp1 + np.sin(t) * perp2)
            vertex = center - (length/2) * axis + offset
            vertices.append(vertex)
        
        # Top circle  
        for t in theta:
            offset = radius * (np.cos(t) * perp1 + np.sin(t) * perp2)
            vertex = center + (length/2) * axis + offset
            vertices.append(vertex)
        
        vertices = np.array(vertices)
        
        # Create triangular faces
        faces = []
        n = len(theta)
        
        # Side faces
        for i in range(n):
            next_i = (i + 1) % n
            # Two triangles per quad
            faces.append([i, next_i, i + n])
            faces.append([next_i, next_i + n, i + n])
        
        faces = np.array(faces)
        
        return go.Mesh3d(
            x=vertices[:, 0], y=vertices[:, 1], z=vertices[:, 2],
            i=faces[:, 0], j=faces[:, 1], k=faces[:, 2],
            color=color,
            opacity=0.8,
            name=name,
            showlegend=False
        )
    
    def create_sphere_mesh(self, center, radius, color, name):
        """Create sphere mesh using Plotly Mesh3d"""
        # Generate sphere vertices using spherical coordinates
        phi = np.linspace(0, 2*np.pi, 12)  # Longitude
        theta = np.linspace(0, np.pi, 8)   # Latitude
        
        vertices = []
        for t in theta:
            for p in phi:
                x = center[0] + radius * np.sin(t) * np.cos(p)
                y = center[1] + radius * np.sin(t) * np.sin(p)
                z = center[2] + radius * np.cos(t)
                vertices.append([x, y, z])
        
        vertices = np.array(vertices)
        
        # Create triangular faces
        faces = []
        n_phi = len(phi)
        n_theta = len(theta)
        
        for i in range(n_theta - 1):
            for j in range(n_phi):
                next_j = (j + 1) % n_phi
                
                # Current row indices
                curr_0 = i * n_phi + j
                curr_1 = i * n_phi + next_j
                
                # Next row indices
                next_0 = (i + 1) * n_phi + j
                next_1 = (i + 1) * n_phi + next_j
                
                # Two triangles per quad
                faces.append([curr_0, curr_1, next_0])
                faces.append([curr_1, next_1, next_0])
        
        faces = np.array(faces)
        
        return go.Mesh3d(
            x=vertices[:, 0], y=vertices[:, 1], z=vertices[:, 2],
            i=faces[:, 0], j=faces[:, 1], k=faces[:, 2],
            color=color,
            opacity=0.8,
            name=name,
            showlegend=False
        )
    
    def create_sphere_points(self, center, radius, num_points):
        """Generate sphere surface points"""
        phi = np.random.uniform(0, 2*np.pi, num_points)
        costheta = np.random.uniform(-1, 1, num_points)
        theta = np.arccos(costheta)
        
        x = center[0] + radius * np.sin(theta) * np.cos(phi)
        y = center[1] + radius * np.sin(theta) * np.sin(phi)
        z = center[2] + radius * np.cos(theta)
        
        return x, y, z
    
    def add_robot_capsules(self, data):
        """Add robot capsules as proper 3D mesh objects"""
        if not data or len(data) < 4:
            return
            
        try:
            import struct
            count = struct.unpack('I', data[:4])[0]
            if count == 0:
                return
            
            offset = 4
            for i in range(count):
                if offset + 28 > len(data):
                    break
                
                values = struct.unpack('fffffff', data[offset:offset+28])
                start = np.array([values[0], values[1], values[2]])
                end = np.array([values[3], values[4], values[5]])
                radius = values[6]
                
                # Create proper capsule mesh objects
                capsule_traces = self.create_capsule_mesh(
                    start, end, radius, 
                    color='steelblue', 
                    name=f'robot_capsule_{i}'
                )
                
                # Add all capsule parts to figure
                for trace in capsule_traces:
                    self.fig.add_trace(trace)
                
                offset += 28
                
        except Exception as e:
            print(f"⚠️ Error parsing robot capsules: {e}")
    
    def add_human_joints(self, data):
        """Add human joints and skeleton"""
        if not data:
            return
            
        joint_spheres, skeleton_lines = self.parser.parse_human_pose(data)
        
        # Add joints
        if joint_spheres and len(joint_spheres.vertices) > 0:
            vertices = np.asarray(joint_spheres.vertices)
            self.fig.add_trace(go.Scatter3d(
                x=vertices[:, 0], y=vertices[:, 1], z=vertices[:, 2],
                mode='markers',
                marker=dict(size=2.4, color='red'),  # Reduced by 70%
                name='Human Joints'
            ))
        
        # Add skeleton lines
        if skeleton_lines and len(skeleton_lines.points) > 0:
            points = np.asarray(skeleton_lines.points)
            lines = np.asarray(skeleton_lines.lines)
            
            # Create line traces
            line_x, line_y, line_z = [], [], []
            for line in lines:
                start_point = points[line[0]]
                end_point = points[line[1]]
                line_x.extend([start_point[0], end_point[0], None])
                line_y.extend([start_point[1], end_point[1], None])
                line_z.extend([start_point[2], end_point[2], None])
            
            self.fig.add_trace(go.Scatter3d(
                x=line_x, y=line_y, z=line_z,
                mode='lines',
                line=dict(color='yellow', width=4),
                name='Skeleton'
            ))
    
    def add_collision_layers(self, data):
        """Add collision layers as proper 3D mesh objects with grouped legend"""
        if not data or len(data) < 4:
            print("❌ VISUALIZER: No layer data or data too small")
            return
            
        try:
            import struct
            offset = 0
            
            print(f"\n=== VISUALIZER DEBUG: PARSING COLLISION LAYERS ===")
            print(f"Layer data size: {len(data)} bytes")
            
            # Track which layers have objects for legend
            layer3_objects = []
            layer2_objects = []
            layer1_objects = []
            
            # Parse Layer 3 count
            if offset + 4 > len(data):
                print("❌ VISUALIZER: Cannot read Layer 3 count")
                return
            
            layer3_count = struct.unpack('I', data[offset:offset+4])[0]
            offset += 4
            print(f"VISUALIZER: Layer 3 count = {layer3_count}")
            
            # Parse Layer 3 primitives (capsules)
            for i in range(min(layer3_count, 16)):  # Max 16 from C++
                if offset + 29 > len(data):  # 7 floats + 1 byte
                    print(f"❌ VISUALIZER: Layer3[{i}] - not enough data")
                    break
                
                values = struct.unpack('fffffffB', data[offset:offset+29])
                start = np.array([values[0], values[1], values[2]])
                end = np.array([values[3], values[4], values[5]])
                radius = values[6]
                is_active = values[7]
                
                # =============================================================================
                # DEBUG: Print what visualizer receives
                # =============================================================================
                print(f"VISUALIZER receives Layer3[{i}]: ({start[0]:.3f}, {start[1]:.3f}, {start[2]:.3f}) to ({end[0]:.3f}, {end[1]:.3f}, {end[2]:.3f}) r={radius:.3f} active={is_active}")
                # =============================================================================
                
                # Create capsule mesh objects
                if radius > 0:
                    color = 'green' if is_active else 'gray'
                    capsule_traces = self.create_capsule_mesh(
                        start, end, radius, 
                        color=color, 
                        name=f'layer3_capsule_{i}'
                    )
                    
                    # Set legend group for all traces in this capsule
                    for j, trace in enumerate(capsule_traces):
                        trace.legendgroup = 'layer3'
                        trace.showlegend = (len(layer3_objects) == 0 and j == 0)  # Only first trace shows in legend
                        if trace.showlegend:
                            trace.name = 'Layer 3 Capsules'
                        self.fig.add_trace(trace)
                    
                    layer3_objects.extend(capsule_traces)
                    print(f"  ✅ Added Layer3[{i}] capsule to visualization")
                else:
                    print(f"  ⚠️  Layer3[{i}] has zero radius, skipping")
                
                offset += 29
            
            print(f"VISUALIZER: Added {len(layer3_objects)} Layer 3 mesh objects")
            
            # Parse Layer 2 count
            if offset + 4 > len(data):
                print("VISUALIZER: Layer 2 parsing complete (no more data)")
                return
            
            layer2_count = struct.unpack('I', data[offset:offset+4])[0]
            offset += 4
            print(f"VISUALIZER: Layer 2 count = {layer2_count}")
            
            # Parse Layer 2 primitives (capsules)
            for i in range(min(layer2_count, 32)):  # Max 32 from C++
                if offset + 29 > len(data):
                    break
                
                values = struct.unpack('fffffffB', data[offset:offset+29])
                start = np.array([values[0], values[1], values[2]])
                end = np.array([values[3], values[4], values[5]])
                radius = values[6]
                is_active = values[7]
                
                if radius > 0:
                    color = 'blue' if is_active else 'darkgray'
                    capsule_traces = self.create_capsule_mesh(
                        start, end, radius, 
                        color=color, 
                        name=f'layer2_capsule_{i}'
                    )
                    
                    # Set legend group for all traces in this capsule
                    for j, trace in enumerate(capsule_traces):
                        trace.legendgroup = 'layer2'
                        trace.showlegend = (len(layer2_objects) == 0 and j == 0)  # Only first trace shows in legend
                        if trace.showlegend:
                            trace.name = 'Layer 2 Capsules'
                        self.fig.add_trace(trace)
                    
                    layer2_objects.extend(capsule_traces)
                
                offset += 29
            
            # Parse Layer 1 count
            if offset + 4 > len(data):
                print("VISUALIZER: Layer 1 parsing complete (no more data)")
                return
            
            layer1_count = struct.unpack('I', data[offset:offset+4])[0]
            offset += 4
            print(f"VISUALIZER: Layer 1 count = {layer1_count}")
            
            # Parse Layer 1 primitives (spheres)
            for i in range(min(layer1_count, 128)):  # Max 128 from C++
                if offset + 17 > len(data):  # 4 floats + 1 byte
                    break
                
                values = struct.unpack('ffffB', data[offset:offset+17])
                center = np.array([values[0], values[1], values[2]])
                radius = values[3]
                is_active = values[4]
                
                if radius > 0:
                    color = 'magenta' if is_active else 'darkgray'
                    sphere_trace = self.create_sphere_mesh(
                        center, radius, 
                        color=color, 
                        name=f'layer1_sphere_{i}'
                    )
                    
                    # Set legend group
                    sphere_trace.legendgroup = 'layer1'
                    sphere_trace.showlegend = (len(layer1_objects) == 0)  # Only first sphere shows in legend
                    if sphere_trace.showlegend:
                        sphere_trace.name = 'Layer 1 Spheres'
                    
                    self.fig.add_trace(sphere_trace)
                    layer1_objects.append(sphere_trace)
                
                offset += 17
            
            print(f"=== END VISUALIZER DEBUG ===")
                
        except Exception as e:
            print(f"❌ VISUALIZER: Error parsing collision layers: {e}")
            import traceback
            traceback.print_exc()
    
    def add_human_vertices(self, vertex_packets, frame_id):
        """Add human mesh vertices"""
        if not vertex_packets:
            return
            
        # Process vertex batches
        all_vertices = []
        for vertex_data in vertex_packets:
            vertex_count = self.parser.parse_human_vertices_batch(vertex_data, frame_id)
            if vertex_count:
                print(f"  Processed {vertex_count} vertices")
        
        # Get complete mesh
        complete_pc = self.parser.get_complete_human_vertices(frame_id)
        if complete_pc and len(complete_pc.points) > 0:
            points = np.asarray(complete_pc.points)
            # Subsample for performance (every 10th point)
            points = points[::10]
            
            self.fig.add_trace(go.Scatter3d(
                x=points[:, 0], y=points[:, 1], z=points[:, 2],
                mode='markers',
                marker=dict(size=1.7, color='wheat', opacity=0.5),  # Increased by 70%
                name='Human Mesh'
            ))
    
    def visualize_frame(self, frame):
        """Create 3D visualization of frame"""
        print(f"🎨 Visualizing frame {frame.frame_id}...")
        
        # Parse all data types
        geometries = self.parser.parse_frame(frame)
        
        # Add robot capsules
        robot_packets = frame.get_all_packets(PacketType.ROBOT_CAPSULES)
        if robot_packets:
            self.add_robot_capsules(robot_packets[-1])
        
        # Add human data
        human_packets = frame.get_all_packets(PacketType.HUMAN_POSE)
        if human_packets:
            joint_packets = [p for p in human_packets if len(p) < 1000]
            vertex_packets = [p for p in human_packets if len(p) >= 1000]
            
            if joint_packets:
                self.add_human_joints(joint_packets[-1])
            
            if vertex_packets:
                self.add_human_vertices(vertex_packets, frame.frame_id)
        
        # Add collision layers
        layer_packets = frame.get_all_packets(PacketType.COLLISION_LAYERS)
        if layer_packets:
            print(f"🎯 Processing collision layers ({len(layer_packets)} packets)")
            self.add_collision_layers(layer_packets[-1])
        else:
            print("❌ No collision layer packets found in frame!")
        
        # Configure layout
        self.fig.update_layout(
            title=f"Collision Debug Frame {frame.frame_id}",
            scene=dict(
                xaxis_title="X (meters)",
                yaxis_title="Y (meters)", 
                zaxis_title="Z (meters)",
                aspectmode='data',
                camera=dict(
                    eye=dict(x=1.5, y=1.5, z=1.5)
                )
            ),
            width=1200,
            height=800
        )
        
        print("✅ Visualization complete")
    
    def show(self):
        """Open in browser"""
        # Save to current directory instead of temp
        html_path = os.path.join(os.getcwd(), f"collision_debug_frame.html")
        self.fig.write_html(html_path)
        
        print(f"🌐 Opening in browser: {html_path}")
        webbrowser.open(f'file://{html_path}')
        
        return html_path


def main():
    """Main debug function"""
    print("🐛 COLLISION FRAME DEBUGGER - Plotly 3D")
    print("=" * 50)
    print("Capturing first valid frame for 3D analysis...")
    
    debugger = FrameDebugger()
    
    # Capture frame
    frame = debugger.capture_frame()
    if not frame:
        print("❌ Failed to capture frame")
        return 1
    
    # Visualize
    debugger.visualize_frame(frame)
    
    # Show
    html_path = debugger.show()
    
    print("\n✅ Debug complete!")
    print(f"HTML saved to: {html_path}")
    print("Use mouse to rotate, zoom, and explore the 3D scene")
    
    return 0


if __name__ == "__main__":
    exit(main())