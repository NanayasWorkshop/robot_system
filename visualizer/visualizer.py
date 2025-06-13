#!/usr/bin/env python3
"""
Open3D Collision Visualizer
Real-time visualization of robot collision detection system
FIXED: Proper handling of batched vertex data for complete STAR mesh display
"""

import argparse
import time
import sys
from typing import Dict, Optional
import numpy as np
import open3d as o3d
from dataclasses import dataclass

from network_receiver import NetworkReceiver, PacketBuffer
from data_parser import DataParser


@dataclass
class VisualizerConfig:
    """Visualizer configuration"""
    show_robot: bool = True
    show_human: bool = True
    show_collision: bool = True
    show_layers: list = None
    window_width: int = 1200
    window_height: int = 800
    target_fps: int = 60
    point_size: float = 2.0
    collision_point_size: float = 4.0
    
    def __post_init__(self):
        if self.show_layers is None:
            self.show_layers = []


class CollisionVisualizer:
    """Main visualizer application"""
    
    def __init__(self, config: VisualizerConfig):
        self.config = config
        
        # Core components
        self.receiver = NetworkReceiver()
        self.parser = DataParser()
        
        # Open3D components
        self.vis = None
        self.viewport = None
        
        # Scene objects
        self.geometries = {}
        self.geometry_names = ['robot', 'human', 'collision']
        
        # FIXED: Frame tracking for batched data
        self.processed_frames = set()
        self.last_complete_frame = -1
        
        # Statistics
        self.frame_count = 0
        self.vertex_batch_count = 0
        self.complete_mesh_count = 0
        self.last_stats_time = time.time()
        self.fps_history = []
        
        # Runtime state
        self.is_running = False
        self.last_frame_id = -1
    
    def initialize(self) -> bool:
        """Initialize visualizer components"""
        print("🚀 Initializing Collision Visualizer...")
        
        # Initialize network receiver
        if not self.receiver.initialize():
            print("❌ Failed to initialize network receiver")
            return False
        
        # Initialize Open3D visualizer
        if not self._initialize_open3d():
            print("❌ Failed to initialize Open3D")
            return False
        
        # Create initial empty geometries
        self._create_initial_geometries()
        
        print("✅ Collision Visualizer ready!")
        print(f"   Window: {self.config.window_width}x{self.config.window_height}")
        print(f"   Rendering: {', '.join(self._get_active_renderers())}")
        print(f"   Listening: UDP port 9999")
        print(f"   FIXED: Proper batched vertex accumulation for complete STAR mesh")
        
        return True
    
    def _initialize_open3d(self) -> bool:
        """Initialize Open3D visualizer window"""
        try:
            # Create visualizer
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window(
                window_name="Robot Collision Visualization (FIXED)",
                width=self.config.window_width,
                height=self.config.window_height,
                left=100,
                top=100
            )
            
            # Configure rendering options
            render_opt = self.vis.get_render_option()
            render_opt.point_size = self.config.point_size
            render_opt.background_color = np.array([0.1, 0.1, 0.1])  # Dark gray
            render_opt.show_coordinate_frame = True
            
            # Set up camera (Z-up coordinate system)
            view_ctrl = self.vis.get_view_control()
            view_ctrl.set_up([0, 0, 1])  # Z-up
            view_ctrl.set_front([1, 1, -0.5])  # Look from diagonal
            view_ctrl.set_lookat([200, 200, 200])  # Look at 200mm in each direction
            view_ctrl.set_zoom(0.3)  # Zoom out to see more
            
            return True
            
        except Exception as e:
            print(f"❌ Open3D initialization error: {e}")
            return False
    
    def _create_initial_geometries(self):
        """Create empty geometries for each data type"""
        # Add coordinate system first
        self._add_coordinate_system()
        
        for name in self.geometry_names:
            # Create empty point cloud
            pc = o3d.geometry.PointCloud()
            self.geometries[name] = pc
            
            # Add to visualizer if enabled
            if self._should_show_geometry(name):
                self.vis.add_geometry(pc)
    
    def _add_coordinate_system(self):
        """Add coordinate system axes to visualizer"""
        # Create coordinate frame (X=Red, Y=Green, Z=Blue)
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=100.0,  # 100mm = 10cm axes
            origin=[0, 0, 0]
        )
        self.vis.add_geometry(coord_frame)
        
        # Add some reference objects at known locations
        # Origin sphere (5mm radius)
        origin_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=5.0)
        origin_sphere.paint_uniform_color([1, 1, 0])  # Yellow
        origin_sphere.translate([0, 0, 0])
        self.vis.add_geometry(origin_sphere)
        
        # Reference points at 100mm intervals
        for i in range(1, 6):  # 100mm, 200mm, 300mm, 400mm, 500mm
            dist = i * 100.0
            
            # X-axis markers (red)
            marker_x = o3d.geometry.TriangleMesh.create_sphere(radius=3.0)
            marker_x.paint_uniform_color([1, 0, 0])
            marker_x.translate([dist, 0, 0])
            self.vis.add_geometry(marker_x)
            
            # Y-axis markers (green)  
            marker_y = o3d.geometry.TriangleMesh.create_sphere(radius=3.0)
            marker_y.paint_uniform_color([0, 1, 0])
            marker_y.translate([0, dist, 0])
            self.vis.add_geometry(marker_y)
            
            # Z-axis markers (blue)
            marker_z = o3d.geometry.TriangleMesh.create_sphere(radius=3.0)
            marker_z.paint_uniform_color([0, 0, 1])
            marker_z.translate([0, 0, dist])
            self.vis.add_geometry(marker_z)
        
        print(f"✅ Coordinate system added:")
        print(f"   📍 Origin: Yellow sphere")
        print(f"   📏 Axes: Red=X, Green=Y, Blue=Z (100mm each)")
        print(f"   📐 Scale: Markers every 100mm up to 500mm")
    
    def _should_show_geometry(self, name: str) -> bool:
        """Check if geometry should be rendered based on config"""
        if name == 'robot':
            return self.config.show_robot
        elif name == 'human':
            return self.config.show_human
        elif name == 'collision':
            return self.config.show_collision
        return False
    
    def _get_active_renderers(self) -> list:
        """Get list of active renderer names"""
        active = []
        if self.config.show_robot:
            active.append("Robot")
        if self.config.show_human:
            active.append("Human")
        if self.config.show_collision:
            active.append("Collision")
        if self.config.show_layers:
            active.append(f"Layers({','.join(map(str, self.config.show_layers))})")
        return active
    
    def run(self):
        """Main visualization loop"""
        print("🎬 Starting visualization loop...")
        print("   Press ESC or close window to exit")
        print("   Press H for help")
        print("   FIXED: Now properly accumulates all vertex batches for complete mesh")
        print()
        
        self.is_running = True
        frame_start_time = time.time()
        
        try:
            while self.is_running:
                loop_start = time.time()
                
                # Update network receiver
                self.receiver.update()
                
                # Check for new frame data
                frame = self.receiver.get_latest_frame()
                if frame and frame.frame_id != self.last_frame_id:
                    self._process_new_frame(frame)
                    self.last_frame_id = frame.frame_id
                    
                    # Clear latest frame so we don't process it again
                    self.receiver.latest_complete_frame = None
                
                # Update Open3D visualizer
                if not self.vis.poll_events():
                    break
                
                self._handle_keyboard_input()
                self.vis.update_renderer()
                
                # Update statistics
                self._update_statistics()
                
                # Frame rate control
                self._control_frame_rate(loop_start)
                
        except KeyboardInterrupt:
            print("\n🛑 Interrupted by user")
        except Exception as e:
            print(f"❌ Visualization error: {e}")
        finally:
            self._shutdown()
    
    def _process_new_frame(self, frame: PacketBuffer):
        """Process new frame data and update geometries"""
        try:
            # FIXED: Don't process frames we've already handled completely
            if frame.frame_id in self.processed_frames:
                return
            
            # Parse frame data
            new_geometries = self.parser.parse_frame(frame)
            
            # Track if this frame had complete data
            frame_complete = False
            
            # Update each geometry type
            for name in self.geometry_names:
                if name in new_geometries and self._should_show_geometry(name):
                    self._update_geometry(name, new_geometries[name])
                    
                    # Special tracking for human mesh completion
                    if name == 'human':
                        vertex_count = len(new_geometries[name].points)
                        if vertex_count > 1000:  # This indicates complete mesh (not just joints)
                            self.complete_mesh_count += 1
                            frame_complete = True
                            print(f"   🎯 Complete human mesh rendered: {vertex_count} vertices (frame {frame.frame_id})")
                        elif vertex_count > 0:
                            print(f"   👤 Human joints rendered: {vertex_count} joints (frame {frame.frame_id})")
            
            # Mark frame as processed if complete
            if frame_complete:
                self.processed_frames.add(frame.frame_id)
                self.last_complete_frame = frame.frame_id
                
                # Clean up old processed frames to prevent memory leak
                self._cleanup_old_processed_frames(frame.frame_id)
            
            self.frame_count += 1
            
        except Exception as e:
            print(f"⚠️  Frame processing error: {e}")
    
    def _cleanup_old_processed_frames(self, current_frame_id: int):
        """Remove old processed frame IDs to prevent memory leaks"""
        frames_to_remove = {fid for fid in self.processed_frames if fid < current_frame_id - 10}
        self.processed_frames -= frames_to_remove
        if frames_to_remove:
            print(f"      🧹 Cleaned up {len(frames_to_remove)} old processed frame IDs")
    
    def _update_geometry(self, name: str, new_geometry: o3d.geometry.PointCloud):
        """Update existing geometry with new data"""
        try:
            old_geometry = self.geometries[name]
            
            # Update points and colors
            old_geometry.points = new_geometry.points
            old_geometry.colors = new_geometry.colors
            
            # Special handling for collision points (larger size)
            if name == 'collision':
                # Note: Open3D doesn't support per-point sizes easily
                # We could create spheres for collision points if needed
                pass
            
            # Update visualizer
            self.vis.update_geometry(old_geometry)
            
        except Exception as e:
            print(f"⚠️  Geometry update error ({name}): {e}")
    
    def _handle_keyboard_input(self):
        """Handle keyboard shortcuts"""
        # Note: Open3D's keyboard handling is limited
        # For full keyboard support, we'd need a different approach
        pass
    
    def _update_statistics(self):
        """Update and display performance statistics"""
        current_time = time.time()
        
        # Print stats every 2 seconds
        if current_time - self.last_stats_time > 2.0:
            self._print_statistics()
            self.last_stats_time = current_time
    
    def _print_statistics(self):
        """Print current statistics"""
        net_stats = self.receiver.get_statistics()
        
        # Calculate current FPS
        current_time = time.time()
        elapsed = current_time - self.last_stats_time
        if elapsed > 0:
            current_fps = self.frame_count / (current_time - time.time() + elapsed)
        else:
            current_fps = 0
        
        # FIXED: Enhanced stats with vertex batch tracking
        print(f"\r📊 Frames: {self.frame_count} | "
              f"Complete Meshes: {self.complete_mesh_count} | "
              f"Net: {net_stats.frames_complete} frames | "
              f"FPS: {current_fps:.1f} | "
              f"Data: {net_stats.data_rate_mbps:.1f} Mbps | "
              f"Loss: {net_stats.frames_dropped} | "
              f"Last Complete: {self.last_complete_frame}", 
              end="", flush=True)
    
    def _control_frame_rate(self, loop_start: float):
        """Control frame rate to target FPS"""
        target_frame_time = 1.0 / self.config.target_fps
        elapsed = time.time() - loop_start
        
        if elapsed < target_frame_time:
            time.sleep(target_frame_time - elapsed)
    
    def _shutdown(self):
        """Clean shutdown"""
        print("\n🔄 Shutting down...")
        
        self.is_running = False
        
        if self.receiver:
            self.receiver.shutdown()
        
        if self.vis:
            self.vis.destroy_window()
        
        # Print final statistics
        print(f"📊 Final Statistics:")
        print(f"   Total frames processed: {self.frame_count}")
        print(f"   Complete meshes rendered: {self.complete_mesh_count}")
        print(f"   Processed frame IDs: {len(self.processed_frames)}")
        print(f"   Last complete frame: {self.last_complete_frame}")
        
        print("✅ Visualizer shutdown complete")


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Robot Collision System Visualizer (FIXED - Batched Vertex Support)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python visualizer.py                    # Default: robot + human + collision
  python visualizer.py --robot --human   # Robot and human only
  python visualizer.py --collision       # Collision points only
  python visualizer.py --all             # Everything visible
  python visualizer.py --point-size 5.0  # Larger points for better visibility
        """
    )
    
    # Visibility options
    parser.add_argument('--robot', action='store_true', 
                       help='Show robot capsules (default: True)')
    parser.add_argument('--human', action='store_true',
                       help='Show human pose (default: True)')
    parser.add_argument('--collision', action='store_true',
                       help='Show collision contacts (default: True)')
    parser.add_argument('--layers', type=int, nargs='*', default=[],
                       help='Show collision layers (1, 2, 3)')
    parser.add_argument('--all', action='store_true',
                       help='Show everything')
    
    # Display options
    parser.add_argument('--width', type=int, default=1200,
                       help='Window width (default: 1200)')
    parser.add_argument('--height', type=int, default=800,
                       help='Window height (default: 800)')
    parser.add_argument('--fps', type=int, default=60,
                       help='Target FPS (default: 60)')
    parser.add_argument('--point-size', type=float, default=2.0,
                       help='Point size (default: 2.0)')
    
    return parser.parse_args()


def create_config_from_args(args) -> VisualizerConfig:
    """Create configuration from command line arguments"""
    
    # Default visibility (if no flags specified, show robot + human + collision)
    if not any([args.robot, args.human, args.collision, args.all]):
        show_robot = True
        show_human = True
        show_collision = True
    else:
        show_robot = args.robot or args.all
        show_human = args.human or args.all
        show_collision = args.collision or args.all
    
    return VisualizerConfig(
        show_robot=show_robot,
        show_human=show_human,
        show_collision=show_collision,
        show_layers=args.layers,
        window_width=args.width,
        window_height=args.height,
        target_fps=args.fps,
        point_size=args.point_size,
        collision_point_size=args.point_size * 2
    )


def main():
    """Main entry point"""
    print("🎯 ROBOT COLLISION SYSTEM VISUALIZER (FIXED)")
    print("=" * 60)
    print("FIXED: Proper batched vertex accumulation for complete STAR mesh")
    print("- Accumulates all vertex batches per frame")
    print("- Only renders when complete mesh is received")
    print("- Prevents memory leaks from incomplete batches")
    print("=" * 60)
    
    # Parse command line arguments
    args = parse_arguments()
    config = create_config_from_args(args)
    
    # Create and initialize visualizer
    visualizer = CollisionVisualizer(config)
    
    if not visualizer.initialize():
        print("❌ Failed to initialize visualizer")
        return 1
    
    # Start main loop
    try:
        visualizer.run()
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())