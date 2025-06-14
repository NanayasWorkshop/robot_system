#!/usr/bin/env python3
"""
Open3D Collision Visualizer
Clean rewrite: STAR-native (Y-up, meters), minimal logging, pure C++ data display
"""

import argparse
import time
import sys
from typing import Dict
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
    window_width: int = 1200
    window_height: int = 800
    target_fps: int = 60
    point_size: float = 2.0


class CollisionVisualizer:
    """Real-time collision visualization - STAR coordinate native"""
    
    def __init__(self, config: VisualizerConfig):
        self.config = config
        
        # Core components
        self.receiver = NetworkReceiver()
        self.parser = DataParser()
        
        # Open3D
        self.vis = None
        self.geometries = {}
        self.geometry_names = ['robot', 'human', 'collision']
        
        # State
        self.is_running = False
        self.last_frame_id = -1
        self.processed_frames = set()
        
        # Stats
        self.frame_count = 0
        self.mesh_count = 0
        self.last_stats_time = time.time()
    
    def initialize(self) -> bool:
        """Initialize all components"""
        print("🚀 Initializing Collision Visualizer (STAR-native)")
        
        # Network
        if not self.receiver.initialize():
            print("❌ Network initialization failed")
            return False
        
        # Open3D
        if not self._init_open3d():
            print("❌ Open3D initialization failed")
            return False
        
        # Geometries
        self._create_geometries()
        
        print("✅ Ready - STAR coordinates (Y-up, meters)")
        print(f"   Showing: {self._get_enabled_features()}")
        return True
    
    def _init_open3d(self) -> bool:
        """Initialize Open3D with STAR coordinate system"""
        try:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window(
                window_name="Robot Collision Visualization - STAR Native (Y-up)",
                width=self.config.window_width,
                height=self.config.window_height
            )
            
            # Rendering
            render_opt = self.vis.get_render_option()
            render_opt.point_size = self.config.point_size
            render_opt.background_color = np.array([0.1, 0.1, 0.1])
            render_opt.show_coordinate_frame = True
            
            # STAR camera setup (Y-up, meters)
            view_ctrl = self.vis.get_view_control()
            view_ctrl.set_up([0, 1, 0])  # Y-up for STAR
            view_ctrl.set_front([1, 0, -0.5])  # Look from front-right
            view_ctrl.set_lookat([0, 1, 0])  # Look at 1m height (standing person)
            view_ctrl.set_zoom(0.5)  # Zoom for meter scale
            
            return True
            
        except Exception as e:
            print(f"❌ Open3D error: {e}")
            return False
    
    def _create_geometries(self):
        """Create coordinate system and empty geometries"""
        # STAR coordinate frame (Y-up, meters)
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.25,  # 25cm axes for meter scale
            origin=[0, 0, 0]
        )
        self.vis.add_geometry(coord_frame)
        
        # Empty geometries for data
        for name in self.geometry_names:
            if self._should_show(name):
                pc = o3d.geometry.PointCloud()
                self.geometries[name] = pc
                self.vis.add_geometry(pc)
    
    def _should_show(self, name: str) -> bool:
        """Check if geometry should be shown"""
        return {
            'robot': self.config.show_robot,
            'human': self.config.show_human,
            'collision': self.config.show_collision
        }.get(name, False)
    
    def _get_enabled_features(self) -> str:
        """Get enabled feature list"""
        features = []
        if self.config.show_robot:
            features.append("Robot")
        if self.config.show_human:
            features.append("Human")
        if self.config.show_collision:
            features.append("Collision")
        return ", ".join(features)
    
    def run(self):
        """Main visualization loop"""
        print("🎬 Starting visualization (quiet mode)")
        print("   ESC or close window to exit")
        
        self.is_running = True
        
        try:
            while self.is_running:
                loop_start = time.time()
                
                # Network update
                self.receiver.update()
                
                # Process new frames
                frame = self.receiver.get_latest_frame()
                if frame and frame.frame_id != self.last_frame_id:
                    self._process_frame(frame)
                    self.last_frame_id = frame.frame_id
                
                # Open3D update
                if not self.vis.poll_events():
                    break
                
                self.vis.update_renderer()
                
                # Stats
                self._update_stats()
                
                # Frame rate control
                self._control_fps(loop_start)
                
        except KeyboardInterrupt:
            print("\n🛑 Interrupted")
        except Exception as e:
            print(f"❌ Error: {e}")
        finally:
            self._shutdown()
    
    def _process_frame(self, frame: PacketBuffer):
        """Process new frame data"""
        try:
            # Skip already processed frames
            if frame.frame_id in self.processed_frames:
                return
            
            # Parse frame
            geometries = self.parser.parse_frame(frame)
            
            # Update geometries
            frame_complete = False
            for name in self.geometry_names:
                if name in geometries and self._should_show(name):
                    self._update_geometry(name, geometries[name])
                    
                    # Track complete human meshes
                    if name == 'human' and len(geometries[name].points) > 1000:
                        self.mesh_count += 1
                        frame_complete = True
            
            # Mark frame as processed
            if frame_complete:
                self.processed_frames.add(frame.frame_id)
                self._cleanup_processed_frames(frame.frame_id)
            
            self.frame_count += 1
            
        except Exception as e:
            # Only log errors occasionally
            if self.frame_count % 100 == 0:
                print(f"⚠️  Frame error: {e}")
    
    def _update_geometry(self, name: str, new_geometry: o3d.geometry.PointCloud):
        """Update geometry with new data"""
        try:
            old_geometry = self.geometries[name]
            old_geometry.points = new_geometry.points
            old_geometry.colors = new_geometry.colors
            self.vis.update_geometry(old_geometry)
        except Exception:
            pass  # Silent fail for geometry updates
    
    def _cleanup_processed_frames(self, current_frame_id: int):
        """Clean up old processed frame IDs"""
        old_frames = {fid for fid in self.processed_frames if fid < current_frame_id - 10}
        self.processed_frames -= old_frames
    
    def _update_stats(self):
        """Update and print stats periodically"""
        current_time = time.time()
        if current_time - self.last_stats_time > 5.0:  # Every 5 seconds
            net_stats = self.receiver.get_statistics()
            
            print(f"\r📊 Frames: {self.frame_count} | "
                  f"Meshes: {self.mesh_count} | "
                  f"Net: {net_stats.frames_complete} | "
                  f"Loss: {net_stats.frames_dropped} | "
                  f"Data: {net_stats.data_rate_mbps:.1f}Mbps",
                  end="", flush=True)
            
            self.last_stats_time = current_time
    
    def _control_fps(self, loop_start: float):
        """Control frame rate"""
        target_time = 1.0 / self.config.target_fps
        elapsed = time.time() - loop_start
        if elapsed < target_time:
            time.sleep(target_time - elapsed)
    
    def _shutdown(self):
        """Clean shutdown"""
        print("\n🔄 Shutting down...")
        
        self.is_running = False
        
        if self.receiver:
            self.receiver.shutdown()
        
        if self.vis:
            self.vis.destroy_window()
        
        print(f"📊 Final: {self.frame_count} frames, {self.mesh_count} complete meshes")
        print("✅ Shutdown complete")


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Robot Collision Visualizer - STAR Native",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python visualizer.py                    # Default: all features
  python visualizer.py --robot --human   # Robot and human only
  python visualizer.py --collision       # Collision only
  python visualizer.py --point-size 4.0  # Larger points
        """
    )
    
    # Features
    parser.add_argument('--robot', action='store_true', 
                       help='Show robot capsules')
    parser.add_argument('--human', action='store_true',
                       help='Show human mesh/joints')
    parser.add_argument('--collision', action='store_true',
                       help='Show collision contacts')
    parser.add_argument('--all', action='store_true',
                       help='Show all features')
    
    # Display
    parser.add_argument('--width', type=int, default=1200,
                       help='Window width')
    parser.add_argument('--height', type=int, default=800,
                       help='Window height')
    parser.add_argument('--fps', type=int, default=60,
                       help='Target FPS')
    parser.add_argument('--point-size', type=float, default=2.0,
                       help='Point size')
    
    return parser.parse_args()


def create_config(args) -> VisualizerConfig:
    """Create config from arguments"""
    # Default: show all if no specific flags
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
        window_width=args.width,
        window_height=args.height,
        target_fps=args.fps,
        point_size=args.point_size
    )


def main():
    """Main entry point"""
    print("🎯 ROBOT COLLISION VISUALIZER - STAR NATIVE")
    print("=" * 50)
    print("Pure C++ data display - Y-up coordinates, meters")
    print("Minimal logging - Clean, production-ready")
    print("=" * 50)
    
    # Parse arguments
    args = parse_arguments()
    config = create_config(args)
    
    # Create visualizer
    visualizer = CollisionVisualizer(config)
    
    if not visualizer.initialize():
        print("❌ Initialization failed")
        return 1
    
    # Run
    try:
        visualizer.run()
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())