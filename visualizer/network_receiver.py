#!/usr/bin/env python3
"""
Network Receiver - UDP Packet Manager
Handles UDP communication with C++ collision system
FIXED: Properly handles multiple packets of the same type per frame (vertex batching)
"""

import socket
import struct
import time
from typing import Dict, Optional, Tuple, Any, List
from enum import IntEnum
from dataclasses import dataclass, field
import threading


class PacketType(IntEnum):
    """Packet types matching C++ visualization_data.hpp"""
    ROBOT_CAPSULES = 1
    HUMAN_POSE = 2
    COLLISION_LAYERS = 3
    COLLISION_CONTACTS = 4
    FRAME_SYNC = 5


@dataclass
class PacketHeader:
    """Packet header structure matching C++"""
    packet_type: int
    data_size: int
    frame_id: int
    timestamp_ms: int


@dataclass
class PacketBuffer:
    """Frame buffer for collecting multiple packets"""
    frame_id: int
    timestamp_ms: int
    packets: Dict[PacketType, List[bytes]] = field(default_factory=dict)  # FIXED: List of packets per type
    is_complete: bool = False
    receive_time: float = 0.0
    
    def __post_init__(self):
        self.receive_time = time.time()
        # Initialize empty lists for all packet types
        for packet_type in PacketType:
            self.packets[packet_type] = []
    
    def add_packet(self, packet_type: PacketType, payload: bytes):
        """Add a packet to this frame buffer"""
        if packet_type not in self.packets:
            self.packets[packet_type] = []
        self.packets[packet_type].append(payload)
    
    def get_single_packet(self, packet_type: PacketType) -> Optional[bytes]:
        """Get single packet (for non-batched types like robot, collision)"""
        packets = self.packets.get(packet_type, [])
        return packets[-1] if packets else None  # Return latest packet
    
    def get_all_packets(self, packet_type: PacketType) -> List[bytes]:
        """Get all packets of a type (for batched types like human vertices)"""
        return self.packets.get(packet_type, [])
    
    def has_packet_type(self, packet_type: PacketType) -> bool:
        """Check if frame has any packets of given type"""
        return len(self.packets.get(packet_type, [])) > 0
    
    def get_packet_count(self, packet_type: PacketType) -> int:
        """Get number of packets of given type"""
        return len(self.packets.get(packet_type, []))
    
    def get_packet_sizes(self, packet_type: PacketType) -> List[int]:
        """Get sizes of all packets of given type (for debugging)"""
        return [len(packet) for packet in self.packets.get(packet_type, [])]


@dataclass
class NetworkStats:
    """Network statistics tracking"""
    packets_received: int = 0
    packets_dropped: int = 0
    frames_complete: int = 0
    frames_dropped: int = 0
    data_rate_mbps: float = 0.0
    last_frame_time: float = 0.0
    bytes_received: int = 0
    
    # NEW: Batch tracking stats
    vertex_batches_received: int = 0
    complete_vertex_frames: int = 0


class NetworkReceiver:
    """UDP packet receiver for collision visualization data"""
    
    def __init__(self, port: int = 9999, timeout_ms: float = 50.0):
        self.port = port
        self.timeout_ms = timeout_ms / 1000.0  # Convert to seconds
        
        # Socket management
        self.socket: Optional[socket.socket] = None
        self.is_running = False
        
        # Frame management
        self.current_frame: Optional[PacketBuffer] = None
        self.latest_complete_frame: Optional[PacketBuffer] = None
        self.frame_timeout = 0.05  # 50ms timeout for incomplete frames
        
        # Statistics
        self.stats = NetworkStats()
        self.start_time = time.time()
        
        # Thread safety
        self.lock = threading.Lock()
    
    def initialize(self) -> bool:
        """Initialize UDP socket"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind(('127.0.0.1', self.port))
            self.socket.settimeout(self.timeout_ms)
            self.is_running = True
            
            print(f"✅ NetworkReceiver listening on port {self.port}")
            print(f"   FIXED: Multi-packet batching support enabled")
            return True
            
        except Exception as e:
            print(f"❌ Failed to initialize NetworkReceiver: {e}")
            return False
    
    def shutdown(self):
        """Clean shutdown"""
        self.is_running = False
        if self.socket:
            self.socket.close()
            self.socket = None
        print("📡 NetworkReceiver shutdown")
    
    def receive_packet(self) -> Optional[Tuple[PacketHeader, bytes]]:
        """Receive and parse single UDP packet"""
        if not self.socket or not self.is_running:
            return None
        
        try:
            # Receive packet
            data, addr = self.socket.recvfrom(8192)  # Max packet size
            
            # Parse header (4 uint32_t = 16 bytes)
            if len(data) < 16:
                print(f"⚠️  Packet too small: {len(data)} bytes")
                return None
            
            header_data = struct.unpack('IIII', data[:16])
            header = PacketHeader(
                packet_type=header_data[0],
                data_size=header_data[1],
                frame_id=header_data[2],
                timestamp_ms=header_data[3]
            )
            
            # Extract payload
            payload = data[16:16+header.data_size]
            
            # Update statistics
            self.stats.packets_received += 1
            self.stats.bytes_received += len(data)
            
            # Track vertex batches specifically
            if PacketType(header.packet_type) == PacketType.HUMAN_POSE and len(payload) > 200:
                self.stats.vertex_batches_received += 1
            
            return header, payload
            
        except socket.timeout:
            # Normal timeout - no data available
            return None
        except Exception as e:
            print(f"⚠️  Packet receive error: {e}")
            return None
    
    def process_packet(self, header: PacketHeader, payload: bytes):
        """Process received packet and update frame buffer"""
        packet_type = PacketType(header.packet_type)
        
        # Check if this is a new frame
        if (self.current_frame is None or 
            self.current_frame.frame_id != header.frame_id):
            
            # Finalize previous frame if it was incomplete
            if self.current_frame and not self.current_frame.is_complete:
                self.stats.frames_dropped += 1
                print(f"⚠️  Dropping incomplete frame {self.current_frame.frame_id}")
            
            # Start new frame
            self.current_frame = PacketBuffer(
                frame_id=header.frame_id,
                timestamp_ms=header.timestamp_ms
            )
            
            print(f"🆕 Starting new frame {header.frame_id}")
        
        # FIXED: Add packet to frame (instead of overwriting)
        self.current_frame.add_packet(packet_type, payload)
        
        # Debug: Log packet addition
        if packet_type == PacketType.HUMAN_POSE:
            vertex_packets = len(self.current_frame.get_all_packets(PacketType.HUMAN_POSE))
            if len(payload) > 200:  # Vertex batch
                print(f"   📦 Added vertex batch {vertex_packets} to frame {header.frame_id} ({len(payload)} bytes)")
            else:
                print(f"   👤 Added joint data to frame {header.frame_id} ({len(payload)} bytes)")
        elif packet_type == PacketType.ROBOT_CAPSULES:
            print(f"   🤖 Added robot data to frame {header.frame_id} ({len(payload)} bytes)")
        elif packet_type == PacketType.COLLISION_CONTACTS:
            print(f"   💥 Added collision data to frame {header.frame_id} ({len(payload)} bytes)")
        elif packet_type == PacketType.FRAME_SYNC:
            print(f"   🔄 Frame {header.frame_id} sync received")
        
        # Check if frame is complete (got FRAME_SYNC)
        if packet_type == PacketType.FRAME_SYNC:
            self.current_frame.is_complete = True
            
            # Check if we got vertex batches
            vertex_packets = self.current_frame.get_all_packets(PacketType.HUMAN_POSE)
            vertex_batch_count = sum(1 for p in vertex_packets if len(p) > 200)
            
            if vertex_batch_count > 0:
                self.stats.complete_vertex_frames += 1
                print(f"   🎯 Frame {header.frame_id} complete with {vertex_batch_count} vertex batches")
            
            # Store as latest complete frame
            with self.lock:
                self.latest_complete_frame = self.current_frame
                self.stats.frames_complete += 1
                self.stats.last_frame_time = time.time()
    
    def update(self) -> bool:
        """Update receiver - call this regularly from main loop"""
        if not self.is_running:
            return False
        
        # Clean up old incomplete frames
        if (self.current_frame and 
            not self.current_frame.is_complete and
            time.time() - self.current_frame.receive_time > self.frame_timeout):
            
            self.stats.frames_dropped += 1
            print(f"⏰ Timeout: Dropping incomplete frame {self.current_frame.frame_id}")
            self.current_frame = None
        
        # Receive multiple packets per update
        packets_this_update = 0
        while packets_this_update < 20:  # Increased limit for batched packets
            packet = self.receive_packet()
            if packet is None:
                break
            
            header, payload = packet
            self.process_packet(header, payload)
            packets_this_update += 1
        
        # Update data rate statistics
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            self.stats.data_rate_mbps = (self.stats.bytes_received * 8) / (elapsed * 1_000_000)
        
        return packets_this_update > 0
    
    def get_latest_frame(self) -> Optional[PacketBuffer]:
        """Get latest complete frame (thread-safe) and clear it"""
        with self.lock:
            frame = self.latest_complete_frame
            self.latest_complete_frame = None  # Clear after returning
            return frame
    
    def get_statistics(self) -> NetworkStats:
        """Get current network statistics"""
        # Calculate current FPS
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        stats_copy = NetworkStats(
            packets_received=self.stats.packets_received,
            packets_dropped=self.stats.packets_dropped,
            frames_complete=self.stats.frames_complete,
            frames_dropped=self.stats.frames_dropped,
            data_rate_mbps=self.stats.data_rate_mbps,
            last_frame_time=self.stats.last_frame_time,
            bytes_received=self.stats.bytes_received,
            vertex_batches_received=self.stats.vertex_batches_received,
            complete_vertex_frames=self.stats.complete_vertex_frames
        )
        
        return stats_copy
    
    def get_debug_info(self) -> str:
        """Get debug information string"""
        stats = self.get_statistics()
        elapsed = time.time() - self.start_time
        
        fps = stats.frames_complete / elapsed if elapsed > 0 else 0
        packet_loss = (stats.frames_dropped / (stats.frames_complete + stats.frames_dropped) * 100) if (stats.frames_complete + stats.frames_dropped) > 0 else 0
        
        return (f"Net: {stats.packets_received} pkts | "
                f"{stats.frames_complete} frames | "
                f"{stats.vertex_batches_received} vertex batches | "
                f"{stats.complete_vertex_frames} complete vertex frames | "
                f"{fps:.1f} FPS | "
                f"{packet_loss:.1f}% loss | "
                f"{stats.data_rate_mbps:.2f} Mbps")


# Test function
if __name__ == "__main__":
    print("🧪 Testing NetworkReceiver (FIXED - Multi-packet support)...")
    
    receiver = NetworkReceiver()
    if not receiver.initialize():
        exit(1)
    
    print("📡 Listening for packets... (Ctrl+C to stop)")
    print("   Now properly handles multiple vertex batches per frame")
    
    try:
        while True:
            receiver.update()
            
            # Check for new frame
            frame = receiver.get_latest_frame()
            if frame:
                print(f"📦 Frame {frame.frame_id}:")
                for ptype in PacketType:
                    packets = frame.get_all_packets(ptype)
                    if packets:
                        print(f"   {ptype.name}: {len(packets)} packets")
                        for i, packet in enumerate(packets):
                            print(f"     Packet {i}: {len(packet)} bytes")
            
            time.sleep(0.01)  # 100 Hz update
            
    except KeyboardInterrupt:
        print("\n🛑 Stopping...")
        receiver.shutdown()