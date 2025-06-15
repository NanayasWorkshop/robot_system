# debug_packets.py
import struct
from network_receiver import NetworkReceiver, PacketType

receiver = NetworkReceiver()
if not receiver.initialize():
    exit(1)

print("🔍 Debugging packet types and sizes...")
packet_counts = {}

try:
    for i in range(100):  # Check 100 packets
        packet = receiver.receive_packet()
        if packet:
            header, payload = packet
            packet_type = PacketType(header.packet_type)
            size = len(payload)
            
            key = f"{packet_type.name}({header.packet_type})"
            if key not in packet_counts:
                packet_counts[key] = []
            packet_counts[key].append(size)
            
            print(f"Frame {header.frame_id}: {packet_type.name} - {size} bytes")
            
            # Stop after collecting some data
            if len(packet_counts) > 0 and i > 50:
                break

except KeyboardInterrupt:
    pass

print("\n📊 Packet Summary:")
for packet_type, sizes in packet_counts.items():
    print(f"  {packet_type}: {len(sizes)} packets, sizes: {min(sizes)}-{max(sizes)} bytes")

receiver.shutdown()