#!/bin/bash

# Collision System + Visualization Build Script (Updated with STAR Daemon Client)
echo "=============================================================="
echo "COLLISION SYSTEM + VISUALIZATION BUILD (WITH STAR DAEMON)"
echo "=============================================================="

BASE_DIR="/home/yuuki/Documents/FABRIK_CPP/robot/delta_unit"
cd "$BASE_DIR" || { echo "❌ Failed to change to base directory"; exit 1; }

# Create directories
mkdir -p cpp/utils cpp/visualization build tests

# Check dependencies
echo "🔍 Checking dependencies..."
if ! pkg-config --exists hdf5; then echo "❌ HDF5 not found"; exit 1; fi
if ! pkg-config --exists eigen3; then echo "❌ Eigen3 not found"; exit 1; fi
if ! pkg-config --exists tbb; then echo "❌ TBB not found"; exit 1; fi
echo "✅ All dependencies found"

# Set compilation flags
COMMON_FLAGS="-std=c++17 -Wall -Wextra -O2"
INCLUDE_FLAGS="-I cpp/ $(pkg-config --cflags eigen3 hdf5 tbb)"
LINK_FLAGS="$(pkg-config --libs hdf5 tbb) -pthread"

# Core collision system
echo "🔨 Compiling collision system..."
CORE_FILES=(
    "cpp/collision/layer_manager.cpp:build/layer_manager.o"
    "cpp/collision/mesh_collision.cpp:build/mesh_collision.o" 
    "cpp/collision/collision_detection_engine.cpp:build/collision_detection_engine.o"
    "cpp/collision_blocks/capsule_creation_block.cpp:build/capsule_creation_block.o"
)

for file_pair in "${CORE_FILES[@]}"; do
    IFS=':' read -r source target <<< "$file_pair"
    if ! g++ $COMMON_FLAGS $INCLUDE_FLAGS -c "$source" -o "$target"; then
        echo "❌ $(basename "$source") failed"; exit 1
    fi
done

# NEW: STAR Daemon Client utilities
echo "🔨 Compiling STAR daemon client..."
STAR_CLIENT_FILES=(
    "cpp/utils/star_daemon_client.cpp:build/star_daemon_client.o"
)

for file_pair in "${STAR_CLIENT_FILES[@]}"; do
    IFS=':' read -r source target <<< "$file_pair"
    if [ -f "$source" ]; then
        if ! g++ $COMMON_FLAGS $INCLUDE_FLAGS -c "$source" -o "$target"; then
            echo "❌ $(basename "$source") failed"; exit 1
        fi
        echo "✅ $(basename "$source") compiled"
    else
        echo "❌ Source file not found: $source"; exit 1
    fi
done

# FABRIK blocks
FABRIK_OBJECTS=""
FABRIK_BLOCKS=(segment_block fabrik_solver_block fabrik_initialization_block fabrik_iteration_block fermat_block kinematics_block joint_state_block orientation_block)

for block in "${FABRIK_BLOCKS[@]}"; do
    if [ -f "cpp/blocks/${block}.cpp" ]; then
        if g++ $COMMON_FLAGS $INCLUDE_FLAGS -c "cpp/blocks/${block}.cpp" -o "build/${block}.o"; then
            FABRIK_OBJECTS="$FABRIK_OBJECTS build/${block}.o"
        fi
    fi
done

# Bridge components
BRIDGE_FILES=(
    "cpp/bridges/robot_collision_bridge.cpp:build/robot_collision_bridge.o"
    "cpp/bridges/star_collision_bridge.cpp:build/star_collision_bridge.o"
)

for file_pair in "${BRIDGE_FILES[@]}"; do
    IFS=':' read -r source target <<< "$file_pair"
    if ! g++ $COMMON_FLAGS $INCLUDE_FLAGS -c "$source" -o "$target"; then
        echo "❌ $(basename "$source") failed"; exit 1
    fi
done

# Visualization system
echo "🔨 Compiling visualization..."
if ! g++ $COMMON_FLAGS $INCLUDE_FLAGS -c cpp/visualization/network_sender.cpp -o build/network_sender.o; then
    echo "❌ network_sender failed"; exit 1
fi
if ! g++ $COMMON_FLAGS $INCLUDE_FLAGS -c cpp/visualization/data_publisher.cpp -o build/data_publisher.o; then
    echo "❌ data_publisher failed"; exit 1
fi

# Test programs
echo "🔨 Compiling tests..."
# UPDATED: Include STAR daemon client in object list (removed vertex_loader)
ALL_OBJECTS="build/layer_manager.o build/mesh_collision.o build/collision_detection_engine.o build/capsule_creation_block.o build/robot_collision_bridge.o build/star_collision_bridge.o build/network_sender.o build/data_publisher.o build/star_daemon_client.o $FABRIK_OBJECTS"

# Receiver (standalone)
if ! g++ $COMMON_FLAGS $INCLUDE_FLAGS tests/test_visualization_receiver.cpp -o build/test_visualization_receiver; then
    echo "❌ receiver failed"; exit 1
fi

# Moving target test (real system with STAR daemon)
if ! g++ $COMMON_FLAGS $INCLUDE_FLAGS tests/test_moving_target.cpp $ALL_OBJECTS $LINK_FLAGS -o build/test_moving_target; then
    echo "❌ moving target failed"; exit 1
fi

# NEW: STAR daemon client test
if ! g++ $COMMON_FLAGS $INCLUDE_FLAGS tests/test_star_daemon_client.cpp build/star_daemon_client.o $LINK_FLAGS -o build/test_star_daemon_client; then
    echo "❌ STAR daemon client test failed"; exit 1
fi

# Verify STAR daemon availability
echo "🔍 Verifying STAR daemon setup..."
if [ -f "python/star_daemon.py" ]; then
    echo "✅ STAR daemon script found: python/star_daemon.py"
    
    # Check if daemon is running
    if [ -S "/tmp/star_daemon.sock" ]; then
        echo "✅ STAR daemon appears to be running"
    else
        echo "⚠️  STAR daemon not running (socket not found)"
        echo "   Start with: python3 python/star_daemon.py"
    fi
else
    echo "⚠️  STAR daemon script not found"
fi

echo ""
echo "=============================================================="
echo "✅ BUILD COMPLETE (WITH STAR DAEMON INTEGRATION)"
echo "=============================================================="
echo "📁 Executables:"
echo "  ./build/test_visualization_receiver     - UDP receiver"
echo "  ./build/test_moving_target              - Real collision system test (with STAR daemon)"
echo "  ./build/test_star_daemon_client         - STAR daemon client test"
echo ""
echo "🚀 Usage:"
echo "  1. Start STAR daemon:    python3 python/star_daemon.py"
echo "  2. Start visualizer:     ./build/test_visualization_receiver"
echo "  3. Run collision test:   ./build/test_moving_target"
echo ""
echo "🧪 Testing:"
echo "  Test daemon alone:       python3 python/test_star_daemon.py"
echo "  Test C++ client:         ./build/test_star_daemon_client"
echo ""
echo "📊 STAR Integration:"
echo "  ✅ Real-time mesh deformation via daemon"
echo "  ✅ C++ client utilities compiled"
echo "  ✅ Low-latency IPC communication ready"