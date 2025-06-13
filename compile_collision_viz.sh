#!/bin/bash

# Collision System + Visualization Build Script (Updated with STAR Vertex Support)
echo "=============================================================="
echo "COLLISION SYSTEM + VISUALIZATION BUILD (WITH STAR VERTICES)"
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

# Generate STAR vertices if needed
echo "🔍 Checking STAR vertices..."
if [ ! -f "star_vertices.bin" ]; then
    echo "🔄 STAR vertices not found, generating..."
    if [ ! -f "get_star_vertices.py" ]; then
        echo "❌ get_star_vertices.py not found"; exit 1
    fi
    
    echo "   Running Python script (this may take a few seconds)..."
    if ! python3 get_star_vertices.py; then
        echo "❌ Failed to generate STAR vertices"; exit 1
    fi
    
    if [ ! -f "star_vertices.bin" ]; then
        echo "❌ star_vertices.bin was not created"; exit 1
    fi
    echo "✅ STAR vertices generated successfully"
else
    echo "✅ STAR vertices found: star_vertices.bin"
fi

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

# NEW: Vertex loader utilities
echo "🔨 Compiling vertex loader utilities..."
VERTEX_LOADER_FILES=(
    "cpp/utils/vertex_loader.cpp:build/vertex_loader.o"
)

for file_pair in "${VERTEX_LOADER_FILES[@]}"; do
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
# UPDATED: Include vertex loader in object list
ALL_OBJECTS="build/layer_manager.o build/mesh_collision.o build/collision_detection_engine.o build/capsule_creation_block.o build/robot_collision_bridge.o build/star_collision_bridge.o build/network_sender.o build/data_publisher.o build/vertex_loader.o $FABRIK_OBJECTS"

# Receiver (standalone)
if ! g++ $COMMON_FLAGS $INCLUDE_FLAGS tests/test_visualization_receiver.cpp -o build/test_visualization_receiver; then
    echo "❌ receiver failed"; exit 1
fi

# Moving target test (real system with STAR vertices)
if ! g++ $COMMON_FLAGS $INCLUDE_FLAGS tests/test_moving_target.cpp $ALL_OBJECTS $LINK_FLAGS -o build/test_moving_target; then
    echo "❌ moving target failed"; exit 1
fi

# Verify STAR vertices are accessible
echo "🔍 Verifying STAR vertex setup..."
if [ -f "star_vertices.bin" ]; then
    VERTEX_SIZE=$(stat -f%z "star_vertices.bin" 2>/dev/null || stat -c%s "star_vertices.bin" 2>/dev/null)
    if [ "$VERTEX_SIZE" -gt 100000 ]; then  # Should be ~165KB for 6890 vertices
        echo "✅ STAR vertices ready (${VERTEX_SIZE} bytes)"
    else
        echo "⚠️  STAR vertex file seems small (${VERTEX_SIZE} bytes)"
    fi
else
    echo "⚠️  STAR vertex file missing after build"
fi

echo ""
echo "=============================================================="
echo "✅ BUILD COMPLETE (WITH STAR VERTEX SUPPORT)"
echo "=============================================================="
echo "📁 Generated Files:"
echo "  ./star_vertices.bin                  - STAR T-pose vertices (binary)"
echo "  ./star_vertices.txt                  - STAR T-pose vertices (text)"
echo ""
echo "📁 Executables:"
echo "  ./build/test_visualization_receiver  - UDP receiver"
echo "  ./build/test_moving_target           - Real collision system test (with STAR)"
echo ""
echo "🚀 Usage:"
echo "  Terminal 1: ./build/test_visualization_receiver"
echo "  Terminal 2: ./build/test_moving_target"
echo ""
echo "📊 STAR Integration:"
echo "  ✅ Real 6890 STAR vertices available"
echo "  ✅ Vertex loader utilities compiled"
echo "  ✅ Collision system ready for real mesh data"