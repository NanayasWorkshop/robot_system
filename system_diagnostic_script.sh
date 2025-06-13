#!/bin/bash

echo "=================================================================="
echo "COMPLETE SYSTEM DIAGNOSTIC FOR H5CPP INSTALLATION"
echo "=================================================================="
echo "Date: $(date)"
echo ""

# ===== BASIC SYSTEM INFO =====
echo "🖥️  SYSTEM INFORMATION:"
echo "------------------------------------------------------------------"
echo "Operating System:"
if [ -f /etc/os-release ]; then
    cat /etc/os-release | grep -E "^(NAME|VERSION|ID|VERSION_ID|PRETTY_NAME)"
fi
echo ""

echo "Kernel Version:"
uname -a
echo ""

echo "Architecture:"
arch
dpkg --print-architecture 2>/dev/null || echo "dpkg not available"
echo ""

echo "Available Memory:"
free -h
echo ""

echo "Available Disk Space:"
df -h /
echo ""

# ===== PACKAGE MANAGER INFO =====
echo "📦 PACKAGE MANAGER INFORMATION:"
echo "------------------------------------------------------------------"
echo "Package Manager Type:"
if command -v apt >/dev/null 2>&1; then
    echo "✅ APT (Debian/Ubuntu-based)"
    apt --version
elif command -v yum >/dev/null 2>&1; then
    echo "✅ YUM (RedHat-based)"
    yum --version
elif command -v dnf >/dev/null 2>&1; then
    echo "✅ DNF (Fedora-based)"
    dnf --version
elif command -v pacman >/dev/null 2>&1; then
    echo "✅ Pacman (Arch-based)"
    pacman --version
else
    echo "❌ Unknown package manager"
fi
echo ""

# ===== DEVELOPMENT TOOLS =====
echo "🔧 DEVELOPMENT TOOLS:"
echo "------------------------------------------------------------------"
echo "C++ Compiler:"
if command -v g++ >/dev/null 2>&1; then
    echo "✅ G++ available:"
    g++ --version | head -1
else
    echo "❌ G++ not found"
fi

if command -v clang++ >/dev/null 2>&1; then
    echo "✅ Clang++ available:"
    clang++ --version | head -1
else
    echo "❌ Clang++ not found"
fi
echo ""

echo "Build Tools:"
for tool in make cmake git pkg-config; do
    if command -v $tool >/dev/null 2>&1; then
        echo "✅ $tool: $($tool --version 2>/dev/null | head -1)"
    else
        echo "❌ $tool: not found"
    fi
done
echo ""

echo "Standard C++ Version Support:"
echo "Testing C++17 support..."
cat > /tmp/cpp17_test.cpp << 'EOF'
#include <iostream>
#include <filesystem>
int main() {
    std::cout << "C++17 works!" << std::endl;
    return 0;
}
EOF

if g++ -std=c++17 /tmp/cpp17_test.cpp -o /tmp/cpp17_test 2>/dev/null; then
    echo "✅ C++17 compilation successful"
    /tmp/cpp17_test
else
    echo "❌ C++17 compilation failed"
fi
rm -f /tmp/cpp17_test.cpp /tmp/cpp17_test
echo ""

# ===== HDF5 CURRENT STATUS =====
echo "📊 HDF5 LIBRARY STATUS:"
echo "------------------------------------------------------------------"
echo "HDF5 Packages Installed:"
if command -v apt >/dev/null 2>&1; then
    dpkg -l | grep -i hdf5 || echo "No HDF5 packages found via dpkg"
elif command -v rpm >/dev/null 2>&1; then
    rpm -qa | grep -i hdf5 || echo "No HDF5 packages found via rpm"
fi
echo ""

echo "HDF5 Headers Search:"
find /usr/include /usr/local/include 2>/dev/null | grep -i hdf5 | head -10 || echo "No HDF5 headers found in standard locations"
echo ""

echo "HDF5 Libraries Search:"
find /usr/lib /usr/local/lib /lib 2>/dev/null | grep -i hdf5 | head -10 || echo "No HDF5 libraries found in standard locations"
echo ""

echo "pkg-config HDF5 Support:"
if command -v pkg-config >/dev/null 2>&1; then
    if pkg-config --exists hdf5; then
        echo "✅ HDF5 found via pkg-config:"
        echo "  Version: $(pkg-config --modversion hdf5)"
        echo "  Cflags: $(pkg-config --cflags hdf5)"
        echo "  Libs: $(pkg-config --libs hdf5)"
    else
        echo "❌ HDF5 not found via pkg-config"
        echo "Available HDF5-related packages:"
        pkg-config --list-all | grep -i hdf5 || echo "None found"
    fi
else
    echo "❌ pkg-config not available"
fi
echo ""

# ===== H5CPP CURRENT STATUS =====
echo "🔬 H5CPP LIBRARY STATUS:"
echo "------------------------------------------------------------------"
echo "H5CPP Headers Search:"
find /usr/include /usr/local/include 2>/dev/null | grep -i h5cpp | head -10 || echo "No h5cpp headers found"
echo ""

echo "H5CPP Libraries Search:"
find /usr/lib /usr/local/lib /lib 2>/dev/null | grep -i h5cpp | head -10 || echo "No h5cpp libraries found"
echo ""

echo "H5CPP Package Search:"
if command -v apt >/dev/null 2>&1; then
    echo "Searching apt repositories for h5cpp packages:"
    apt search h5cpp 2>/dev/null | head -5 || echo "No h5cpp packages found in repositories"
    echo ""
    
    echo "Searching for libh5cpp packages:"
    apt search libh5cpp 2>/dev/null | head -5 || echo "No libh5cpp packages found in repositories"
fi
echo ""

# ===== EIGEN LIBRARY STATUS =====
echo "🧮 EIGEN LIBRARY STATUS:"
echo "------------------------------------------------------------------"
echo "Eigen Headers:"
find /usr/include /usr/local/include 2>/dev/null | grep -E "eigen3?/" | head -5 || echo "No Eigen headers found"
echo ""

echo "Eigen Packages:"
if command -v apt >/dev/null 2>&1; then
    dpkg -l | grep -i eigen || echo "No Eigen packages found"
fi
echo ""

# ===== TEST COMPILATION ENVIRONMENT =====
echo "🧪 COMPILATION ENVIRONMENT TEST:"
echo "------------------------------------------------------------------"
echo "Testing basic compilation with current setup..."

# Test 1: Basic HDF5 compilation
echo "Test 1: Basic HDF5 compilation"
cat > /tmp/hdf5_test.c << 'EOF'
#include <hdf5.h>
#include <stdio.h>
int main() {
    printf("HDF5 library version: %s\n", H5_VERS_INFO);
    return 0;
}
EOF

if gcc $(pkg-config --cflags hdf5 2>/dev/null) /tmp/hdf5_test.c $(pkg-config --libs hdf5 2>/dev/null) -o /tmp/hdf5_test 2>/dev/null; then
    echo "✅ Basic HDF5 C compilation successful"
    /tmp/hdf5_test
else
    echo "❌ Basic HDF5 C compilation failed"
    echo "Trying without pkg-config..."
    if gcc -I/usr/include/hdf5/serial /tmp/hdf5_test.c -lhdf5 -o /tmp/hdf5_test 2>/dev/null; then
        echo "✅ HDF5 C compilation with manual flags successful"
        /tmp/hdf5_test
    else
        echo "❌ HDF5 C compilation failed even with manual flags"
    fi
fi
rm -f /tmp/hdf5_test.c /tmp/hdf5_test
echo ""

# Test 2: H5CPP compilation
echo "Test 2: H5CPP compilation"
cat > /tmp/h5cpp_test.cpp << 'EOF'
#include <h5cpp/hdf5.hpp>
#include <iostream>
int main() {
    std::cout << "h5cpp compilation successful!" << std::endl;
    return 0;
}
EOF

if g++ -std=c++17 $(pkg-config --cflags hdf5 2>/dev/null) /tmp/h5cpp_test.cpp $(pkg-config --libs hdf5 2>/dev/null) -o /tmp/h5cpp_test 2>/dev/null; then
    echo "✅ H5CPP compilation successful"
    /tmp/h5cpp_test
else
    echo "❌ H5CPP compilation failed (expected if h5cpp not installed)"
fi
rm -f /tmp/h5cpp_test.cpp /tmp/h5cpp_test
echo ""

# Test 3: Eigen compilation
echo "Test 3: Eigen compilation"
cat > /tmp/eigen_test.cpp << 'EOF'
#include <Eigen/Dense>
#include <iostream>
int main() {
    Eigen::Vector3d v(1.0, 2.0, 3.0);
    std::cout << "Eigen compilation successful! Vector: " << v.transpose() << std::endl;
    return 0;
}
EOF

if g++ -std=c++17 -I/usr/include/eigen3 /tmp/eigen_test.cpp -o /tmp/eigen_test 2>/dev/null; then
    echo "✅ Eigen compilation successful"
    /tmp/eigen_test
else
    echo "❌ Eigen compilation failed"
fi
rm -f /tmp/eigen_test.cpp /tmp/eigen_test
echo ""

# ===== NETWORK AND DOWNLOAD CAPABILITIES =====
echo "🌐 NETWORK AND DOWNLOAD CAPABILITIES:"
echo "------------------------------------------------------------------"
echo "Network connectivity:"
if ping -c 1 8.8.8.8 >/dev/null 2>&1; then
    echo "✅ Internet connectivity available"
else
    echo "❌ No internet connectivity"
fi

echo "Download tools:"
for tool in wget curl git; do
    if command -v $tool >/dev/null 2>&1; then
        echo "✅ $tool available"
    else
        echo "❌ $tool not found"
    fi
done
echo ""

# ===== CURRENT DIRECTORY CONTEXT =====
echo "📁 CURRENT DIRECTORY CONTEXT:"
echo "------------------------------------------------------------------"
echo "Current working directory:"
pwd
echo ""

echo "Current directory contents:"
ls -la
echo ""

echo "Looking for collision_data.h5:"
if [ -f "collision_data.h5" ]; then
    echo "✅ Found collision_data.h5"
    ls -lh collision_data.h5
    file collision_data.h5
else
    echo "❌ collision_data.h5 not found in current directory"
    echo "Searching for collision_data.h5 in nearby directories:"
    find . -name "collision_data.h5" 2>/dev/null || echo "Not found in current tree"
fi
echo ""

echo "Looking for C++ collision system files:"
if [ -d "cpp/collision" ]; then
    echo "✅ Found cpp/collision directory"
    ls -la cpp/collision/
else
    echo "❌ cpp/collision directory not found"
    echo "Looking for collision-related files:"
    find . -name "*collision*" -type f 2>/dev/null | head -10 || echo "No collision files found"
fi
echo ""

# ===== SUMMARY AND RECOMMENDATIONS =====
echo "=================================================================="
echo "📋 DIAGNOSTIC SUMMARY AND RECOMMENDATIONS"
echo "=================================================================="

echo "System Type: $(cat /etc/os-release 2>/dev/null | grep PRETTY_NAME | cut -d'"' -f2 || uname -s)"
echo ""

echo "Ready for h5cpp installation:"
if command -v g++ >/dev/null 2>&1 && command -v cmake >/dev/null 2>&1 && command -v git >/dev/null 2>&1; then
    echo "✅ Basic development tools available"
else
    echo "❌ Missing development tools - install build-essential cmake git first"
fi

if pkg-config --exists hdf5 2>/dev/null; then
    echo "✅ HDF5 properly configured"
else
    echo "❌ HDF5 not properly configured - install libhdf5-dev"
fi

echo ""
echo "Recommended next steps will be provided based on this diagnostic..."
echo ""
echo "=================================================================="
echo "DIAGNOSTIC COMPLETE - Please share this output"
echo "=================================================================="
