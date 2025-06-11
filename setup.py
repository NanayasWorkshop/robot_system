from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup, find_packages
import pybind11

# Define the extension module
ext_modules = [
    Pybind11Extension(
        "delta_robot_cpp",
        [
            "cpp/bindings/bindings.cpp",
            "cpp/blocks/fermat_block.cpp",
            "cpp/blocks/kinematics_block.cpp", 
            "cpp/blocks/joint_state_block.cpp", 
            "cpp/blocks/orientation_block.cpp", 
            "cpp/blocks/segment_block.cpp", 
            "cpp/blocks/fabrik_initialization_block.cpp",
            "cpp/blocks/fabrik_iteration_block.cpp",
            "cpp/blocks/fabrik_solver_block.cpp",
            # NEW: Collision blocks
            "cpp/collision_blocks/capsule_creation_block.cpp",
            # Future collision blocks can be added here
        ],
        include_dirs=[
            # Path to your headers
            "cpp/",
            # Eigen (assuming system installation)
            "/usr/include/eigen3",  # Linux
            "/usr/local/include/eigen3",  # macOS with Homebrew
            # Add Windows Eigen path if needed
        ],
        cxx_std=17,  # C++17 standard
    ),
]

setup(
    name="delta_robot",
    version="0.1.0",
    author="Yuuki",
    description="Delta Robot Block-Based Control System with FABRIK and Collision Avoidance",
    long_description="Clean, modular delta robot control with FABRIK inverse kinematics, collision avoidance, and visualization",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    packages=find_packages(where="python"),
    package_dir={"": "python"},
    python_requires=">=3.8",
    install_requires=[
        "numpy",
        "plotly",
        "pybind11",
    ],
    zip_safe=False,
)