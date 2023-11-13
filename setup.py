from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
setup_args = generate_distutils_setup(
    packages=['robot_arm_3D_scan'],
    package_dir={'': 'scripts'},
    install_requires=[
        #Arrays
        "numpy",
        #Rotation and optimization tools
        "scipy",
        #Plots
        "matplotlib",
        #YAML file handling
        "pyyaml",
        #Point cloud handling
        "trimesh",
        "open3d"
    ]
)
setup(**setup_args)