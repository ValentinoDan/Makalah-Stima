from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vis_astar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        # urdf files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        # maps
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.yaml') + glob('maps/*.pgm')),
        # config (rviz, obstacles)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.rviz') + glob('config/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel550',
    maintainer_email='19624107@mahasiswa.itb.ac.id',
    description='Clearance-Aware A* path planner — Enhancing A* with Obstacle Clearance Awareness for Mobile Robot Navigation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node = vis_astar.planner_node:main',
            'path_follower_node = vis_astar.path_follower_node:main',
            'odom_node = vis_astar.odom_node:main',
            'evaluate = vis_astar.evaluate:run_benchmark',
        ],
    },
)
