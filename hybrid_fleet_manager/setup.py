from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'hybrid_fleet_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='kaisar',
    maintainer_email='kaisar@example.com',
    description='Hybrid fleet manager for MAPF + Nav2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_manager = src.fleet_manager_node:main',
            'grid_monitor = src.grid_monitor_node:main',
        ],
    },
)
