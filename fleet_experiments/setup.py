from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'fleet_experiments'


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='kaisar',
    maintainer_email='kaisar.abutalipovv@gmail.com',
    description='Experiment runner and result logger for fleet evaluation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_experiment = fleet_experiments.run_experiment:main',
            'summarize_results = fleet_experiments.summarize_results:main',
            'send_nav2_goals = fleet_experiments.send_nav2_goals:main',
        ],
    },
)
