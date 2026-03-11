from setuptools import find_packages, setup

package_name = 'task_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaisar',
    maintainer_email='kaisar@example.com',
    description='Simple CLI task publisher for fleet manager',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_manager = task_manager.task_manager:main',
        ],
    },
)