import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'laser_scan'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WingXly',
    maintainer_email='mslcwingxly@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "radar_static_tf = laser_scan.radar_static_tf:main",
            "stm32 = laser_scan.stm32:main",
            "cmd_controller = laser_scan.cmd_controller:main",
            "visualizer = laser_scan.visualizer:main"
        ],
    },
)
