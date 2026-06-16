from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo_coord_transform'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch and config directories
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noah',
    maintainer_email='noah@todo.todo',
    description='Transform YOLO 2D pixel coordinates to 3D workspace coordinates using Homography',
    license='GPLv3',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_coord_transform_node = yolo_coord_transform.yolo_coord_transform_node:main',
            'calibration_node = yolo_coord_transform.calibration_node:main'
        ],
    },
)
