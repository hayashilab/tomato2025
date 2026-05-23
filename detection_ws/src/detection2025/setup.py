from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'detection2025'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='.'),
    package_dir={'': '.'},
    data_files=[
        # Standard ROS2 metadata
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),

        # Config files
        (f'share/{package_name}/config', glob('config/*.yaml')),

        # Model files
        (f'share/{package_name}/models', glob('models/*')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'ultralytics',
        'opencv-python',
        'numpy',
        'cv_bridge',
        'torch',
        'torchvision'
    ],
    zip_safe=True,
    maintainer='hayashi',
    maintainer_email='jasinyonpang@gmail.com',
    description='YOLO-based tomato detection and depth integration for ROS2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'main = detection2025.main:main',
            'tomato_detection = detection2025.tomato_detection:main',
        ],
    },
)
