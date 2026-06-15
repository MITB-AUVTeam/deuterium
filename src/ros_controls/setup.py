from setuptools import find_packages, setup
from glob import glob 
import os

package_name = 'ros_controls'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    

      data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line tells ROS to install all files in the launch folder
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.onnx'))),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cupcake',
    maintainer_email='cupcake@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop =ros_controls.teleop:main',
            'combined_detections=ros_controls.combined_detections:main',
            'combined_detections_hsv_pose=ros_controls.combined_detection_hsv_pose:main',
            'dataset_collector_front =ros_controls.dataset_collector_front:main',
            'dataset_collector_down =ros_controls.dataset_collector_down:main',
            'hsv_tuned_detections=ros_controls.hsv_tuned_detections',
            'combined_detections_v2=ros_controls.combined_detections_v2',
        ],
    },
)
