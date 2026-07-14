import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    zed_launch_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'launch',
        'zed_camera.launch.py'
    )

    front_cam_config = os.path.join(
        get_package_share_directory('ros_controls'), 'config', 'front_cam.yaml'
    )
    yolo_config = os.path.join(
        get_package_share_directory('ros_controls'), 'config', 'main_gate_yolo.yaml'
    )

    cam_front = GroupAction(actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch_path),
            launch_arguments={
                'camera_name':    'zed2i_front',
                'camera_model':   'zed2i',
                'serial_number':  '34636984',
                'publish_tf':     'true',
                'publish_map_tf': 'true',
                'ros_params_override_path': front_cam_config,
                'custom_object_detection_config_path': yolo_config,
            }.items(),
        ),
    ])
    
    combined_detections = Node(
        package='ros_controls',
        executable='combined_detections',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        cam_front,
        combined_detections,
        rviz_node
    ])
