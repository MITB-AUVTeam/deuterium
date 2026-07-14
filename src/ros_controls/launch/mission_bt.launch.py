import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mission_params = os.path.join(
        get_package_share_directory('main_task'), 'config', 'mission_params.yaml'
    )

    zed_launch_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'launch',
        'zed_camera.launch.py'
    )

    front_cam_config = os.path.join(
        get_package_share_directory('ros_controls'), 'config', 'front_cam.yaml'
    )
    yolo_config = os.path.join(
        get_package_share_directory('ros_controls'), 'config', 'custom_yolo.yaml'
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

    combined_detections_hsv_pose = Node(
        package='ros_controls',
        executable='combined_detections',
    )

    maintask_bt = Node(
        package='main_task',
        executable='main_task_node',
        parameters=[mission_params],
        output='screen'
    )

    surge_service_node = Node(
        package='ros_controls',
        executable='surge_service',
        output='screen'
    )

    return LaunchDescription([
        cam_front,
        combined_detections_hsv_pose,
        maintask_bt,
        surge_service_node
    ])
