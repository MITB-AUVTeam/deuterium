import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace,Node
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
    down_cam_config = os.path.join(
        get_package_share_directory('ros_controls'), 'config', 'down_cam.yaml'
    )

    down_cam_perf_config = os.path.join(
        get_package_share_directory('ros_controls'), 'config', 'down_cam_performance_mode.yaml'
    )


    yolo_config=os.path.join(
        get_package_share_directory('ros_controls'), 'config', 'custom_yolo.yaml'
    )

    cam_front = GroupAction(actions=[
        #PushRosNamespace('zed_front'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch_path),
            launch_arguments={
                'camera_name':    'zed2i_front',
                'camera_model':   'zed2i',
                'serial_number':  '34636984',
                'publish_tf':     'true',
                'publish_map_tf': 'true',
                'ros_params_override_path':front_cam_config,
                'custom_object_detection_config_path': yolo_config,
            }.items(),
        ),
    ])

    #Bottom camera runs on PERFORMANCE mode to prevent lag caused due to running both cams and yolo model on them both
    #Can switch to NEURAL_LIGHT too , change it in the depth_mode parameter of down_cam_performance_mode.yaml

    cam_down = GroupAction(actions=[
        #PushRosNamespace('zed_down'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch_path),
            launch_arguments={
                'camera_name':    'zed2i_down',
                'camera_model':   'zed2i',
                'serial_number':  '38605411',
                'publish_tf':     'false',
                'publish_map_tf': 'false',
                'ros_params_override_path':down_cam_perf_config,
                #'custom_object_detection_config_path': yolo_config,
            }.items(),
        ),
    ])

    combined_detections_hsv_pose = Node(
    	package='ros_controls',
    	executable='combined_detections_hsv_pose',
    )
    

    return LaunchDescription([cam_front, cam_down,combined_detections_hsv_pose])
