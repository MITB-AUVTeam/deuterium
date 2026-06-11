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
    yolo_config=os.path.join(
        get_package_share_directory('ros_controls'), 'config', 'custom_yolo.yaml'
    )
    mission_params = os.path.join(
        get_package_share_directory('prequalification_bt'), 'config', 'mission_params.yaml'
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
    
    combined_detections = Node(
    	package='ros_controls',
    	executable='combined_detections',
    )
    
    prequal_bt = Node(
    	package='prequalification_bt',
    	executable='prequalification',
    	parameters=[mission_params],
    )
    hsv_tuned_detections=Node(
        package='ros_controls',
    	executable='hsv_tuned_detections',
    )

    combined_detections_v2=Node(
    	package='ros_controls',
    	executable='combined_detections_v2',
    )
    
    

    #return LaunchDescription([cam_front,combined_detections,prequal_bt])

    #if this doesn't work uncomment the above one to go back to first version
    return LaunchDescription([cam_front,combined_detections_v2,prequal_bt])
