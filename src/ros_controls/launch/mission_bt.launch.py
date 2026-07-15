import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mission_params = os.path.join(
        get_package_share_directory('main_task'), 'config', 'mission_params.yaml'
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
        maintask_bt,
        surge_service_node
    ])
