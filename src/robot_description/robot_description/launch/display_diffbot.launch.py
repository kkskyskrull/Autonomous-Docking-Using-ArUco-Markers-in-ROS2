import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the robot_description package
    pkg_path = get_package_share_directory('robot_description')
    
    # Use absolute path from source directory
    # Change 'user' to your actual username
    source_urdf_path = '/home/user/ros2_ws/src/robot_description/robot_description/urdf/diffbot.urdf'
    source_rviz_path = '/home/user/ros2_ws/src/robot_description/robot_description/rviz/diffbot.rviz'
    
    # Read URDF contents
    with open(source_urdf_path, 'r') as infp:
        robot_desc = infp.read()
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', source_rviz_path],
            output='screen'
        )
    ])
