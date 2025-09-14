import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('iiwa_description'),
        'urdf',
        'iiwa7.urdf'
    )

    with open(urdf_path, 'r') as file:
        robot_desc = file.read()

    rviz_config_path = os.path.join(
        get_package_share_directory('iiwa_description'),
        'rviz',
        'view_robot.rviz'
    )

    if not os.path.exists(rviz_config_path):
        rviz_config_path = ""

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'publish_frequency': 50.0
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path] if rviz_config_path else [],
        parameters=[{
            'use_sim_time': False
        }]
    )

    # 创建启动描述
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        # joint_state_publisher_node,
        rviz2_node
    ])
