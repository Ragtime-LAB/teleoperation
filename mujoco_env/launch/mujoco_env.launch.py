from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    teleoperation_node = Node(
        package='mujoco_env',
        executable='mujoco_env',
        name='mujoco_env',
        output='screen',
        parameters=[{'mjcf_path': LaunchConfiguration('mjcf_path')}]
    )

    ld = LaunchDescription()
    ld.add_action(teleoperation_node)

    return ld
