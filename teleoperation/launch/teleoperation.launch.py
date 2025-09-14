from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    teleoperation_node = Node(
        package='teleoperation',
        executable='teleoperation',
        name='teleoperation',
        output='screen',
        parameters=[LaunchConfiguration('param_file')]
    )

    robot_controller_node = Node(
        package='teleoperation',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        parameters=[LaunchConfiguration('param_file'), {'urdf_path': LaunchConfiguration('urdf_path')}]
    )

    mujoco_env_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            PathJoinSubstitution([
                FindPackageShare('mujoco_env'),
                'launch',
                'mujoco_env.launch.py'
            ])
        ])
    )

    ld = LaunchDescription()
    ld.add_action(mujoco_env_launch)
    ld.add_action(teleoperation_node)
    ld.add_action(robot_controller_node)

    return ld
