from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    iiwa_description_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            PathJoinSubstitution([
                FindPackageShare('iiwa_description'),
                'launch',
                'iiwa_description.launch.py'
            ]),
        ])
    )

    teleoperation_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            PathJoinSubstitution([
                FindPackageShare('teleoperation'),
                'launch',
                'teleoperation.launch.py'
            ])
        ])
    )

    ld = LaunchDescription()
    ld.add_action(iiwa_description_launch)
    ld.add_action(teleoperation_launch)

    return ld
