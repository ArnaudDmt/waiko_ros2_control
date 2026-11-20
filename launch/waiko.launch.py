"""Run the WAIKO estimator node with config/waiko_estimator.yaml."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waiko_ros2_control',
            executable='waiko_estimator_node',
            name='waiko_estimator',
            output='screen',
            parameters=[PathJoinSubstitution(
                [FindPackageShare('waiko_ros2_control'), 'config', 'waiko_estimator.yaml'])],
        ),
    ])
