"""Replay a WAIKO input bag through the estimator node and record the estimated states."""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    input_bag = LaunchConfiguration('input_bag')
    output_bag = LaunchConfiguration('output_bag')

    estimator = Node(
        package='waiko_ros2_control',
        executable='waiko_estimator_node',
        name='waiko_estimator',
        output='screen',
        parameters=[PathJoinSubstitution(
            [FindPackageShare('waiko_ros2_control'), 'config', 'waiko_estimator.yaml'])],
    )
    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', output_bag, '/waiko_estimator/estimated_state'],
        output='screen',
    )
    replay = Node(
        package='waiko_ros2_control',
        executable='rosbag_publish_waiko',
        output='screen',
        parameters=[{'bag_path': input_bag}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('input_bag', description='Bag with the WAIKO initial state and inputs.'),
        DeclareLaunchArgument('output_bag', description='Bag to create with the estimated states.'),
        estimator,
        recorder,
        # Leaves time to the recorder to subscribe before the replay starts.
        TimerAction(period=2.0, actions=[replay]),
        RegisterEventHandler(OnProcessExit(
            target_action=replay,
            on_exit=[EmitEvent(event=Shutdown(reason='Replay finished'))])),
    ])
