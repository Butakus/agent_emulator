#!/usr/bin/python3

""" Test launchfile to get familiar with new syntax """

import os
import sys
import math

from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch.actions import TimerAction, ExecuteProcess

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory

def deg_to_rad(deg):
    return deg * math.pi / 180.0

def generate_launch_description():

    ld = LaunchDescription()

    # Initial poses
    initial_poses = [
        [4.0, 0.0, 0.0],
        [-4.0, 0.0, 0.0],
        [-0.0, 4.0, 0.0],
    ]

    # Load agent nodes
    for i, pose in enumerate(initial_poses):
        agent_name = "agent_{:02d}".format(i)
        agent_nodes = []
        #########################################################################
        # Agent emulator
        #########################################################################
        # Parameters
        agent_emulator_params = [{
            "initial_pose": pose,
            "update_rate": 50.0,
            "agent_frame_id": agent_name + "/base_link",
            "odom_frame_id": agent_name + "/odom",
            "goto_velocity_linear": 1.0,
            "goto_velocity_angular": deg_to_rad(90),
            "goto_goal_tolerance": 0.05,
        }]
        agent_emulator_node = ComposableNode(
            package='agent_emulator', plugin='agent_emulator::Agent', name='agent',
            namespace="/" + agent_name, parameters=agent_emulator_params
        )
        agent_nodes.append(agent_emulator_node)

        #########################################################################
        # Noisy Localization
        #########################################################################
        # Parameters
        noisy_localization_params = [{
            "position_stddev": 0.5,
            "orientation_stddev": deg_to_rad(0.5),
            "velocity_linear_stddev": 0.2,
            "velocity_angular_stddev": deg_to_rad(0.2),
            "rate": 20.0,
        }]
        noisy_localization_node = ComposableNode(
            package='noisy_localization', plugin='noisy_localization::NoisyLocalization',
            name='noisy_localization', namespace="/" + agent_name,
            parameters=noisy_localization_params
        )
        agent_nodes.append(noisy_localization_node)

        #########################################################################
        # Noisy Odometry
        #########################################################################
        # Parameters
        noisy_odometry_params = [{
            "position_drift_noise": 0.01,
            "position_drift_vel_rate": 0.02,
            "orientation_drift_noise": 0.02,
            "orientation_drift_vel_rate": 0.01,
            "rate": 20.0,
        }]
        noisy_odometry_node = ComposableNode(
            package='noisy_localization', plugin='noisy_localization::NoisyOdometry',
            name='noisy_odometry', namespace="/" + agent_name,
            parameters=noisy_odometry_params
        )
        agent_nodes.append(noisy_odometry_node)

        #########################################################################
        # Velocity controller
        #########################################################################
        # Parameters
        triangular_velocity_controller_params = [{
            'acceleration': 1.0,
            'period': 5.0,
            'offset': 2.0,
            'rate': 10.0,
        }]
        random_velocity_controller_params = [{
            'velocity_mean': 3.0,
            'velocity_stddev': 1.0,
            'rate': 10.0,
        }]
        random_acceleration_controller_params = [{
            'min_velocity': 1.0,
            'max_velocity': 3.0,
            'acceleration_stddev': 0.2,
            'rate': 10.0,
        }]
        velocity_controller_node = ComposableNode(
            package='agent_velocity_controller', plugin='agent_velocity_controller::RandomAccelerationController',
            name='velocity_controller', namespace='/' + agent_name,
            parameters=random_acceleration_controller_params
        )
        # agent_nodes.append(velocity_controller_node)

        #########################################################################
        # Constant velocity trigger
        #########################################################################
        velocity_cmd = ['ros2', 'topic', 'pub', '-1',
            '/{}/target_velocity'.format(agent_name),
            'geometry_msgs/msg/Twist',
            "{'linear': {'x': 4.0 }, 'angular': {'z': 0.5} }",
        ]
        ld.add_action(TimerAction(period=5.0, actions=[
            ExecuteProcess(cmd=velocity_cmd, name='{}_velocity_pub'.format(agent_name))
        ]))

        #########################################################################
        # Node container
        #########################################################################
        agent_container = ComposableNodeContainer(
            name=agent_name + '_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=agent_nodes,
            output='screen',
        )
        ld.add_action(agent_container)

    # Load agent_visualization node
    agent_viz_params = [{
        "pose_topics": ["/agent_{:02d}/pose".format(i) for i in range(len(initial_poses))],
    }]
    agent_viz_node = Node(
                    package='agent_emulator', executable='agent_viz', name='agent_viz',
                    output='screen', parameters=agent_viz_params
                   )
    ld.add_action(agent_viz_node)

    rviz_config_file = os.path.join(get_package_share_directory('agent_emulator'), 'launch/agents.rviz')
    rviz_node = Node(
                    package='rviz2', executable='rviz2', name='rviz2',
                    output='screen', arguments=['-d', rviz_config_file]
                   )
    ld.add_action(rviz_node)

    return ld


def main():
    ld = generate_launch_description()

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()
