#!/usr/bin/python3

""" Launch localization and visualization for carla controlled agents """

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

    # Number of agents
    N = 20

    # Load agent nodes
    for i in range(N):
        agent_name = "loco_agent_{:02d}".format(i)
        agent_nodes = []

        #########################################################################
        # Noisy Localization
        #########################################################################
        # Parameters
        noisy_localization_params = [{
            "position_stddev": 2.0,
            "orientation_stddev": deg_to_rad(0.5),
            "velocity_linear_stddev": 1.0,
            "velocity_angular_stddev": deg_to_rad(0.2),
            "rate": 20.0,
        }]
        noisy_localization_node = ComposableNode(
            package='noisy_localization', plugin='noisy_localization::NoisyLocalization',
            name='noisy_localization', namespace="/carla/" + agent_name,
            parameters=noisy_localization_params
        )
        # agent_nodes.append(noisy_localization_node)

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
        # ld.add_action(agent_container)

    # Load agent_visualization node
    agent_viz_params = [{
        "odom_topics": ["loco_agent_{:02d}/odom".format(i) for i in range(N)],
    }]
    agent_viz_node = Node(
                    package='agent_emulator', executable='carla_agent_viz', name='carla_agent_viz',
                    namespace='/carla', output='screen', parameters=agent_viz_params
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
