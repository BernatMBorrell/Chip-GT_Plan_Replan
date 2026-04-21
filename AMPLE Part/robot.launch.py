#!/usr/bin/env python

import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from webots_cmd.control_node import CMD_TYPE, ControllerType
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction

def robot_setup(context, *args, **kwargs):

    ns = LaunchConfiguration('ns').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    controller_type = LaunchConfiguration('controller_type').perform(context)
    max_vel = float(LaunchConfiguration('max_vel').perform(context))

    node_ns = robot_name
    if ns != "":
        node_ns = "/"+ns+"/"+robot_name

    robot_node = Node(
        package="webots_cmd",
        executable="control_node.py",
        namespace=node_ns,
        output="screen",
        remappings=[
            ("/tf",node_ns+"/tf"),
            ("/tf_static",node_ns+"/tf_static"),   
        ],
        respawn=True,
        parameters=[{'use_sim_time': True}],
    )

    controller_execs = []
    if controller_type == str(ControllerType.SIMPLE.value):
        relay_node = Node(
            package="topic_tools",
            executable="relay_field",
            namespace=node_ns,
            output="screen",
            arguments=[
                node_ns+"/odom", 
                node_ns+"/pose",
                "geometry_msgs/msg/PoseStamped",
                "{header: m.header, pose: m.pose.pose}",
                "--wait-for-start"
            ],
            remappings=[
                ("current_goal",node_ns+"/current_goal"),   
            ],
            respawn=True,
            parameters=[{'use_sim_time': True}],
        )
        move_to_node = Node(
            package="move_to",
            executable="move_to_node",
            namespace=node_ns,
            output="screen",
            remappings=[
                ("~/cmd_vel",node_ns+"/cmd_vel"),
                ("/robot/pose",node_ns+"/pose"),   
            ],
            parameters=[
                {"max_speed":max_vel},
                {"position_tolerance":0.3},
                {"use_sim_time":True}
            ],
            respawn=True,
        )

        controller_execs = [relay_node,move_to_node]
    
    elif controller_type == str(ControllerType.NAV2.value):
        nav_launch_arguments = {
            "namespace":node_ns,
            "params_file":params_file
        }
        nav_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("webots_cmd"),
                    "launch",
                    "navigation.launch.py",
                )
            ),
            launch_arguments=nav_launch_arguments.items(),
        )
        controller_execs = [nav_launch]
    
    else:
        raise ValueError(f"Unknown controller_type.")

    detector = Node(
        package="webots_cmd",
        executable="basic_detector.py",
        namespace=node_ns,
        remappings=[
            ("detect",node_ns+"/detect"),
        ],
        respawn=True,
        parameters=[{'use_sim_time': True}],
    )

    disarmer = Node(
        package="webots_cmd",
        executable="basic_disarmer.py",
        namespace=node_ns,
        remappings=[
            ("disarm",node_ns+"/disarm"),
        ],
        respawn=True,
        parameters=[{'use_sim_time': True}],
    )

    return [
        robot_node,
        detector,
        disarmer
    ]+controller_execs

def generate_launch_description():
    ns = DeclareLaunchArgument(
        'ns',
        default_value='',
    )
    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='robot',
    )
    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory("webots_cmd"), 'params', 'nav2_params.yaml'),
    )
    controller_type = DeclareLaunchArgument(
        "controller_type",
        default_value=str(ControllerType.SIMPLE.value),
    )
    max_vel = DeclareLaunchArgument(
        "max_vel",
        default_value="1.5",
    )

    return LaunchDescription([
            ns,
            robot_name,
            params_file,
            controller_type,
            max_vel,
            OpaqueFunction(function=robot_setup),
    ])
