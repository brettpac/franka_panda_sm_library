# Copyright 2021 RobosoftAI Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

import yaml

def generate_launch_description():

 #   declared_arguments = []
 #   declared_arguments.append(
 #       DeclareLaunchArgument(
 #           "rviz_config",
 #           default_value="panda_moveit_config_demo.rviz",
 #           description="RViz configuration file",
 #      )
 #   )

 #    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

    ros2_control_hardware_type = DeclareLaunchArgument(
        'ros2_control_hardware_type',
        default_value='isaac',
        description=(
            'ROS2 control hardware interface type to use for the launch file -- '
            'possible values: [mock_components, isaac]'
        )
    )


    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path='config/panda.urdf.xacro',
            mappings={
                'ros2_control_hardware_type': LaunchConfiguration(
                    'ros2_control_hardware_type'
                )
            },
        )
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        # This block is where the action is
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        #end of where the action is. 
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

        # Add cuMotion to list of planning pipelines.
    cumotion_config_file_path = os.path.join(
        get_package_share_directory('isaac_ros_cumotion_moveit'),
        'config',
        'isaac_ros_cumotion_planning.yaml'
    )
    with open(cumotion_config_file_path) as cumotion_config_file:
        cumotion_config = yaml.safe_load(cumotion_config_file)
    moveit_config.planning_pipelines['planning_pipelines'].append('isaac_ros_cumotion')
    moveit_config.planning_pipelines['isaac_ros_cumotion'] = cumotion_config

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        prefix="xterm -hold -e",
    )

    #rviz_base = LaunchConfiguration("rviz_config")
    #rviz_config = PathJoinSubstitution(
    #    [FindPackageShare("sm_panda_single_1"), "launch", rviz_base]
    #)

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory('sm_panda_single_1'),
        'rviz',
   #     'panda_moveit_config_demo.rviz',
        'franka_moveit_config.rviz',
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    world2robot_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['--frame-id', 'world', '--child-frame-id', 'panda_link0'],
    )
    hand2camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=[
            '0.04',
            '0.0',
            '0.04',
            '0.0',
            '0.0',
            '0.0',
            'panda_hand',
            'sim_camera',
        ],
    )
    # Static TF
    #static_tf = Node(
    #    package="tf2_ros",
    #    executable="static_transform_publisher",
    #    name="static_transform_publisher",
    #    output="log",
    #    arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    #)

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
        prefix="xterm -hold -e",
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
   # ros2_control_node = Node(
   #     package="controller_manager",
   #     executable="ros2_control_node",
   #     parameters=[moveit_config.robot_description, ros2_controllers_path],
   #     output="both",
   # )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
        prefix="xterm -hold -e",
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
        prefix="xterm -hold -e",
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
        prefix="xterm -hold -e",
    )

    smacc_state_machine_spawner = Node(
        package="sm_panda_single_1",
        executable="sm_panda_single_1_node",
        prefix="xterm -hold -e",
        output="screen",
    )

    keyboard_client_node = Node(
        package="keyboard_client",
        executable="keyboard_server_node.py",
        name="keyboard_client",
        output="screen",
        prefix="xterm -hold -e",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller", "-c", "/controller_manager"],
        prefix="xterm -hold -e",
    )

    return LaunchDescription([
        ros2_control_hardware_type,
        smacc_state_machine_spawner,
        rviz_node,
        #static_tf,
        world2robot_tf_node,
        hand2camera_tf_node,
        robot_state_publisher,
        move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        panda_hand_controller_spawner,
        keyboard_client_node,
    ]
)

