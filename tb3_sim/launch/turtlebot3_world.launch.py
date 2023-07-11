#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Joep Tool

import os
from launch.actions import DeclareLaunchArgument

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

  # Get the urdf file
  TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
  model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
  urdf_path = os.path.join(
      get_package_share_directory('turtlebot3_gazebo'),
      'models',
      model_folder,
      'model.sdf'
  )

  launch_file_dir = os.path.join(
      get_package_share_directory('turtlebot3_gazebo'), 'launch')
  pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
  pkg_tb3_sim = get_package_share_directory('tb3_sim')

  use_sim_time = LaunchConfiguration('use_sim_time', default='true')
  x_pose = LaunchConfiguration('x_pose')
  y_pose = LaunchConfiguration('y_pose')
  entity_name = LaunchConfiguration('entity_name')
  robot_namespace = LaunchConfiguration('robot_namespace')

  declare_x_position_cmd = DeclareLaunchArgument(
      'x_pose', default_value='-2.0',
      description='Specify namespace of the robot')

  declare_y_position_cmd = DeclareLaunchArgument(
      'y_pose', default_value='-0.5',
      description='Specify namespace of the robot')

  declare_entity_name_cmd = DeclareLaunchArgument(
      'entity_name', default_value='tb_t',
      description='Specify namespace of the robot')

  declare_robot_namespace_cmd = DeclareLaunchArgument(
      'robot_namespace', default_value='tb_nst',
      description='Specify namespace of the robot')

  # x_pose2 = LaunchConfiguration('x_pose', default='0.5')
  # y_pose2 = LaunchConfiguration('y_pose', default='0.5')
  # entity_name2 = LaunchConfiguration('entity_name', default='tb_second')
  # robot_namespace2 = LaunchConfiguration('robot_namespace', default='tb_ns2')

  x_pose2 = LaunchConfiguration('x_pose2', default='0.5')
  y_pose2 = LaunchConfiguration('y_pose2', default='0.5')
  entity_name2 = LaunchConfiguration(
      'entity_name2', default='tb_secondttttttttt')
  robot_namespace2 = LaunchConfiguration(
      'robot_namespace2', default='tb_nstttttttt')

  # declare_x_position_cmd2 = DeclareLaunchArgument(
  #     'x_pose2', default_value='-0.5',
  #     description='Specify namespace of the robot')

  # declare_y_position_cmd2 = DeclareLaunchArgument(
  #     'y_pose2', default_value='0.5',
  #     description='Specify namespace of the robot')

  # declare_entity_name_cmd2 = DeclareLaunchArgument(
  #     'entity_name2', default_value='tb_t2',
  #     description='Specify namespace of the robot')

  # declare_robot_namespace_cmd2 = DeclareLaunchArgument(
  #     'robot_namespace2', default_value='tb_nst2',
  #     description='Specify namespace of the robot')

  world = os.path.join(
      get_package_share_directory('turtlebot3_gazebo'),
      'worlds',
      'turtlebot3_world.world'
  )

  gzserver_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
      ),
      launch_arguments={'world': world}.items()
  )

  gzclient_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
      )
  )

  robot_state_publisher_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
      ),
      launch_arguments={'use_sim_time': use_sim_time}.items()
  )

  start_gazebo_ros_spawner_cmd = Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      arguments=[
          '-entity', entity_name,
          '-file', urdf_path,
          '-robot_namespace', robot_namespace,
          '-x', x_pose,
          '-y', y_pose,
          '-z', '0.01'
      ],
      output='screen',
  )

  start_gazebo_ros_spawner_cmd2 = Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      arguments=[
          '-entity', entity_name2,
          '-file', urdf_path,
          '-robot_namespace', robot_namespace2,
          '-x', x_pose2,
          '-y', y_pose2,
          '-z', '0.01'
      ],
      output='screen',
  )

  # spawn_turtlebot_cmd = IncludeLaunchDescription(
  #     PythonLaunchDescriptionSource(
  #         os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
  #     ),
  #     launch_arguments={
  #         'x_pose': x_pose,
  #         'y_pose': y_pose,
  #         'entity_name': entity_name,
  #         'robot_namespace': robot_namespace
  #     }.items()
  # )

  # spawn_turtlebot_cmd2 = IncludeLaunchDescription(
  #     PythonLaunchDescriptionSource(
  #         os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
  #     ),
  #     launch_arguments={
  #         'x_pose': x_pose2,
  #         'y_pose': y_pose2,
  #         'entity_name': entity_name2,
  #         'robot_namespace': robot_namespace2
  #     }.items()
  # )

  # spawn_turtlebot_cmd2 = IncludeLaunchDescription(
  #     PythonLaunchDescriptionSource(
  #         os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
  #     ),
  #     launch_arguments={
  #         'x_pose': x_pose2,
  #         'y_pose': y_pose2,
  #         'entity_name': entity_name2,
  #         'robot_namespace': robot_namespace2
  #     }.items()
  # )

  ld = LaunchDescription()

  ld.add_action(gzserver_cmd)
  ld.add_action(gzclient_cmd)

  ld.add_action(declare_x_position_cmd)
  ld.add_action(declare_y_position_cmd)
  ld.add_action(declare_entity_name_cmd)
  ld.add_action(declare_robot_namespace_cmd)

  # ld.add_action(declare_x_position_cmd2)
  # ld.add_action(declare_y_position_cmd2)
  # ld.add_action(declare_entity_name_cmd2)
  # ld.add_action(declare_robot_namespace_cmd2)

  # ld.add_action(robot_state_publisher_cmd)
  # ld.add_action(spawn_turtlebot_cmd)
  # ld.add_action(spawn_turtlebot_cmd2)
  ld.add_action(start_gazebo_ros_spawner_cmd)
  ld.add_action(start_gazebo_ros_spawner_cmd2)

  return ld
