#!/usr/bin/env python3

import os
from launch.actions import DeclareLaunchArgument

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

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

  use_sim_time = LaunchConfiguration('use_sim_time', default='true')
  x_pose = LaunchConfiguration('x_pose', default='-2.0')
  y_pose = LaunchConfiguration('y_pose', default='-0.5')
  entity_name = LaunchConfiguration('entity_name', default='tb3_one')
  robot_namespace = LaunchConfiguration('robot_namespace', default='tb3_1')

  x_pose2 = LaunchConfiguration('x_pose2', default='0.5')
  y_pose2 = LaunchConfiguration('y_pose2', default='0.5')
  entity_name2 = LaunchConfiguration('entity_name2', default='tb3_two')
  robot_namespace2 = LaunchConfiguration('robot_namespace2', default='tb3_2')

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

  ld = LaunchDescription()

  ld.add_action(gzserver_cmd)
  ld.add_action(gzclient_cmd)

  ld.add_action(robot_state_publisher_cmd)

  ld.add_action(start_gazebo_ros_spawner_cmd)
  ld.add_action(start_gazebo_ros_spawner_cmd2)

  return ld
