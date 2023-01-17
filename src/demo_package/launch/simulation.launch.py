import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'demo_package'
    building_robot = 'description/demo_motor_simulation.urdf.xacro'

    xacro_file = os.path.join(get_package_share_directory(pkg_name), building_robot)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    joint_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True}]
    )

    joint_state_gui = Node(
      name='joint_state_publisher_gui',
      package='joint_state_publisher_gui',
      executable='joint_state_publisher_gui',
      output='screen',
      arguments=[])

    rviz2 = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        arguments=[])

    rqt = Node(
        name='rqt_gui',
        package='rqt_gui',
        executable='rqt_gui',
        namespace='',
        arguments=[])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-entity', 'demo_motor_simulation'])

    return LaunchDescription([
        gazebo,
        joint_state_publisher,
        #joint_state_gui,
        rviz2,
        rqt,
        spawn_entity
    ])
