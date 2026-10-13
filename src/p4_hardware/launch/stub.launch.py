import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('p4_hardware')

    xacro_file = os.path.join(pkg, 'urdf', 'p4_stub.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    controllers = os.path.join(pkg, 'config', 'controllers.yaml')

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers],
        output='screen',
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    spawn_broadcaster = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'joint_state_broadcaster'],
        output='screen',
    )

    spawn_position = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'position_controller'],
        output='screen',
    )

    return LaunchDescription([
        control_node,
        robot_state_pub,
        spawn_broadcaster,
        spawn_position,
    ])
