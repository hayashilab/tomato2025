#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the share directory of azure_kinect_ros_driver
    pkg_share = get_package_share_directory('azure_kinect_ros_driver')
    urdf_file = os.path.join(pkg_share, 'urdf', 'azure_kinect.urdf.xacro')

    # Create a robot_description parameter from the xacro file
    robot_description = {
        'robot_description': Command(['xacro ', urdf_file])
    }

    # Declare launch arguments (matching your original <arg> tags in XML)
    rgb_rect_arg = DeclareLaunchArgument('rgb_rect', default_value='1')
    depth_rect_arg = DeclareLaunchArgument('depth_rect', default_value='1')
    ir_rect_arg = DeclareLaunchArgument('ir_rect', default_value='1')
    point_cloud_arg = DeclareLaunchArgument('point_cloud', default_value='1')
    rgb_point_cloud_arg = DeclareLaunchArgument('rgb_point_cloud', default_value='1')

    # Joint State Publisher (no namespace)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Robot State Publisher (no namespace)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description]
    )

    # Nodelet manager (namespace='k4a')
    manager_node = Node(
        package='nodelet',
        executable='nodelet',
        namespace='k4a',
        name='manager',
        arguments=['manager'],
        output='screen',
        parameters=[{'num_worker_threads': 16}]
    )

    # Rectify RGB node
    rectify_rgb_node = Node(
        condition=IfCondition(LaunchConfiguration('rgb_rect')),
        package='nodelet',
        executable='nodelet',
        namespace='k4a',
        name='rectify_rgb',
        arguments=['load', 'image_proc/rectify', 'manager', '--no-bond'],
        output='screen',
        remappings=[('image_mono', 'rgb/image_raw'),
                    ('image_rect', 'rgb/image_rect_color')]
    )

    # Rectify Depth node
    rectify_depth_node = Node(
        condition=IfCondition(LaunchConfiguration('depth_rect')),
        package='nodelet',
        executable='nodelet',
        namespace='k4a',
        name='rectify_depth',
        arguments=['load', 'image_proc/rectify', 'manager', '--no-bond'],
        output='screen',
        remappings=[('image_mono', 'depth/image_raw'),
                    ('image_rect', 'depth/image_rect')],
        parameters=[{'interpolation': 0}]
    )

    # Rectify IR node
    rectify_ir_node = Node(
        condition=IfCondition(LaunchConfiguration('ir_rect')),
        package='nodelet',
        executable='nodelet',
        namespace='k4a',
        name='rectify_ir',
        arguments=['load', 'image_proc/rectify', 'manager', '--no-bond'],
        output='screen',
        remappings=[('image_mono', 'ir/image_raw'),
                    ('image_rect', 'ir/image_rect')]
    )

    # Point Cloud XYZ node (used when rgb_point_cloud is false)
    point_cloud_xyz_node = Node(
        condition=UnlessCondition(LaunchConfiguration('rgb_point_cloud')),
        package='nodelet',
        executable='nodelet',
        namespace='k4a',
        name='point_cloud_xyz',
        arguments=['load', 'depth_image_proc/point_cloud_xyz', 'manager', '--no-bond'],
        output='screen',
        remappings=[('image_rect', 'depth/image_rect')]
    )

    # Depth Register node (used when rgb_point_cloud is true)
    depth_register_node = Node(
        condition=IfCondition(LaunchConfiguration('rgb_point_cloud')),
        package='nodelet',
        executable='nodelet',
        namespace='k4a',
        name='depth_register',
        arguments=['load', 'depth_image_proc/register', 'manager', '--no-bond'],
        output='screen'
    )

    # Point Cloud XYZRGB node (used when rgb_point_cloud is true)
    point_cloud_xyzrgb_node = Node(
        condition=IfCondition(LaunchConfiguration('rgb_point_cloud')),
        package='nodelet',
        executable='nodelet',
        namespace='k4a',
        name='point_cloud_xyzrgb',
        arguments=['load', 'depth_image_proc/point_cloud_xyzrgb', 'manager', '--no-bond'],
        output='screen'
    )

    # K4A ROS bridge node
    k4a_ros_bridge_node = Node(
        package='nodelet',
        executable='nodelet',
        namespace='k4a',
        name='k4a_ros_bridge',
        arguments=['load', 'Azure_Kinect_ROS_Driver/K4AROSBridgeNodelet', 'manager', '--no-bond'],
        output='screen',
        parameters=[{
            'depth_enabled': True,
            'depth_mode': 'NFOV_UNBINNED',
            'color_enabled': True,
            'color_resolution': '1536P',
            'fps': 5,
            'point_cloud': True,
            'rgb_point_cloud': True
        }]
    )

    # Build the LaunchDescription
    ld = LaunchDescription()

    # Add all declared arguments
    ld.add_action(rgb_rect_arg)
    ld.add_action(depth_rect_arg)
    ld.add_action(ir_rect_arg)
    ld.add_action(point_cloud_arg)
    ld.add_action(rgb_point_cloud_arg)

    # Add the two top-level publishers
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)

    # Add the "k4a" nodes
    ld.add_action(manager_node)
    ld.add_action(rectify_rgb_node)
    ld.add_action(rectify_depth_node)
    ld.add_action(rectify_ir_node)
    ld.add_action(point_cloud_xyz_node)
    ld.add_action(depth_register_node)
    ld.add_action(point_cloud_xyzrgb_node)
    ld.add_action(k4a_ros_bridge_node)

    return ld
