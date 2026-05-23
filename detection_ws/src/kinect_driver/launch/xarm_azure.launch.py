# Copyright (c) Microsoft Corporation.
# Modified by Ja Sin, 2025.
# Licensed under the MIT License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    # === Paths ===
    kinect_share = get_package_share_directory("kinect_driver")

    # === Default calibration file path ===
    default_calibration_file = os.path.join(
        kinect_share, "config", "kinect_calibration.yaml"
    )

    # ==============================================================
    # LAUNCH ARGUMENTS
    # ==============================================================
    return LaunchDescription([

        DeclareLaunchArgument(
            'calibration_file',
            default_value=default_calibration_file,
            description="Path to the Kinect calibration file"
        ),
        DeclareLaunchArgument('depth_enabled', default_value='true'),
        DeclareLaunchArgument('depth_mode', default_value='WFOV_UNBINNED'),
        DeclareLaunchArgument('depth_unit', default_value='16UC1'),
        DeclareLaunchArgument('color_enabled', default_value='true'),
        DeclareLaunchArgument('color_format', default_value='bgra'),
        DeclareLaunchArgument('color_resolution', default_value='1536P'),
        DeclareLaunchArgument('fps', default_value='5'),
        DeclareLaunchArgument('point_cloud', default_value='true'),
        DeclareLaunchArgument('rgb_point_cloud', default_value='true'),
        DeclareLaunchArgument('point_cloud_in_depth_frame', default_value='false'),
        DeclareLaunchArgument('sensor_sn', default_value=''),
        DeclareLaunchArgument('recording_file', default_value=''),
        DeclareLaunchArgument('recording_loop_enabled', default_value='false'),
        DeclareLaunchArgument('body_tracking_enabled', default_value='false'),
        DeclareLaunchArgument('body_tracking_smoothing_factor', default_value='0.0'),
        DeclareLaunchArgument('rescale_ir_to_mono8', default_value='false'),
        DeclareLaunchArgument('ir_mono8_scaling_factor', default_value='1.0'),
        DeclareLaunchArgument('imu_rate_target', default_value='0'),
        DeclareLaunchArgument('wired_sync_mode', default_value='0'),
        DeclareLaunchArgument('subordinate_delay_off_master_usec', default_value='0'),

        # ==============================================================
        #  1️⃣ AZURE KINECT DRIVER NODE
        # ==============================================================
        launch_ros.actions.Node(
            package='kinect_driver',
            executable='node',
            name='k4a_ros_device_node',
            output='screen',
            parameters=[
                {'calibration_file': LaunchConfiguration('calibration_file')},
                {'depth_enabled': LaunchConfiguration('depth_enabled')},
                {'depth_mode': LaunchConfiguration('depth_mode')},
                {'depth_unit': LaunchConfiguration('depth_unit')},
                {'color_enabled': LaunchConfiguration('color_enabled')},
                {'color_format': LaunchConfiguration('color_format')},
                {'color_resolution': LaunchConfiguration('color_resolution')},
                {'fps': LaunchConfiguration('fps')},
                {'point_cloud': LaunchConfiguration('point_cloud')},
                {'rgb_point_cloud': LaunchConfiguration('rgb_point_cloud')},
                {'point_cloud_in_depth_frame': LaunchConfiguration('point_cloud_in_depth_frame')},
                {'sensor_sn': LaunchConfiguration('sensor_sn')},
                {'recording_file': LaunchConfiguration('recording_file')},
                {'recording_loop_enabled': LaunchConfiguration('recording_loop_enabled')},
                {'body_tracking_enabled': LaunchConfiguration('body_tracking_enabled')},
                {'body_tracking_smoothing_factor': LaunchConfiguration('body_tracking_smoothing_factor')},
                {'rescale_ir_to_mono8': LaunchConfiguration('rescale_ir_to_mono8')},
                {'ir_mono8_scaling_factor': LaunchConfiguration('ir_mono8_scaling_factor')},
                {'imu_rate_target': LaunchConfiguration('imu_rate_target')},
                {'wired_sync_mode': LaunchConfiguration('wired_sync_mode')},
                {'subordinate_delay_off_master_usec': LaunchConfiguration('subordinate_delay_off_master_usec')}
            ]
        ),
        # comment out as already define in xarm7_robot_macro.xacro
        # ==============================================================
        #  2️⃣ STATIC TRANSFORM (xArm base → Kinect base)
        # ==============================================================
        # launch_ros.actions.Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='xarm_to_kinect_tf',
        #     arguments=[
        #         # translation (x y z)
        #         '-0.18', '-0.28', '0.84',
        #         # rotation (rpy in radians)
        #         '0', '0', '0',
        #         # parent and child frames
        #         'link_base', 'camera_base'
        #     ]
        # )
    ])
