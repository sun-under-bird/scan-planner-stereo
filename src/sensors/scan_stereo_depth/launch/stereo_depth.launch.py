"""Rectified D435i infrared stereo to metric depth; no camera driver or motion."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    config = os.path.join(get_package_share_directory('scan_stereo_depth'), 'config', 'stereo.yaml')
    return [Node(package='scan_stereo_depth', executable='stereo_depth_node', name='stereo_depth_node',
                 output='screen', parameters=[config, {
                     'method': value('method'), 'downsample': int(value('downsample')),
                     'opencv_threads': int(value('opencv_threads')),
                     'num_disparities': int(value('num_disparities'))}],
                 remappings=[('left/image_rect', value('left_image')),
                             ('right/image_rect', value('right_image')),
                             ('left/camera_info', value('left_info')),
                             ('right/camera_info', value('right_info')),
                             ('depth', value('depth_topic')), ('camera_info', value('depth_info_topic')),
                             ('statistics', value('statistics_topic'))])]


def generate_launch_description():
    defaults = {
        'left_image': '/camera/camera/infra1/image_rect_raw',
        'right_image': '/camera/camera/infra2/image_rect_raw',
        'left_info': '/camera/camera/infra1/camera_info',
        'right_info': '/camera/camera/infra2/camera_info',
        'depth_topic': '/scan/stereo/depth', 'depth_info_topic': '/scan/stereo/camera_info',
        'statistics_topic': '/scan/stereo/statistics', 'method': 'bm', 'downsample': '1',
        'opencv_threads': '2', 'num_disparities': '96',
    }
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
