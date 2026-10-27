"""D435i software stereo + timestamped TF + /odom + optional existing cmd_vel bridge."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    enabled = lambda name: value(name).lower() in ('true', '1', 'yes')
    share = get_package_share_directory('scan_planner')
    params = [os.path.join(share, 'config', 'planner.yaml'),
              os.path.join(share, 'config', 'passive_stereo.yaml')]
    if value('params_file'):
        params.append(value('params_file'))
    mode = int(value('navi_mode'))
    if mode not in (1, 2, 3):
        raise RuntimeError('navi_mode must be 1, 2, or 3')
    if mode == 2:
        if not os.path.isfile(value('keypoints_file')):
            raise RuntimeError('navi_mode=2 needs keypoints_file')
        params.append(value('keypoints_file'))
    params.append({'grid_map.frame_id': value('planning_frame'), 'fsm.navi_mode': mode})
    actions = []
    if enabled('compute_depth'):
        stereo_share = get_package_share_directory('scan_stereo_depth')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(stereo_share, 'launch', 'stereo_depth.launch.py')),
            launch_arguments={k: value(k) for k in (
                'left_image', 'right_image', 'left_info', 'right_info', 'depth_topic', 'depth_info_topic',
                'statistics_topic', 'method', 'downsample', 'opencv_threads', 'num_disparities')}.items()))
    actions.append(Node(package='scan_planner', executable='scan_planner_node',
                        name='scan_planner_node', output='screen', parameters=params,
                        remappings=[('body_pose', value('odom_topic')), ('depth', value('depth_topic')),
                                    ('depth_camera_info', value('depth_info_topic')),
                                    ('move_base_simple/goal', value('goal_topic')),
                                    ('initial_path', value('path_topic'))]))
    if enabled('start_controller'):
        actions.append(Node(package='scan_planner', executable='closed_loop_controller',
                            name='closed_loop_controller', output='screen',
                            parameters=[os.path.join(share, 'config', 'controllers.yaml'), {
                                'planning_frame': value('planning_frame'), 'odom_timeout': 0.5,
                                'require_depth': True}],
                            remappings=[('body_pose', value('odom_topic')), ('cmd_vel', value('cmd_vel_topic'))]))
    return actions


def generate_launch_description():
    defaults = {
        'odom_topic': '/odom', 'planning_frame': 'odom', 'cmd_vel_topic': '/cmd_vel',
        'goal_topic': '/move_base_simple/goal', 'path_topic': '/initial_path',
        'depth_topic': '/scan/stereo/depth', 'depth_info_topic': '/scan/stereo/camera_info',
        'statistics_topic': '/scan/stereo/statistics',
        'left_image': '/camera/camera/infra1/image_rect_raw',
        'right_image': '/camera/camera/infra2/image_rect_raw',
        'left_info': '/camera/camera/infra1/camera_info',
        'right_info': '/camera/camera/infra2/camera_info',
        'compute_depth': 'true', 'start_controller': 'false', 'navi_mode': '1',
        'params_file': '', 'keypoints_file': '', 'method': 'bm', 'downsample': '1',
        'opencv_threads': '2', 'num_disparities': '96',
    }
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
