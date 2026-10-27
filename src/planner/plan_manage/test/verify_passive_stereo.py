#!/usr/bin/env python3
"""Isolated board integration checks. No physical camera or production cmd_vel."""
import os
os.environ['ROS_DOMAIN_ID'] = '228'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ.pop('ROS_LOCALHOST_ONLY', None)
os.environ['CYCLONEDDS_URI'] = '<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
import argparse
import datetime
import json
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import yaml
import rclpy
from rclpy.qos import qos_profile_sensor_data as qos
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import TransformStamped, Twist, Point, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from scan_planner_msgs.msg import Bspline


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', default=str(Path(__file__).resolve().parents[4]))
    parser.add_argument('--goal-distance', type=float, default=1.5)
    parser.add_argument('--binary-workspace', default=None)
    parser.add_argument('--start-speed', type=float, default=0.0)
    parser.add_argument('--point-filter', action='store_true')
    args = parser.parse_args()
    root = Path(args.workspace)
    binaries = Path(args.binary_workspace) if args.binary_workspace else root
    out = root / 'log' / ('passive-verification-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    out.mkdir()
    processes, logfiles, checks = [], [], []
    ns = '/passive_check'
    node = None
    data = {'depth': [], 'maps': [], 'pose': [], 'ready': [], 'cmd': [], 'trajectory': []}

    def start(binary, name, params, remaps=()):
        logfile = (out / (name + '.log')).open('w'); logfiles.append(logfile)
        command = [str(binary), '--ros-args', '-r', '__ns:=' + ns]
        if isinstance(params, Path):
            command += ['--params-file', str(params)]
        else:
            for key, value in params.items():
                command += ['-p', key + ':=' + str(value).lower()]
        for key, value in remaps:
            command += ['-r', key + ':=' + value]
        process = subprocess.Popen(command, stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    def stop(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=3)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.02, end - time.monotonic())))

    def check(name, condition, detail=None):
        checks.append({'name': name, 'passed': bool(condition), 'detail': detail})
        print(json.dumps(checks[-1]), flush=True)
        if not condition: raise AssertionError(name)

    def image(array, stamp, frame='optical'):
        message = Image(); message.header.stamp = stamp; message.header.frame_id = frame
        message.height, message.width = array.shape
        message.encoding = {np.dtype('uint16'): '16UC1', np.dtype('float32'): '32FC1', np.dtype('uint8'): 'mono8'}[array.dtype]
        message.step = array.strides[0]; message.data = array.tobytes()
        return message

    def info(width, height, frame='optical', fx=200.0, tx=0.0):
        message = CameraInfo(); message.header.frame_id = frame
        message.width, message.height = width, height
        message.p = [fx, 0., (width - 1)/2, tx, 0., fx, (height - 1)/2, 0., 0., 0., 1., 0.]
        return message

    def odometry(stamp):
        message = Odometry(); message.header.stamp = stamp; message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'; message.pose.pose.position.z = 0.5
        message.pose.pose.orientation.w = 1.
        message.twist.twist.linear.x = args.start_speed
        return message

    def points(message):
        field = next(f for f in message.fields if f.name == 'x')
        return np.ndarray((message.width * message.height,), dtype='<f4', buffer=message.data,
                          offset=field.offset, strides=(message.point_step,))

    result = {'output': str(out), 'domain': 228, 'physical_camera': False, 'checks': checks}
    def interrupted(signum, frame): raise KeyboardInterrupt
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP): signal.signal(sig, interrupted)
    try:
        rclpy.init(); node = rclpy.create_node('passive_test_driver', namespace=ns)
        subscriptions = [
            node.create_subscription(Image, ns+'/stereo_depth', lambda m: data['depth'].append(m), qos),
            node.create_subscription(PointCloud2, ns+'/grid_map/occupancy', lambda m: data['maps'].append(m), qos),
            node.create_subscription(Odometry, ns+'/grid_map/sensor_pose_extrinsic', lambda m: data['pose'].append(m), 10),
            node.create_subscription(Bool, ns+'/grid_map/depth_ready', lambda m: data['ready'].append(m.data), 1),
            node.create_subscription(Twist, ns+'/observed_cmd_vel', lambda m: data['cmd'].append(m), 10),
            node.create_subscription(Bspline, ns+'/planning/bspline', lambda m: data['trajectory'].append(m), 10)]
        publishers = {name: node.create_publisher(kind, ns+'/'+name,
                      1 if kind == Bool else (10 if kind == Bspline else qos))
                      for name, kind in [('left/image_rect', Image), ('right/image_rect', Image),
                                         ('left/camera_info', CameraInfo), ('right/camera_info', CameraInfo),
                                         ('depth', Image), ('depth_camera_info', CameraInfo), ('body_pose', Odometry),
                                         ('controller_ready', Bool), ('planning/bspline', Bspline)]}
        goal_pub = node.create_publisher(PoseStamped, ns+'/move_base_simple/goal', 1)
        rng = np.random.default_rng(8)
        left = rng.integers(0, 256, (240, 320), dtype=np.uint8)
        right = np.roll(left, -8, axis=1)
        stereo_binary = binaries/'install/scan_stereo_depth/lib/scan_stereo_depth/stereo_depth_node'
        for scale, disparities in [(1, 48), (2, 32)]:
            data['depth'].clear()
            stereo = start(stereo_binary, 'stereo_scale%d'%scale,
                           {'downsample': scale, 'num_disparities': disparities, 'block_size': 9},
                           [('depth', ns+'/stereo_depth'), ('camera_info', ns+'/stereo_info')])
            spin(0.7)
            for _ in range(15):
                stamp = node.get_clock().now().to_msg()
                publishers['left/camera_info'].publish(info(320,240))
                # RealSense can express right CameraInfo in the left reference frame.
                publishers['right/camera_info'].publish(info(320,240, tx=-10.0))
                publishers['left/image_rect'].publish(image(left,stamp))
                publishers['right/image_rect'].publish(image(right,stamp,'right_optical'))
                spin(0.08)
            check('stereo_output_scale%d'%scale, bool(data['depth']))
            message = data['depth'][-1]
            z = np.frombuffer(message.data,dtype='<u2').reshape(message.height,message.width)
            values=z[:,60//scale:-12//scale]; values=values[values>0]
            check('metric_depth_scale%d'%scale, len(values)>100 and abs(float(np.median(values))-1250)<4,
                  float(np.median(values)) if len(values) else None)
            check('depth_metadata_scale%d'%scale, message.header.frame_id=='optical' and
                  message.encoding=='16UC1' and message.width==320//scale and message.height==240//scale)
            stop(stereo); spin(0.2)

        params = yaml.safe_load((root/'src/planner/plan_manage/config/planner.yaml').read_text())['scan_planner_node']['ros__parameters']
        params.update({'grid_map.sensor_type':'depth','grid_map.depth_pose_source':'tf',
                       'grid_map.need_extrinsic':False,'grid_map.use_camera_info':True,
                       'grid_map.invalid_depth_is_free':False,'grid_map.frame_id':'odom',
                       'fsm.odom_twist_in_child_frame':True,'grid_map.resolution':0.1,
                       'grid_map.sliding_map_size_x':8.,'grid_map.sliding_map_size_y':8.,'grid_map.sliding_map_size_z':3.,
                       'grid_map.skip_pixel':2})
        if args.point_filter:
            profile=yaml.safe_load((root/'src/planner/plan_manage/config/passive_stereo.yaml').read_text())['scan_planner_node']['ros__parameters']
            params.update(profile)
        config=out/'planner.yaml'; config.write_text(yaml.safe_dump({ns+'/scan_planner_node':{'ros__parameters':params}}))
        planner=start(binaries/'install/scan_planner/lib/scan_planner/scan_planner_node','planner',config)
        static = StaticTransformBroadcaster(node)
        transform=TransformStamped(); transform.header.frame_id='odom';transform.child_frame_id='optical'
        transform.transform.translation.z=0.5
        transform.transform.rotation.x=-0.5; transform.transform.rotation.y=0.5
        transform.transform.rotation.z=-0.5; transform.transform.rotation.w=0.5
        ground=TransformStamped();ground.header.frame_id='odom';ground.child_frame_id='base_footprint'
        ground.transform.rotation.w=1.
        static.sendTransform([transform,ground])
        spin(0.8)
        width,height=1280,720
        calibration=info(width,height,fx=800.0)
        def feed(array, count=10):
            for _ in range(count):
                stamp=node.get_clock().now().to_msg()
                publishers['depth_camera_info'].publish(calibration)
                publishers['body_pose'].publish(odometry(stamp))
                publishers['depth'].publish(image(array,stamp))
                spin(0.1)
                if planner.poll() is not None: raise RuntimeError('planner exited')
        feed(np.full((height,width),2000,dtype=np.uint16))
        check('large_depth_buffer_1280x720', planner.poll() is None)
        check('tf_depth_occupancy',bool(data['maps']) and np.count_nonzero(abs(points(data['maps'][-1])-2.)<0.12)>0)
        check('tf_camera_pose',bool(data['pose']) and data['pose'][-1].child_frame_id=='optical')
        before=np.count_nonzero(abs(points(data['maps'][-1])-2.)<0.12)
        feed(np.zeros((height,width),dtype=np.uint16),10)
        after=np.count_nonzero(abs(points(data['maps'][-1])-2.)<0.12)
        check('invalid_zero_does_not_clear_obstacles',before==after,[int(before),int(after)])
        check('invalid_depth_blocks_controller_health', bool(data['ready']) and not data['ready'][-1])
        # Keep the 1280x720 uint16 buffer regression above; smaller float messages
        # avoid Python's list serialization dominating the exposure timestamp age.
        width,height=640,360
        calibration=info(width,height,fx=400.0)
        feed(np.full((height,width),np.nan,dtype=np.float32),3)
        check('nan_does_not_clear_obstacles',np.count_nonzero(abs(points(data['maps'][-1])-2.)<0.12)==before)
        feed(np.full((height,width),2.8,dtype=np.float32),15)
        check('float_depth_metres',np.count_nonzero(abs(points(data['maps'][-1])-2.8)<0.12)>0,
              [float(np.min(points(data['maps'][-1]))),float(np.max(points(data['maps'][-1])))])
        check('valid_depth_clears_old_obstacle',np.count_nonzero(abs(points(data['maps'][-1])-2.)<0.12)<before)
        goal=PoseStamped();goal.header.frame_id='odom';goal.pose.position.x=args.goal_distance;goal.pose.position.z=0.5
        goal.pose.orientation.w=1.;goal_pub.publish(goal)
        feed(np.full((height,width),2.8,dtype=np.float32),15)
        check('planner_produces_bspline',bool(data['trajectory']))
        planned = data['trajectory'][-1]
        control = np.array([[p.x,p.y,p.z] for p in planned.pos_pts])
        knots = np.array(planned.knots)
        degree = int(planned.order)
        peaks = []
        for _ in range(2):
            derivative = np.array([degree*(control[i+1]-control[i])/(knots[i+degree+1]-knots[i+1])
                                   for i in range(len(control)-1)])
            peaks.append(float(np.max(np.linalg.norm(derivative,axis=1))))
            control,knots,degree = derivative,knots[1:-1],degree-1
        tolerance = params['manager.feasibility_tolerance']
        check('trajectory_vector_limits',
              peaks[0] <= params['manager.max_vel']*(1+tolerance)+1e-4 and
              peaks[1] <= params['manager.max_acc']*(1+tolerance)+1e-4,
              {'velocity_control_bound':peaks[0],'acceleration_control_bound':peaks[1]})

        dynamic=TransformBroadcaster(node)
        from rclpy.duration import Duration
        old_stamp=(node.get_clock().now()-Duration(seconds=0.1)).to_msg()
        future_stamp=(node.get_clock().now()+Duration(seconds=0.1)).to_msg()
        a=TransformStamped();a.header.frame_id='odom';a.child_frame_id='timed_optical'
        a.header.stamp=old_stamp;a.transform.translation.x=1.;a.transform.translation.z=0.5
        a.transform.rotation=transform.transform.rotation
        b=TransformStamped();b.header.frame_id='odom';b.child_frame_id='timed_optical'
        b.header.stamp=future_stamp;b.transform.translation.x=3.;b.transform.translation.z=0.5
        b.transform.rotation=transform.transform.rotation
        dynamic.sendTransform([a,b]);spin(0.07)
        publishers['depth_camera_info'].publish(info(320,240,frame='timed_optical'))
        for _ in range(3):
            publishers['depth'].publish(image(np.full((240,320),2000,dtype=np.uint16),old_stamp,'timed_optical'))
            spin(0.07)
        timed=[m for m in data['pose'] if m.child_frame_id=='timed_optical']
        check('tf_uses_exposure_time_not_latest',bool(timed) and abs(timed[-1].pose.pose.position.x-1.)<1e-4)
        publishers['depth_camera_info'].publish(info(320,240,frame='missing_optical'))
        publishers['depth'].publish(image(np.full((240,320),2000,dtype=np.uint16),
                                    node.get_clock().now().to_msg(),'missing_optical'))
        spin(0.35)
        check('missing_tf_is_rejected',not any(m.child_frame_id=='missing_optical' for m in data['pose']))
        stop(planner)

        controller=start(binaries/'install/scan_planner/lib/scan_planner/closed_loop_controller', 'controller',
                         {'require_depth': True,'odom_timeout':0.3,'planning_frame':'odom'},
                         [('cmd_vel',ns+'/observed_cmd_vel'),('grid_map/depth_ready',ns+'/controller_ready')])
        spin(0.7)
        trajectory=Bspline();trajectory.order=3;trajectory.traj_id=1
        trajectory.pos_pts=[Point(x=x,y=0.,z=0.5) for x in (0.,0.5,1.,1.5)]
        trajectory.knots=[0.,0.,0.,0.,2.,2.,2.,2.]
        for _ in range(3):
            publishers['body_pose'].publish(odometry(node.get_clock().now().to_msg()))
            publishers['planning/bspline'].publish(trajectory);spin(0.05)
        check('controller_waits_for_depth',bool(data['cmd']) and abs(data['cmd'][-1].linear.x)<1e-6)
        for _ in range(8):
            publishers['body_pose'].publish(odometry(node.get_clock().now().to_msg()))
            publishers['controller_ready'].publish(Bool(data=True));spin(0.05)
        check('controller_twist_output',any(m.linear.x>0.01 for m in data['cmd']))
        spin(0.6)
        check('controller_stops_on_stale_inputs',abs(data['cmd'][-1].linear.x)<1e-6 and abs(data['cmd'][-1].angular.z)<1e-6)
        stop(controller)
        result['passed']=True
    except BaseException as error:
        result['passed']=False;result['error']=repr(error)
        raise
    finally:
        for process in reversed(processes):stop(process)
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for logfile in logfiles:logfile.close()
        result['all_owned_processes_stopped']=all(p.poll() is not None for p in processes)
        (out/'result.json').write_text(json.dumps(result,indent=2))
        print('Verification report:',out/'result.json',flush=True)


if __name__=='__main__':main()
