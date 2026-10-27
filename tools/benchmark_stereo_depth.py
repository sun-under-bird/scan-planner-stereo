#!/usr/bin/env python3
"""Board-only live D435i benchmark. All processes and DDS participants are isolated."""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=6)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)


def cpu_seconds(pid):
    fields = Path('/proc/%d/stat' % pid).read_text().rsplit(')', 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')


def percentile(values, q):
    if not values:
        return None
    values = sorted(values)
    return values[int((len(values) - 1) * q)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', default='/home/marsdog/SCAN-Planner')
    parser.add_argument('--camera-executable', required=True)
    parser.add_argument('--camera-lib-dir', action='append', default=[])
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--domain', type=int, default=227)
    parser.add_argument('--output')
    parser.add_argument('--cases', help='Comma-separated case names; default is the three stereo-only cases')
    parser.add_argument('--with-planner', action='store_true', help='Add bm_640_with_mapping with synthetic stationary pose')
    args = parser.parse_args()
    if not 0 <= args.domain <= 230 or args.seconds < 5:
        parser.error('use a valid isolated domain and at least 5 seconds per case')
    for directory in Path('/proc').iterdir():
        if directory.name.isdigit() and int(directory.name) != os.getpid():
            try:
                if ('ROS_DOMAIN_ID=%d' % args.domain).encode() in (directory / 'environ').read_bytes().split(b'\0'):
                    parser.error('requested domain already has a running process: ' + directory.name)
            except (PermissionError, FileNotFoundError, ProcessLookupError):
                pass
    os.environ.update({
        'ROS_DOMAIN_ID': str(args.domain), 'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
        'CYCLONEDDS_URI': '<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces>'
                         '<AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex>'
                         '<Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>',
        'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'})
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    from std_msgs.msg import String, Bool

    root = Path(args.workspace)
    output = Path(args.output) if args.output else root / 'log' / ('stereo-performance-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    output.mkdir(parents=True, exist_ok=False)
    processes, logs = [], []
    report = {'started': datetime.datetime.now().astimezone().isoformat(), 'domain': args.domain,
              'camera': str(args.camera_executable), 'input': 'live D435i IR 640x480@30, emitter off',
              'cases': [], 'output': str(output), 'motion_controller': False}
    node = None
    state = {'measuring': False, 'depth_times': [], 'latencies': [], 'statistics': [],
             'left_times': [], 'map_times': [], 'ready': [], 'left_info': None, 'right_info': None}

    def launch(command, name, env=None):
        logfile = (output / (name + '.log')).open('w')
        logs.append(logfile)
        def pin():
            os.sched_setaffinity(0, {4, 5, 6, 7} & os.sched_getaffinity(0))
        proc = subprocess.Popen(command, stdout=logfile, stderr=subprocess.STDOUT,
                                env=env, start_new_session=True, preexec_fn=pin)
        processes.append(proc)
        return proc

    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.1, until - time.monotonic())))

    def depth(message):
        if state['measuring']:
            state['depth_times'].append(time.monotonic())
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            state['latencies'].append((time.time() - stamp) * 1000)

    def statistics(message):
        if state['measuring']:
            state['statistics'].append(json.loads(message.data))

    def interrupted(signum, frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    try:
        rclpy.init()
        node = rclpy.create_node('stereo_benchmark_observer')
        subscriptions = [
            node.create_subscription(Image, '/bench/depth', depth, qos_profile_sensor_data),
            node.create_subscription(String, '/bench/statistics', statistics, 1),
            node.create_subscription(Image, '/camera/camera/infra1/image_rect_raw',
                lambda msg: state['left_times'].append(time.monotonic()) if state['measuring'] else None,
                qos_profile_sensor_data),
            node.create_subscription(CameraInfo, '/camera/camera/infra1/camera_info',
                lambda msg: state.update(left_info=msg), qos_profile_sensor_data),
            node.create_subscription(CameraInfo, '/camera/camera/infra2/camera_info',
                lambda msg: state.update(right_info=msg), qos_profile_sensor_data),
            node.create_subscription(PointCloud2, '/bench/map',
                lambda msg: state['map_times'].append(time.monotonic()) if state['measuring'] else None,
                qos_profile_sensor_data),
            node.create_subscription(Bool, '/grid_map/depth_ready',
                lambda msg: state['ready'].append(msg.data) if state['measuring'] else None, 1)]
        odom_pub = node.create_publisher(Odometry, '/bench/odom', qos_profile_sensor_data)
        def publish_pose():
            message = Odometry(); message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = 'bench_odom'; message.child_frame_id = 'bench_body'
            message.pose.pose.position.z = 0.5; message.pose.pose.orientation.w = 1.0
            odom_pub.publish(message)
        pose_timer = node.create_timer(0.02, publish_pose)
        pose_tf = StaticTransformBroadcaster(node)
        transform = TransformStamped(); transform.header.frame_id = 'bench_odom'; transform.child_frame_id = 'camera_link'
        transform.transform.translation.z = 0.5; transform.transform.rotation.w = 1.0
        pose_tf.sendTransform(transform)
        camera_env = os.environ.copy()
        camera_env['LD_LIBRARY_PATH'] = camera_env.get('LD_LIBRARY_PATH', '') + ':' + ':'.join(args.camera_lib_dir)
        camera_command = [args.camera_executable, '--ros-args', '-r', '__ns:=/camera', '-r', '__node:=camera',
                          '-p', 'camera_name:=camera', '-p', 'enable_infra1:=true', '-p', 'enable_infra2:=true',
                          '-p', 'depth_module.infra_profile:=640,480,30', '-p', 'enable_depth:=false',
                          '-p', 'enable_color:=false', '-p', 'enable_gyro:=false', '-p', 'enable_accel:=false',
                          '-p', 'enable_sync:=true', '-p', 'depth_module.emitter_enabled:=0',
                          '-p', 'publish_tf:=true', '-p', 'infra1_qos:=SENSOR_DATA', '-p', 'infra2_qos:=SENSOR_DATA']
        report['camera_command'] = camera_command
        camera = launch(camera_command, 'camera', camera_env)
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and (state['left_info'] is None or state['right_info'] is None):
            spin(0.2)
            if camera.poll() is not None:
                raise RuntimeError('camera exited; see camera.log')
        if state['left_info'] is None or state['right_info'] is None:
            raise RuntimeError('no stereo CameraInfo received')
        report['calibration'] = {key: {'frame': state[key].header.frame_id,
                                    'width': state[key].width, 'height': state[key].height,
                                    'p': list(state[key].p)} for key in ('left_info', 'right_info')}
        configurations = [('bm_640_threads2', 'bm', 1, 2, 96, 15),
                          ('bm_320_threads2', 'bm', 2, 2, 48, 15),
                          ('sgbm_640_threads2', 'sgbm', 1, 2, 96, 5)]
        if args.with_planner:
            configurations.append(('bm_640_with_mapping', 'bm', 1, 2, 96, 15))
        if args.cases:
            wanted = set(args.cases.split(','))
            configurations = [c for c in configurations if c[0] in wanted]
            if len(configurations) != len(wanted):
                raise RuntimeError('unknown benchmark case; use --with-planner for the mapping case')
        binary = root / 'install/scan_stereo_depth/lib/scan_stereo_depth/stereo_depth_node'
        for name, method, scale, threads, disparities, block in configurations:
            print('Benchmark starting:', name, flush=True)
            command = [str(binary), '--ros-args']
            for key, value in {'method': method, 'downsample': scale, 'opencv_threads': threads,
                               'num_disparities': disparities, 'block_size': block}.items():
                command += ['-p', str(key) + ':=' + str(value)]
            for key, value in {'left/image_rect': '/camera/camera/infra1/image_rect_raw',
                               'right/image_rect': '/camera/camera/infra2/image_rect_raw',
                               'left/camera_info': '/camera/camera/infra1/camera_info',
                               'right/camera_info': '/camera/camera/infra2/camera_info',
                               'depth': '/bench/depth', 'camera_info': '/bench/camera_info',
                               'statistics': '/bench/statistics'}.items():
                command += ['-r', key + ':=' + value]
            process = launch(command, name)
            planner = None
            if name == 'bm_640_with_mapping':
                import yaml
                params = yaml.safe_load((root/'src/planner/plan_manage/config/planner.yaml').read_text())
                params['scan_planner_node']['ros__parameters'].update({
                    'grid_map.sensor_type': 'depth', 'grid_map.depth_pose_source': 'tf',
                    'grid_map.use_camera_info': True, 'grid_map.need_extrinsic': False,
                    'grid_map.invalid_depth_is_free': False, 'grid_map.frame_id': 'bench_odom',
                    'fsm.odom_twist_in_child_frame': True})
                params_file = output/'mapping.yaml'; params_file.write_text(yaml.safe_dump(params))
                planner = launch([str(root/'install/scan_planner/lib/scan_planner/scan_planner_node'),
                                  '--ros-args', '--params-file', str(params_file),
                                  '-r', 'body_pose:=/bench/odom', '-r', 'depth:=/bench/depth',
                                  '-r', 'depth_camera_info:=/bench/camera_info',
                                  '-r', 'grid_map/occupancy:=/bench/map'], name+'_planner')
            spin(4)
            if process.poll() is not None:
                raise RuntimeError(name + ' exited')
            for key in ('depth_times', 'latencies', 'statistics', 'left_times', 'map_times', 'ready'):
                state[key] = []
            before_cpu = cpu_seconds(process.pid)
            before_planner_cpu = cpu_seconds(planner.pid) if planner else None
            before_time = time.monotonic()
            state['measuring'] = True
            spin(args.seconds)
            state['measuring'] = False
            elapsed = time.monotonic() - before_time
            cpu = (cpu_seconds(process.pid) - before_cpu) / elapsed * 100
            status = Path('/proc/%d/status' % process.pid).read_text()
            rss = next(int(line.split()[1]) for line in status.splitlines() if line.startswith('VmHWM:'))
            samples = state['statistics']
            means = lambda key: sum(s[key] for s in samples) / len(samples) if samples else None
            result = {'name': name, 'method': method, 'downsample': scale, 'opencv_threads': threads,
                      'num_disparities': disparities, 'block_size': block, 'seconds': elapsed,
                      'input_hz': len(state['left_times']) / elapsed,
                      'received_depth_hz': len(state['depth_times']) / elapsed,
                      'frames': len(state['depth_times']), 'cpu_percent_one_core_100': cpu,
                      'peak_rss_MiB': rss / 1024,
                      'receive_latency_ms_mean': sum(state['latencies']) / len(state['latencies']) if state['latencies'] else None,
                      'receive_latency_ms_p95': percentile(state['latencies'], 0.95),
                      'process_ms_mean': means('process_ms_mean'),
                      'worst_window_process_ms_p95': max((s['process_ms_p95'] for s in samples), default=None),
                      'valid_fraction': means('valid_fraction'), 'statistics': samples, 'command': command}
            if planner:
                result['mapper_cpu_percent_one_core_100'] = (cpu_seconds(planner.pid)-before_planner_cpu)/elapsed*100
                result['occupancy_hz'] = len(state['map_times'])/elapsed
                result['depth_ready_fraction'] = sum(state['ready'])/len(state['ready']) if state['ready'] else 0.0
                mapper_status = Path('/proc/%d/status'%planner.pid).read_text()
                result['mapper_peak_rss_MiB'] = next(int(l.split()[1]) for l in mapper_status.splitlines() if l.startswith('VmHWM:'))/1024
                stop(planner)
            report['cases'].append(result)
            print(json.dumps({k: v for k, v in result.items() if k not in ('statistics', 'command')}), flush=True)
            stop(process)
            if result['frames'] == 0:
                raise RuntimeError('no depth frames: ' + name)
        report['passed'] = True
    except BaseException as error:
        report['passed'] = False
        report['error'] = repr(error)
        raise
    finally:
        for process in reversed(processes):
            stop(process)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for logfile in logs:
            logfile.close()
        report['all_owned_processes_stopped'] = all(p.poll() is not None for p in processes)
        report['finished'] = datetime.datetime.now().astimezone().isoformat()
        (output / 'result.json').write_text(json.dumps(report, indent=2))
        print('Performance report:', output / 'result.json', flush=True)


if __name__ == '__main__':
    main()
