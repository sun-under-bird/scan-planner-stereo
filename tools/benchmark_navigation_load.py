#!/usr/bin/env python3
"""Board-only, isolated D435i -> stereo -> map/planner -> controller load test.

Real camera images are combined with a simulated body pose. This measures software
load, not geometrically consistent navigation, real SLAM, or physical motion.
Requires the built SCAN-Planner workspace, ROS 2 Humble, PyYAML and CycloneDDS.
"""
import argparse
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def percentile(values, fraction):
    return sorted(values)[int((len(values)-1)*fraction)] if values else None


def proc_sample(pid):
    fields = Path('/proc/%d/stat' % pid).read_text().rsplit(')', 1)[1].split()
    status = dict(line.split(':', 1) for line in Path('/proc/%d/status' % pid).read_text().splitlines() if ':' in line)
    return {'cpu_s': (int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK'),
            'rss_mib': int(status.get('VmRSS', '0 kB').split()[0])/1024,
            'hwm_mib': int(status.get('VmHWM', '0 kB').split()[0])/1024,
            'threads': int(status.get('Threads', '0').strip())}


def system_sample():
    cpus = {}
    for line in Path('/proc/stat').read_text().splitlines():
        parts = line.split()
        if parts and parts[0].startswith('cpu'):
            counts = list(map(int, parts[1:9]))  # guest is already included in user/nice.
            cpus[parts[0]] = {'total': sum(counts), 'idle': counts[3]+counts[4]}
    memory = {line.split(':', 1)[0]: int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines()}
    temperatures = {}
    for folder in Path('/sys/class/thermal').glob('thermal_zone*'):
        try:
            temperatures[(folder/'type').read_text().strip()] = int((folder/'temp').read_text())/1000
        except OSError:
            pass
    frequencies = {}
    for folder in Path('/sys/devices/system/cpu/cpufreq').glob('policy*'):
        try:
            frequencies[folder.name] = int((folder/'scaling_cur_freq').read_text())
        except OSError:
            pass
    return {'cpu': cpus, 'mem_available_mib': memory['MemAvailable']/1024,
            'swap_used_mib': (memory['SwapTotal']-memory['SwapFree'])/1024,
            'temperature_c': temperatures, 'frequency_khz': frequencies,
            'memory_psi': Path('/proc/pressure/memory').read_text().strip()}


def cpu_delta(before, after):
    result = {}
    for name, first in before['cpu'].items():
        last = after['cpu'][name]
        total, idle = last['total']-first['total'], last['idle']-first['idle']
        result[name] = 100*(total-idle)/total if total else 0
    return result


def isolated_environment(domain):
    os.environ.update({'ROS_DOMAIN_ID': str(domain), 'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
                       'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1',
                       'CYCLONEDDS_URI': '<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces>'
                       '<AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex>'
                       '<Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'})
    os.environ.pop('ROS_LOCALHOST_ONLY', None)


def simulation_worker():
    import rclpy
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
    rclpy.init()
    node = rclpy.create_node('load_benchmark_kinematics')
    pub = node.create_publisher(Odometry, '/load_bench/odom', 10)
    dynamic, static = TransformBroadcaster(node), StaticTransformBroadcaster(node)
    mount = TransformStamped()
    # The real camera remains fixed. Do not attach its unchanging images to the
    # moving virtual body: doing so invents moving obstacles and corrupts load.
    mount.header.frame_id, mount.child_frame_id = 'bench_odom', 'camera_link'
    mount.transform.translation.z = .5
    mount.transform.rotation.w = 1.
    static.sendTransform(mount)
    state = {'x': -1.5, 'y': 0., 'yaw': 0., 'vx': 0., 'vy': 0., 'w': 0.,
             'command_time': 0., 'tick': time.monotonic()}

    def command(msg):
        state.update(vx=max(-.75, min(.75, msg.linear.x)), vy=max(-.35, min(.35, msg.linear.y)),
                     w=max(-1., min(1., msg.angular.z)), command_time=time.monotonic())

    subscription = node.create_subscription(Twist, '/load_bench/cmd_vel', command, 20)

    def tick():
        now = time.monotonic()
        dt = max(0., min(.1, now-state['tick']))
        state['tick'] = now
        vx, vy, w = (state[k] for k in ('vx', 'vy', 'w')) if now-state['command_time']<.3 else (0., 0., 0.)
        c, s = math.cos(state['yaw']), math.sin(state['yaw'])
        state['x'] += (c*vx-s*vy)*dt
        state['y'] += (s*vx+c*vy)*dt
        state['yaw'] = math.atan2(math.sin(state['yaw']+w*dt), math.cos(state['yaw']+w*dt))
        msg = Odometry()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'bench_odom', 'bench_body'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = state['x'], state['y'], .5
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(state['yaw']/2), math.cos(state['yaw']/2)
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z = vx, vy, w
        pub.publish(msg)
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = state['x'], state['y'], .5
        transform.transform.rotation = msg.pose.pose.orientation
        dynamic.sendTransform(transform)

    timer = node.create_timer(.01, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', default=str(Path(__file__).resolve().parents[1]))
    parser.add_argument('--camera-executable')
    parser.add_argument('--camera-lib-dir', action='append', default=[])
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--warmup', type=float, default=8)
    parser.add_argument('--baseline-seconds', type=float, default=8)
    parser.add_argument('--cases', default='closed_loop,goal_churn')
    parser.add_argument('--map-outputs', choices=['occupancy','both'], default='occupancy')
    parser.add_argument('--domain', type=int, default=226)
    parser.add_argument('--output')
    parser.add_argument('--worker', choices=['simulation'])
    args = parser.parse_args()
    isolated_environment(args.domain)
    if args.worker:
        simulation_worker()
        return
    if not args.camera_executable or args.seconds<5 or args.warmup<3 or args.fps<=0 or not 0<=args.domain<=230:
        parser.error('camera executable, >=5 measurement seconds, >=3 warmup seconds, and an isolated domain are required')
    cases = args.cases.split(',')
    if not cases or any(case not in ('closed_loop', 'goal_churn', 'mapping_idle') for case in cases):
        parser.error('cases: closed_loop,goal_churn,mapping_idle')
    # Do not touch cameras or DDS domains already used by the user.
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name)==os.getpid(): continue
        try:
            env = (entry/'environ').read_bytes().split(b'\0')
            cmd = (entry/'cmdline').read_bytes().split(b'\0')
            if ('ROS_DOMAIN_ID=%d'%args.domain).encode() in env:
                parser.error('domain is already used by PID '+entry.name)
            if cmd and Path(os.fsdecode(cmd[0])).name=='realsense2_camera_node':
                parser.error('camera already used by PID '+entry.name)
        except (PermissionError, FileNotFoundError, ProcessLookupError):
            pass
    import rclpy
    import yaml
    import numpy as np
    from rclpy.qos import qos_profile_sensor_data as qos
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped, Twist
    from std_msgs.msg import String, Bool
    from scan_planner_msgs.msg import Bspline

    root = Path(args.workspace)
    output = Path(args.output) if args.output else root/'log'/('navigation-load-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    output.mkdir(parents=True, exist_ok=False)
    report = {'started': datetime.datetime.now().astimezone().isoformat(), 'output': str(output),
              'configuration': vars(args), 'physical_robot_motion': False,
              'workload': 'fixed real D435i with static world TF + software stereo + map/planner + controller + separate moving virtual body',
              'limitations': ['The camera is external and fixed in the test world; this is not an onboard-camera moving robot test.',
                              'No real VIO/SLAM/odometry algorithm, chassis bridge, RViz or real actuator execution.',
                              'Scene-dependent short benchmark; not a navigation or thermal-soak acceptance test.'],
              'affinity': {'application': [4,5,6,7], 'observer_and_simulator': [0,1,2,3]},
              'cases': [], 'owned_processes': []}
    protected = [root/'src/planner/plan_manage/src/planner_manager.cpp',
                 root/'src/planner/plan_manage/config/planner.yaml', root/'src/planner/plan_manage/config/passive_stereo.yaml']
    report['source_sha256'] = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in protected}
    processes, logs, node = [], [], None
    low_cpus = {0,1,2,3} & os.sched_getaffinity(0)
    os.sched_setaffinity(0, low_cpus)
    # Children may expand this process's affinity to the big cores.

    def launch(command, name, low=False, env=None):
        log = (output/(name+'.log')).open('w'); logs.append(log)
        def pin(): os.sched_setaffinity(0, {0,1,2,3} if low else {4,5,6,7})
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=env,
                                   start_new_session=True, preexec_fn=pin)
        processes.append(process)
        report['owned_processes'].append({'name': name, 'pid': process.pid, 'pgid': process.pid, 'command': command})
        return process

    def stop(process):
        # Also terminate descendants if the process-group leader has exited.
        for sig, seconds in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
            try: os.killpg(process.pid, sig)
            except ProcessLookupError: break
            try: process.wait(timeout=seconds)
            except subprocess.TimeoutExpired: continue
            try: os.killpg(process.pid, 0)
            except ProcessLookupError: break
        if process.poll() is None: process.wait(timeout=2)

    def spin(seconds):
        until = time.monotonic()+seconds
        while time.monotonic()<until:
            rclpy.spin_once(node, timeout_sec=max(0., min(.02, until-time.monotonic())))

    def command(binary, parameters=(), remaps=None):
        result = [str(binary), '--ros-args']
        for entry in parameters:
            if isinstance(entry, Path): result += ['--params-file', str(entry)]
            else:
                for key, value in entry.items():
                    result += ['-p', key+':='+(str(value).lower() if isinstance(value, bool) else str(value))]
        for key, value in (remaps or {}).items(): result += ['-r', key+':='+value]
        return result

    def interrupt(signum, frame): raise KeyboardInterrupt
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP): signal.signal(sig, interrupt)
    state = {'left_info': None, 'right_info': None, 'odom': None, 'inflated_map': None,
             'active': False, 'record': None}

    def choose_goal(pose, yaw):
        # Pick a test goal using the published inflated map, without changing
        # map contents, footprint, motion limits, or production planning code.
        message = state['inflated_map']
        occupied = set()
        if message is not None and message.width*message.height:
            offsets = {field.name:field.offset for field in message.fields}
            dtype = np.dtype({'names':['x','y','z'], 'formats':['<f4']*3,
                              'offsets':[offsets[k] for k in ('x','y','z')], 'itemsize':message.point_step})
            points = np.ndarray((message.width*message.height,), dtype=dtype, buffer=message.data)
            if args.map_outputs=='both':
                near = points[np.abs(points['z']-pose.position.z)<.075]
                occupied = set(zip(np.floor(near['x']/.05).astype(int), np.floor(near['y']/.05).astype(int)))
            else:
                near = points[(points['z']>=pose.position.z-.15)&(points['z']<=pose.position.z+.45)]
                cells=set(zip(np.floor(near['x']/.05).astype(int),np.floor(near['y']/.05).astype(int)))
                offsets=[(x,y) for x in range(-5,6) for y in range(-5,6) if x*x+y*y<=25]
                occupied={(int(x)+dx,int(y)+dy) for x,y in cells for dx,dy in offsets}
        candidates=[]
        for angle in (0,45,-45,90,-90,135,-135,180):
            heading=yaw+math.radians(angle)
            c,s=math.cos(heading),math.sin(heading)
            if abs(pose.position.x+1.5*c)>3.5 or abs(pose.position.y+1.5*s)>3.5:continue
            collisions=0
            for distance in np.linspace(0.,1.5,31):
                for offset in (-.18,.18):
                    x=pose.position.x+(distance+offset)*c
                    y=pose.position.y+(distance+offset)*s
                    collisions += (math.floor(x/.05),math.floor(y/.05)) in occupied
            candidates.append((collisions,angle,heading))
            if not collisions:break
        if not candidates:raise RuntimeError('virtual robot left the bounded test area')
        selected=min(candidates,key=lambda item:item[0])
        return (pose.position.x+1.5*math.cos(selected[2]), pose.position.y+1.5*math.sin(selected[2]),
                {'relative_bearing_deg':selected[1],'occupied_footprint_samples':selected[0],
                 'slice_cells':len(occupied),'map_points':message.width*message.height if message else 0})

    def event(key, message):
        if key=='odom': state['odom']=message
        elif key=='inflated_map':state['inflated_map']=message
        elif key=='map' and args.map_outputs=='occupancy':state['inflated_map']=message
        if not state['active']: return
        record = state['record']
        record['counts'][key] = record['counts'].get(key, 0)+1
        if key=='depth':
            record['latency_ms'].append((time.time()-message.header.stamp.sec-message.header.stamp.nanosec/1e9)*1000)
        elif key=='ready': record['ready'].append(message.data)
        elif key=='statistics': record['statistics'].append(json.loads(message.data))
        elif key=='cmd' and abs(message.linear.x)+abs(message.linear.y)+abs(message.angular.z)>1e-3:
            record['nonzero_commands']+=1
        elif key=='odom':
            x,y=message.pose.pose.position.x,message.pose.pose.position.y
            if record['positions']:
                last=record['positions'][-1]
                record['travel_m']+=math.hypot(x-last[0],y-last[1])
            record['positions'].append([x,y])

    try:
        print('Output:', output, flush=True)
        report['baseline_before'] = system_sample()
        time.sleep(args.baseline_seconds)
        report['baseline_after'] = system_sample()
        report['baseline_cpu_busy_percent'] = cpu_delta(report['baseline_before'], report['baseline_after'])
        rclpy.init(); node=rclpy.create_node('navigation_load_observer')
        subscriptions = [node.create_subscription(kind, topic, lambda msg,key=key:event(key,msg), quality)
                         for key,kind,topic,quality in [
                             ('left',Image,'/camera/camera/infra1/image_rect_raw',qos),
                             ('right',Image,'/camera/camera/infra2/image_rect_raw',qos),
                             ('depth',Image,'/load_bench/depth',qos),
                             ('map',PointCloud2,'/grid_map/occupancy',qos),
                             ('ready',Bool,'/grid_map/depth_ready',1),
                             ('statistics',String,'/load_bench/statistics',1),
                             ('odom',Odometry,'/load_bench/odom',qos),
                             ('trajectory',Bspline,'/planning/bspline',10),
                             ('cmd',Twist,'/load_bench/cmd_vel',20)]]
        if args.map_outputs=='both':
            subscriptions.append(node.create_subscription(PointCloud2,'/grid_map/occupancy_inflate',
                                 lambda msg:event('inflated_map',msg),qos))
        for side in ('left','right'):
            topic='/camera/camera/infra%d/camera_info'%(1 if side=='left' else 2)
            subscriptions.append(node.create_subscription(CameraInfo,topic,
                                 lambda msg,key=side+'_info':state.update({key:msg}),qos))
        goal_pub = node.create_publisher(PoseStamped, '/move_base_simple/goal', 1)
        camera_env=os.environ.copy()
        camera_env['LD_LIBRARY_PATH']=camera_env.get('LD_LIBRARY_PATH','')+':'+':'.join(args.camera_lib_dir)
        camera=launch(command(args.camera_executable,[{'camera_name':'camera', 'enable_infra1':True,
            'enable_infra2':True, 'depth_module.infra_profile':'640,480,%d'%args.fps, 'enable_depth':False,
            'enable_color':False,'enable_gyro':False,'enable_accel':False,'enable_sync':True,
            'depth_module.emitter_enabled':0,'publish_tf':True,'infra1_qos':'SENSOR_DATA','infra2_qos':'SENSOR_DATA'}],
            {'__ns':'/camera','__node':'camera'}), 'camera', env=camera_env)
        deadline=time.monotonic()+25
        while state['left_info'] is None or state['right_info'] is None:
            spin(.1)
            if camera.poll() is not None or time.monotonic()>deadline: raise RuntimeError('camera startup failed; see camera.log')
        report['calibration']={key:{'frame':state[key].header.frame_id, 'p':list(state[key].p),
                                   'width':state[key].width,'height':state[key].height} for key in ('left_info','right_info')}
        for case in cases:
            print('Starting case:', case, flush=True)
            state['odom']=None
            state['inflated_map']=None
            sim=launch([sys.executable, str(Path(__file__).resolve()), '--worker','simulation','--domain',str(args.domain)],case+'_simulation',low=True)
            stereo=launch(command(root/'install/scan_stereo_depth/lib/scan_stereo_depth/stereo_depth_node',
                [root/'src/sensors/scan_stereo_depth/config/stereo.yaml'],
                {'left/image_rect':'/camera/camera/infra1/image_rect_raw','right/image_rect':'/camera/camera/infra2/image_rect_raw',
                 'left/camera_info':'/camera/camera/infra1/camera_info','right/camera_info':'/camera/camera/infra2/camera_info',
                 'depth':'/load_bench/depth','camera_info':'/load_bench/camera_info','statistics':'/load_bench/statistics'}),case+'_stereo')
            config=yaml.safe_load((root/'src/planner/plan_manage/config/planner.yaml').read_text())
            config['scan_planner_node']['ros__parameters'].update(yaml.safe_load((root/'src/planner/plan_manage/config/passive_stereo.yaml').read_text())['scan_planner_node']['ros__parameters'])
            config['scan_planner_node']['ros__parameters']['grid_map.frame_id']='bench_odom'
            configfile=output/(case+'_planner.yaml'); configfile.write_text(yaml.safe_dump(config))
            planner=launch(command(root/'install/scan_planner/lib/scan_planner/scan_planner_node',[configfile],
                {'body_pose':'/load_bench/odom','depth':'/load_bench/depth','depth_camera_info':'/load_bench/camera_info'}),case+'_planner')
            controller=launch(command(root/'install/scan_planner/lib/scan_planner/closed_loop_controller',
                [root/'src/planner/plan_manage/config/controllers.yaml',{'planning_frame':'bench_odom','odom_timeout':.5,'require_depth':True}],
                {'body_pose':'/load_bench/odom','cmd_vel':'/load_bench/cmd_vel'}),case+'_controller')
            roles={'camera':camera,'stereo':stereo,'planner_and_map':planner,'controller':controller,'simulator':sim}
            record={'counts':{},'latency_ms':[],'ready':[],'statistics':[],'nonzero_commands':0,
                    'positions':[],'travel_m':0.,'goals':[], 'windows':[]}
            interval = 1. if case=='goal_churn' else 8.
            last_goal,begin,measuring = 0.,time.monotonic(),False
            before_process,before_system,before_time,last_time,last_process,last_system = {},None,None,None,{},None
            while True:
                spin(.02)
                now=time.monotonic()
                for role,process in roles.items():
                    if process.poll() is not None: raise RuntimeError(role+' exited during '+case)
                if case!='mapping_idle' and state['odom'] is not None and now-begin>3 and now-last_goal>=interval:
                    pose=state['odom'].pose.pose
                    q=pose.orientation
                    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    goal=PoseStamped(); goal.header.frame_id='bench_odom';goal.header.stamp=node.get_clock().now().to_msg()
                    goal.pose.position.x,goal.pose.position.y,choice=choose_goal(pose,yaw)
                    goal.pose.position.z=.5;goal.pose.orientation.w=1.
                    goal_pub.publish(goal);last_goal=now
                    if not measuring:print('Warmup goal:',json.dumps(choice),flush=True)
                    if measuring: record['goals'].append({'elapsed_s':now-before_time,'x':goal.pose.position.x,'y':goal.pose.position.y,**choice})
                if not measuring and now-begin>=args.warmup:
                    for key in ('counts','latency_ms','ready','statistics','positions','goals','windows'):
                        record[key]={} if key=='counts' else []
                    record['nonzero_commands']=0;record['travel_m']=0.
                    before_process={role:proc_sample(process.pid) for role,process in roles.items()}
                    before_process['observer']=proc_sample(os.getpid())
                    before_system=system_sample();before_time=time.monotonic()
                    last_process=before_process;last_system=before_system;last_time=before_time
                    state.update(active=True,record=record);measuring=True
                    print('Measuring:',case,flush=True)
                if measuring and now-last_time>=1.:
                    current={role:proc_sample(process.pid) for role,process in roles.items()}
                    current['observer']=proc_sample(os.getpid())
                    system=system_sample();current_time=time.monotonic()
                    window={'elapsed_s':current_time-before_time,'seconds':current_time-last_time,
                            'process_cpu_one_core_100':{role:100*(value['cpu_s']-last_process[role]['cpu_s'])/(current_time-last_time) for role,value in current.items()},
                            'rss_mib':{role:value['rss_mib'] for role,value in current.items()},
                            'system_busy_percent':cpu_delta(last_system,system),'system':system}
                    record['windows'].append(window)
                    last_time,last_process,last_system=current_time,current,system
                    if len(record['windows'])%15==0:
                        print(json.dumps({'case':case,'elapsed_s':round(window['elapsed_s'],1),'cpu':window['process_cpu_one_core_100'],
                                          'counts':record['counts'],'depth_ready_fraction':sum(record['ready'])/max(1,len(record['ready']))}),flush=True)
                if measuring and now-before_time>=args.seconds:break
            state['active']=False
            elapsed=time.monotonic()-before_time
            after_system=system_sample()
            after_process={role:proc_sample(process.pid) for role,process in roles.items()}
            after_process['observer']=proc_sample(os.getpid())
            process_metrics={}
            for role,after in after_process.items():
                windows=[w['process_cpu_one_core_100'][role] for w in record['windows']]
                process_metrics[role]={'cpu_average_one_core_100':100*(after['cpu_s']-before_process[role]['cpu_s'])/elapsed,
                                       'cpu_1s_p95_one_core_100':percentile(windows,.95),'cpu_1s_max_one_core_100':max(windows,default=None),
                                       'peak_rss_mib':after['hwm_mib'],'threads':after['threads']}
            production_roles=('camera','stereo','planner_and_map','controller')
            total_windows=[sum(w['process_cpu_one_core_100'][role] for role in production_roles) for w in record['windows']]
            planner_log=(output/(case+'_planner.log')).read_text(errors='replace')
            result={'name':case,'seconds':elapsed,'goal_interval_s':interval if case!='mapping_idle' else None,
                    'processes':process_metrics,
                    'application_cpu_average_one_core_100':sum(process_metrics[role]['cpu_average_one_core_100'] for role in production_roles),
                    'application_cpu_1s_p95_one_core_100':percentile(total_windows,.95),
                    'application_cpu_1s_max_one_core_100':max(total_windows,default=None),
                    'application_peak_rss_sum_mib':sum(process_metrics[role]['peak_rss_mib'] for role in production_roles),
                    'system_average_busy_percent':cpu_delta(before_system,after_system),
                    'rates_hz':{key:value/elapsed for key,value in record['counts'].items()},
                    'counts':record['counts'],'goals_sent':len(record['goals']),'goals':record['goals'],
                    'nonzero_command_fraction':record['nonzero_commands']/max(1,record['counts'].get('cmd',0)),
                    'simulated_travel_m':record['travel_m'],'simulated_position_start':record['positions'][0] if record['positions'] else None,
                    'simulated_position_end':record['positions'][-1] if record['positions'] else None,
                    'depth_ready_fraction':sum(record['ready'])/max(1,len(record['ready'])),
                    'depth_receive_latency_ms_mean':sum(record['latency_ms'])/max(1,len(record['latency_ms'])),
                    'depth_receive_latency_ms_p95':percentile(record['latency_ms'],.95),
                    'stereo_process_ms_mean':sum(x['process_ms_mean'] for x in record['statistics'])/max(1,len(record['statistics'])),
                    'stereo_valid_fraction':sum(x['valid_fraction'] for x in record['statistics'])/max(1,len(record['statistics'])),
                    'planner_log_counts_including_warmup':{key:planner_log.count(key) for key in ('final_plan_success=1','final_plan_success=0','Dynamic feasibility failed','EMERGENCY_STOP','Close to goal')},
                    'full_chain_active':planner_log.count('final_plan_success=1')>0 and record['travel_m']>.05 and record['nonzero_commands']>0,
                    'min_mem_available_mib':min([before_system['mem_available_mib']]+[w['system']['mem_available_mib'] for w in record['windows']]),
                    'max_swap_used_mib':max([before_system['swap_used_mib']]+[w['system']['swap_used_mib'] for w in record['windows']]),
                    'max_temperature_c':max([v for w in record['windows'] for v in w['system']['temperature_c'].values()],default=None),
                    'windows':record['windows'],'stereo_statistics':record['statistics']}
            report['cases'].append(result)
            (output/'result.partial.json').write_text(json.dumps(report,indent=2))
            print('Case complete:',json.dumps({k:result[k] for k in ('name','application_cpu_average_one_core_100','rates_hz','simulated_travel_m','depth_ready_fraction','planner_log_counts_including_warmup')}),flush=True)
            for process in (controller,planner,stereo,sim):stop(process)
            spin(.5)
        report['completed']=True
    except BaseException as error:
        report['completed']=False;report['error']=repr(error)
        raise
    finally:
        state['active']=False
        for process in reversed(processes):
            try: stop(process)
            except (ProcessLookupError,subprocess.TimeoutExpired): pass
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        for log in logs:log.close()
        remaining=[]
        owned_groups={process.pid for process in processes}
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():continue
            try:
                if os.getpgid(int(entry.name)) in owned_groups:remaining.append(int(entry.name))
            except (ProcessLookupError,PermissionError):pass
        report['remaining_owned_processes']=remaining
        report['all_owned_processes_stopped']=not remaining and all(p.poll() is not None for p in processes)
        report['protected_source_unchanged']=all(hashlib.sha256(p.read_bytes()).hexdigest()==report['source_sha256'][str(p.relative_to(root))] for p in protected)
        report['finished']=datetime.datetime.now().astimezone().isoformat()
        (output/'result.json').write_text(json.dumps(report,indent=2))
        print('Report:',output/'result.json',flush=True)


if __name__=='__main__':
    main()
