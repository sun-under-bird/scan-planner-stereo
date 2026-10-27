#!/usr/bin/env python3
"""Isolated synthetic 15Hz depth/filter benchmark; never opens a camera or drives a robot."""
import os
os.environ['ROS_DOMAIN_ID']='230'
os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
os.environ.pop('ROS_LOCALHOST_ONLY',None)
os.environ['CYCLONEDDS_URI']='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
import argparse,json,time,subprocess,signal
from pathlib import Path
import numpy as np
import yaml,rclpy
from rclpy.qos import qos_profile_sensor_data as qos
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool,String
from tf2_ros import StaticTransformBroadcaster

def main():
 ap=argparse.ArgumentParser(description=__doc__)
 ap.add_argument('--workspace',default=str(Path(__file__).resolve().parents[4]))
 ap.add_argument('--binary-workspace')
 ap.add_argument('--seconds',type=float,default=10.)
 args=ap.parse_args();root=Path(args.workspace);binary=Path(args.binary_workspace or args.workspace)
 out=root/'log'/time.strftime('filter-benchmark-%Y%m%d-%H%M%S');out.mkdir()
 rclpy.init();node=rclpy.create_node('depth_filter_benchmark',namespace='/filter_benchmark')
 ns='/filter_benchmark';data={'filtered':[],'ready':[],'stats':[]}
 def filtered(m):
  stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
  data['filtered'].append((time.monotonic(),stamp,(node.get_clock().now().nanoseconds-stamp)/1e6,m.width))
 subs=[node.create_subscription(PointCloud2,ns+'/grid_map/filtered_obstacles',filtered,qos),
       node.create_subscription(Bool,ns+'/grid_map/depth_ready',lambda m:data['ready'].append((time.monotonic(),m.data)),1),
       node.create_subscription(String,ns+'/grid_map/depth_filter_statistics',lambda m:data['stats'].append((time.monotonic(),json.loads(m.data))),qos)]
 pubs={k:node.create_publisher(t,ns+'/'+k,qos) for k,t in [('depth',Image),('depth_camera_info',CameraInfo),('body_pose',Odometry)]}
 params=yaml.safe_load((root/'src/planner/plan_manage/config/planner.yaml').read_text())['scan_planner_node']['ros__parameters']
 params.update(yaml.safe_load((root/'src/planner/plan_manage/config/passive_stereo.yaml').read_text())['scan_planner_node']['ros__parameters'])
 cfg=out/'params.yaml';cfg.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
 results=[];report={'domain':230,'physical_camera':False,'robot_commands':False,'cases':results}
 logfile=(out/'planner.log').open('w')
 proc=subprocess.Popen([str(binary/'install/scan_planner/lib/scan_planner/scan_planner_node'),'--ros-args','-r','__ns:='+ns,'--params-file',str(cfg)],stdout=logfile,stderr=subprocess.STDOUT,start_new_session=True)
 try:
  static=StaticTransformBroadcaster(node)
  a=TransformStamped();a.header.frame_id='odom';a.child_frame_id='filter_optical'
  a.transform.translation.z=.5
  a.transform.rotation.x=-.5;a.transform.rotation.y=.5;a.transform.rotation.z=-.5;a.transform.rotation.w=.5
  b=TransformStamped();b.header.frame_id='odom';b.child_frame_id='base_footprint';b.transform.rotation.w=1.
  static.sendTransform([a,b])
  c=CameraInfo();c.header.frame_id='filter_optical';c.width=640;c.height=480
  c.p=[400.,0.,320.,0.,0.,400.,240.,0.,0.,0.,1.,0.]
  rng=np.random.default_rng(41)
  cases={'wall':np.full((480,640),2.,dtype=np.float32),
         'passive_sparse':np.where(rng.random((480,640))<.4,
                                  rng.uniform(.4,3.,(480,640)),0.).astype(np.float32)}
  def spin(until):
   while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=max(0.,min(.002,until-time.monotonic())))
  def interrupted(s,f):raise KeyboardInterrupt
  for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupted)
  spin(time.monotonic()+1.0)
  for name,array in cases.items():
   for x in data.values():x.clear()
   image=Image();image.header.frame_id='filter_optical';image.width=640;image.height=480
   image.encoding='32FC1';image.step=2560;image.data=array.tobytes()
   start=time.monotonic();end=start+args.seconds;published=0;next_frame=start
   while time.monotonic()<end:
    stamp=node.get_clock().now().to_msg();image.header.stamp=stamp;c.header.stamp=stamp
    odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom';odom.child_frame_id='base_link'
    odom.pose.pose.position.z=.36;odom.pose.pose.orientation.w=1.
    pubs['depth_camera_info'].publish(c);pubs['body_pose'].publish(odom);pubs['depth'].publish(image)
    published+=1;next_frame+=1/15.;spin(min(next_frame,end))
    if proc.poll() is not None:raise RuntimeError('planner exited')
   spin(time.monotonic()+.25)
   frames=[x for x in data['filtered'] if start+1<=x[0]<end]
   duration=(frames[-1][0]-frames[0][0]) if len(frames)>1 else 0
   lat=[x[2] for x in frames];stats=[x[1] for x in data['stats']]
   result={'name':name,'published':published,'observed':len(data['filtered']),
           'output_hz':(len(frames)-1)/duration if duration else 0,
           'latency_p95_ms':float(np.percentile(lat,95)) if lat else None,
           'filter_ms_max_sampled':max((x['filter_ms'] for x in stats),default=None),
           'ready_true_fraction':sum(x[1] for x in data['ready'])/max(1,len(data['ready'])),
           'latest_statistics':stats[-1:] }
   result['passed']=result['output_hz']>=13.5 and result['ready_true_fraction']>=.9
   results.append(result);print(json.dumps(result),flush=True)
  report['passed']=all(x['passed'] for x in results)
 finally:
  if proc.poll() is None:
   os.killpg(proc.pid,signal.SIGINT)
   try:proc.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
  report['all_owned_processes_stopped']=proc.poll() is not None
  node.destroy_node();rclpy.shutdown();logfile.close()
  (out/'result.json').write_text(json.dumps(report,indent=2))
  print('REPORT',out/'result.json',flush=True)
if __name__=='__main__':main()
