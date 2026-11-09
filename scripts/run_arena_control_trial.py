#!/usr/bin/env python3
"""Bounded B2 control integration trial with actual camera recordings.

This is an odometry waypoint / MIL interface test, not an M3 or Nav2 score run.
Use a fresh default arena with B2 as ego and validation_camera enabled.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from maicon26.hal.fpv_recording import SimulationVideoWriter, decode_bgr


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, LaserScan, JointState
    from std_msgs.msg import String
    from mil_interfaces_sim.srv import Fire, Gimbal

    rclpy.init()
    node = Node('arena_control_trial', parameter_overrides=[Parameter('use_sim_time', value=True)])
    state, odom, scans, joints, events = {}, {}, {}, {}, []
    args.output_dir.mkdir(parents=True, exist_ok=True)
    writers = {
        '/image_raw': SimulationVideoWriter(args.output_dir/'b2_control_fpv.mp4', 10, 1., 'B2 control integration / actual camera'),
        '/arena/validation/image_raw': SimulationVideoWriter(args.output_dir/'b2_control_third_person.mp4', 5, 1., 'B2 control integration / actual Gazebo')}
    def now():
        return node.get_clock().now().nanoseconds*1e-9
    def on_odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        odom.update(x=p.x, y=p.y, yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)), at=now())
    def on_scan(msg):
        readings = [r for i, r in enumerate(msg.ranges)
                    if abs(msg.angle_min+i*msg.angle_increment) < .4 and math.isfinite(r)]
        scans.update(front=min(readings) if readings else 0., at=now())
    def on_joint(msg):
        joints.update({n.split('::')[-1]: p for n, p in zip(msg.name, msg.position)})
    def on_image(topic, msg):
        writers[topic].add(decode_bgr(msg), msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/arena/state', lambda m: state.update(json.loads(m.data)), qos)
    node.create_subscription(String, '/arena/events', lambda m: events.append(json.loads(m.data)), 50)
    node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    node.create_subscription(JointState, '/arena/ego/joint_states', on_joint, qos_profile_sensor_data)
    for topic in writers:
        node.create_subscription(Image, topic, lambda m, t=topic: on_image(t, m), qos_profile_sensor_data)
    cmd = node.create_publisher(Twist, '/cmd_vel', 10)
    control = node.create_publisher(String, '/arena/control', 10)
    fire, gimbal = node.create_client(Fire, '/mil/fire'), node.create_client(Gimbal, '/mil/gimbal')
    opened, trace, checks = time.monotonic(), [], {}
    def spin():
        rclpy.spin_once(node, timeout_sec=.02)
        if time.monotonic()-opened > 100:
            raise TimeoutError('Bounded control trial exceeded wall time')
    def wait_for(predicate, timeout=10.):
        deadline = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError('Required live data or service response not received')
            spin()
    def wait_sim(seconds):
        end = now()+seconds
        while now() < end:
            spin()
    def command(v=0., w=0.):
        message = Twist()
        message.linear.x, message.angular.z = float(v), float(w)
        cmd.publish(message)
    def service(client, request):
        wait_for(client.service_is_ready)
        future = client.call_async(request)
        wait_for(future.done)
        return future.result()
    error = None
    try:
        wait_for(lambda: state.get('phase') == 'ready' and odom and scans and
                 all(writer.source_frames for writer in writers.values()), 25.)
        # Odometry must be at the documented start, not a reused previous trial.
        if math.hypot(odom['x'], odom['y']) > .05:
            raise ValueError('Fresh relative odometry is required')
        checks['ready_fire_locked'] = service(fire, Fire.Request()).message == 'locked'
        control.publish(String(data=json.dumps({'cmd': 'start'})))
        wait_for(lambda: state.get('phase') == 'running')
        request = Gimbal.Request(pan=45., tilt=40., speed=90., acc=90.)
        checks['gimbal_accepted'] = service(gimbal, request).message
        wait_sim(1.6)
        checks['measured_pan_rad'] = joints.get('pan_joint')
        checks['measured_tilt_rad'] = joints.get('tilt_joint')
        checks['gimbal_reached'] = (abs(joints.get('pan_joint', 99)+math.radians(22.5)) < .035 and
                                    abs(joints.get('tilt_joint', 99)-math.radians(6.)) < .035)
        checks['first_fire'] = service(fire, Fire.Request()).message
        checks['second_fire'] = service(fire, Fire.Request()).message
        service(gimbal, Gimbal.Request(pan=0., tilt=0., speed=90., acc=90.))
        start = now()
        goals = ((.40, 0.), (.40, .55), (.65, .55))
        for gx, gy in goals:
            while math.hypot(gx-odom['x'], gy-odom['y']) > .055:
                if now()-start > 45:
                    raise TimeoutError('Waypoint control did not converge')
                if now()-odom['at'] > .3 or now()-scans['at'] > .3:
                    raise RuntimeError('Stale odometry or scan; stopping')
                dx, dy = gx-odom['x'], gy-odom['y']
                angle = math.atan2(math.sin(math.atan2(dy, dx)-odom['yaw']),
                                   math.cos(math.atan2(dy, dx)-odom['yaw']))
                v = min(.18, .65*math.hypot(dx, dy)) if abs(angle) < .25 else 0.
                if v and scans['front'] < .24:
                    raise RuntimeError('Measured obstacle within forward stopping margin')
                command(v, max(-.75, min(.75, 1.6*angle)))
                if not trace or now()-trace[-1]['t'] >= .2:
                    trace.append(dict(t=now(), x=odom['x'], y=odom['y'], yaw=odom['yaw'], front=scans['front']))
                spin()
            command()
            wait_sim(.3)
        checks['waypoints_reached'] = len(goals)
        # One final command, then intentional command silence. The emulator must stop.
        command(.08)
        wait_sim(.9)
        stopped = (odom['x'], odom['y'])
        wait_sim(.7)
        checks['watchdog_drift_m'] = math.dist(stopped, (odom['x'], odom['y']))
        checks['watchdog_stopped'] = checks['watchdog_drift_m'] < .02
        checks['ammo_after_one_fire'] = state['ugvs']['B-2']['ammo']
        checks['passed'] = (checks['ready_fire_locked'] and checks['gimbal_reached'] and
                            checks['first_fire'] == 'fired' and checks['second_fire'] == 'reloading' and
                            checks['watchdog_stopped'] and checks['ammo_after_one_fire'] == 7)
    except Exception as exc:
        error = type(exc).__name__+': '+str(exc)
        checks['passed'] = False
    finally:
        command()
        control.publish(String(data=json.dumps({'cmd': 'pause'})))
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=.05)
        summaries = {topic: writer.close() for topic, writer in writers.items()}
        result = dict(scope='odometry/MIL control integration; not autonomous M3/Nav2 scoring',
                      checks=checks, error=error, trace=trace, events=events, recordings=summaries)
        (args.output_dir/'control_trial.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps({'checks': checks, 'error': error}), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if checks.get('passed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
