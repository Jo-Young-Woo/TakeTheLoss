#!/usr/bin/env python3
"""Teleport B2 across six base beams in a fresh, ready, single-UGV arena.

This diagnostic changes Gazebo poses only; it never sends hardware commands or
changes scoring time. The original pose is restored and the match stays ready.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    spec = yaml.safe_load(args.spec.read_text(encoding='utf-8'))
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = Node('arena_base_beam_probe')
    state, poses, events, rows = {}, {}, [], []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/arena/state', lambda m: state.update(json.loads(m.data)), qos)
    node.create_subscription(String, '/arena/events', lambda m: events.append(json.loads(m.data)), 50)

    def receive(msg):
        for transform in msg.transforms:
            if transform.child_frame_id.startswith('ugv_') and '::' not in transform.child_frame_id:
                p, q = transform.transform.translation, transform.transform.rotation
                poses[transform.child_frame_id] = (p.x, p.y, p.z,
                    math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
    node.create_subscription(TFMessage, '/arena/dynamic_pose', receive, 10)

    def wait_for(predicate, seconds=8.):
        deadline = time.monotonic()+seconds
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError('Expected live pose or beam event was not received')
            rclpy.spin_once(node, timeout_sec=.02)
        if state.get('phase') != 'ready':
            raise RuntimeError('Beam diagnostic requires the ready phase')

    def teleport(pose):
        x, y, z, yaw = pose
        request = ('name: "ugv_B2", position: {x: %.9f, y: %.9f, z: %.9f}, '
                   'orientation: {w: %.9f, z: %.9f}' %
                   (x, y, z, math.cos(yaw/2), math.sin(yaw/2)))
        result = subprocess.run(['ign', 'service', '-s', '/world/maicon_arena/set_pose',
            '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '3000', '--req', request], capture_output=True, text=True, timeout=5)
        if result.returncode or 'data: true' not in result.stdout:
            raise RuntimeError('Gazebo did not acknowledge pose update')
        wait_for(lambda: 'ugv_B2' in poses and math.dist(poses['ugv_B2'][:2], (x, y)) < .02)

    original, error = None, None
    try:
        wait_for(lambda: state and 'ugv_B2' in poses)
        if set(poses) != {'ugv_B2'}:
            raise RuntimeError('Use ugvs:=1 so other vehicles cannot mask beam transitions')
        original = poses['ugv_B2']
        neutral = (-2.20, 0., original[2], 0.)
        teleport(neutral)
        for base, (x, first, last) in spec['bases']['photoelectric_beam']['segments'].items():
            index = len(events)
            target = (x, (first+last)/2, original[2], 0.)
            teleport(target)
            wait_for(lambda: any(e.get('type') == 'base_beam' and e.get('base') == base
                                 and e.get('blocked') is True for e in events[index:]))
            actual = poses['ugv_B2']
            teleport(neutral)
            wait_for(lambda: any(e.get('type') == 'base_beam' and e.get('base') == base
                                 and e.get('blocked') is False for e in events[index:]))
            rows.append({'base': base, 'requested_pose': target, 'observed_pose': actual,
                         'events': [e for e in events[index:] if e.get('base') == base]})
    except Exception as exc:
        error = str(exc)
    finally:
        if original is not None:
            try:
                teleport(original)
            except Exception as exc:
                error = error or str(exc)
        result = {'passed': error is None and len(rows) == 6, 'error': error,
                  'scope': 'actual Gazebo set_pose -> dynamic_pose -> anonymous referee beam events',
                  'phase': state.get('phase'), 'checks': rows,
                  'capture_events': [e for e in events if e.get('type') == 'base_captured']}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
        node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps({'passed': result['passed'], 'bases': len(rows), 'error': error}))
    return 0 if result['passed'] and not result['capture_events'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
