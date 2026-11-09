#!/usr/bin/env python3
"""Loopback-only arena API: actual UAV pixels + referee state, never world truth."""
import argparse
import datetime as dt
import json
from pathlib import Path
import signal
import sys
import threading

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from maicon26.credentials import read_mission_code
from maicon26.link.secrets import redacted_output
from maicon26.sim.gazebo_api import GazeboApiAdapter, jpeg_from_ros_image, make_http_server


class SafeParser(argparse.ArgumentParser):
    def error(self, message):
        raise ValueError("Invalid local mock API arguments; use --help. Credentials are not accepted in arguments.")


def parse_args(argv=None):
    parser = SafeParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--bind", default="127.0.0.1", help="Literal loopback IP only")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--code-file", type=Path, help="Credential file; MAICON_CODE/legacy environment has priority")
    parser.add_argument("--slot", choices=("B-1", "B-2", "B-3", "R-1", "R-2", "R-3"), default="B-2")
    parser.add_argument("--operator-config", type=Path, help="Explicit orders/targets/test_mode JSON; absent means no invented missions")
    parser.add_argument("--epoch", help="Optional ISO8601 wall epoch for simulation t=0, including timezone")
    parser.add_argument("--image-topic", default="/uav/image_raw")
    parser.add_argument("--state-topic", default="/arena/state")
    parser.add_argument("--clock-topic", default="/clock")
    parser.add_argument("--jpeg-quality", type=int, default=85)
    parser.add_argument("--max-frame-age", type=float, default=2.)
    args = parser.parse_args(argv)
    if not 0 <= args.port <= 65535 or not 1 <= args.jpeg_quality <= 95:
        parser.error("invalid numeric range")
    return args


@redacted_output
def main(argv=None):
    args = parse_args(argv)
    credential = read_mission_code(args.code_file)
    if args.operator_config is None:
        operator = {}
    else:
        try:
            operator = json.loads(args.operator_config.read_text(encoding="utf-8-sig"))
        except (OSError, ValueError, UnicodeError):
            raise ValueError("Cannot read operator configuration JSON") from None
    try:
        epoch = dt.datetime.fromisoformat(args.epoch.replace("Z", "+00:00")) if args.epoch else None
    except ValueError:
        raise ValueError("Epoch must be an ISO8601 timestamp with a timezone") from None
    adapter = GazeboApiAdapter({credential: args.slot}, operator_config=operator, epoch=epoch,
                               max_input_age_s=args.max_frame_age)
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    class FrameNode(Node):
        def __init__(self):
            super().__init__("maicon_arena_mock_api", parameter_overrides=[Parameter("use_sim_time", value=True)])
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Clock, args.clock_topic, self.clock, qos_profile_sensor_data)
            self.create_subscription(Image, args.image_topic, self.image, qos_profile_sensor_data)
            self.create_subscription(String, args.state_topic, self.state, qos)
            self.controls = self.create_publisher(String, "/arena/control", 10)
            self.create_timer(.05, self.flush_controls)

        def clock(self, message):
            adapter.update_time(message.clock.sec+message.clock.nanosec*1e-9)

        def state(self, message):
            try:
                if not adapter.update_state(json.loads(message.data)):
                    adapter.stats["state_rejected"] += 1
            except (ValueError, TypeError):
                adapter.stats["state_rejected"] += 1

        def image(self, message):
            try:
                jpeg = jpeg_from_ros_image(message, quality=args.jpeg_quality)
                stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
                adapter.capture_jpeg(jpeg, stamp)
            except (ValueError, TypeError, OSError):
                adapter.stats["image_decode_rejected"] += 1

        def flush_controls(self):
            for control in adapter.drain_controls():
                self.controls.publish(String(data=json.dumps(control)))

    rclpy.init(args=[])
    node = FrameNode()
    server = None
    worker = None
    previous = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGTERM, lambda *_: rclpy.try_shutdown())
    try:
        server = make_http_server(adapter, args.bind, args.port)
        worker = threading.Thread(target=server.serve_forever, name="arena-loopback-api", daemon=True)
        worker.start()
        print(f"Local arena API listening on loopback port {server.server_address[1]}; waiting for real ROS frames/state", flush=True)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        adapter.close()
        if server is not None:
            server.shutdown()
            server.server_close()
        if worker is not None:
            worker.join(timeout=2.)
        node.destroy_node()
        rclpy.try_shutdown()
        signal.signal(signal.SIGTERM, previous)
        print("Local arena API stopped; frame counters: "+json.dumps(dict(adapter.stats), sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
