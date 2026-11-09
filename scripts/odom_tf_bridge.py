#!/usr/bin/env python3
"""/odom → TF odom→base_link (로봇이 TF 를 안 낼 때). 자세한 설명은 maicon26/hal/odom_tf.py.
    source /opt/ros/humble/setup.bash && python3 scripts/odom_tf_bridge.py            # 이미 TF 가 있으면 쉰다
    python3 scripts/odom_tf_bridge.py --force                                         # 무조건 발행
정적 TF(base_link→laser 등)는 ros2 launch ros2/launch/robot_description.launch.py 가 낸다."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maicon26.hal.odom_tf import main

if __name__ == "__main__":
    sys.exit(main())
