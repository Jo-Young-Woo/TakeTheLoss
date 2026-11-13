"""Single-WINGBOT Nav2 trial configuration from model metadata, never world poses."""
from pathlib import Path
import json
import yaml

ROOT = Path(__file__).resolve().parents[1]


def configure(cfg, spec, frames):
    lo, hi = frames['visual_bounds']['min'], frames['visual_bounds']['max']
    cfg.robot.length = 2*max(abs(float(lo[0])), abs(float(hi[0])))
    cfg.robot.width = 2*max(abs(float(lo[1])), abs(float(hi[1])))
    cfg.nav.backend = 'nav2'
    cfg.nav2.base_frame = 'base_footprint'
    cfg.nav2.use_sim_time = True
    cfg.nav2.allow_builtin_fallback = False
    cfg.nav2.publish_depth_obstacles = False
    # The documented, configured starting base is a permitted initial prior.
    start = spec['bases']['start_pose']['B-2']
    cfg.nav2.start_pose_override = (float(start[0])+spec['floor']['size'][0]/2,
                                    float(start[1])+spec['floor']['size'][1]/2, float(start[2]))
    return cfg


def prepare(output, spec_path):
    from maicon26.launch_profile import load_run_config
    cfg, _ = load_run_config(str(ROOT/'config/waveshare_real.json'), None, 'nav2')
    spec = yaml.safe_load(Path(spec_path).read_text())
    frames = yaml.safe_load((ROOT/'ros2/wingbot_description/config/frames.yaml').read_text())
    configure(cfg, spec, frames)
    params = yaml.safe_load((ROOT/'ros2/config/nav2_params.yaml').read_text())
    # Full-face observations/refinements request 3 cm arrival accuracy.
    # Nav2 must not declare success at the old 8 cm radius or substitute
    # a planner endpoint up to 20 cm away from the selected viewpoint.
    params['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance'] = .025
    params['planner_server']['ros__parameters']['GridBased']['tolerance'] = .05
    fp = cfg.budget().nav2_footprint
    for name in ('local_costmap','global_costmap'):
        c = params[name][name]['ros__parameters']
        c.pop('robot_radius', None)
        c['footprint'] = json.dumps(fp)
        c['footprint_padding'] = 0.
    dest = Path(output)/'nav2_wingbot.yaml'
    dest.write_text(yaml.safe_dump(params, sort_keys=False), encoding='utf8')
    return ['ros2','launch',str(ROOT/'ros2/launch/maicon_nav.launch.py'),
            'slam:=cartographer','nav2:=true','use_sim_time:=true','odom_tf:=off',
            'base_frame:=base_footprint','cartographer_config:=cartographer_wingbot_sim.lua',
            'scan_topic:=/scan','odom_topic:=/odom','nav_cmd_topic:=/cmd_vel_nav2',
            'depth_obstacles:=false','params_file:='+str(dest)]
