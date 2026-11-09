#!/usr/bin/env python3
"""Local primitive UGV models; only the chosen ego owns real rendered sensors."""
from pathlib import Path
import math
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import yaml


def value(parent, name, content):
    node = ET.SubElement(parent, name)
    node.text = " ".join(map(str, content)) if isinstance(content, (tuple, list)) else str(content)
    return node


def shape(parent, kind, size):
    geom = ET.SubElement(parent, "geometry")
    node = ET.SubElement(geom, kind)
    if kind == "box":
        value(node, "size", size)
    elif kind == "sphere":
        value(node, "radius", size)
    else:
        value(node, "radius", size[0])
        value(node, "length", size[1])


def visual(link, name, kind, size, color, pose=None):
    node = ET.SubElement(link, "visual", name=name+"_visual")
    if pose is not None:
        value(node, "pose", pose)
    shape(node, kind, size)
    mat = ET.SubElement(node, "material")
    value(mat, "ambient", color)
    value(mat, "diffuse", color)
    return node


def collision(link, name, kind, size, friction=None):
    node = ET.SubElement(link, "collision", name=name)
    shape(node, kind, size)
    if friction is not None:
        surface = ET.SubElement(node, "surface")
        ode = ET.SubElement(ET.SubElement(surface, "friction"), "ode")
        value(ode, "mu", friction)
        value(ode, "mu2", friction)
    return node


def inertial(link, mass, size):
    node = ET.SubElement(link, "inertial")
    value(node, "mass", mass)
    inertia = ET.SubElement(node, "inertia")
    x, y, z = size
    for key, amount in (("ixx", mass*(y*y+z*z)/12), ("iyy", mass*(x*x+z*z)/12),
                        ("izz", mass*(x*x+y*y)/12), ("ixy", 0), ("ixz", 0), ("iyz", 0)):
        value(inertia, key, amount)


def joint(model, name, parent, child, axis=None, limits=None, settings=None):
    node = ET.SubElement(model, "joint", name=name, type="revolute" if axis else "fixed")
    value(node, "parent", parent)
    value(node, "child", child)
    if axis:
        ax = ET.SubElement(node, "axis")
        value(ax, "xyz", axis)
        if limits:
            limit = ET.SubElement(ax, "limit")
            for key, amount in limits.items():
                value(limit, key, amount)
        if settings:
            dyn = ET.SubElement(ax, "dynamics")
            for key, amount in settings.items():
                value(dyn, key, amount)
    return node


def sensor(link, name, kind, topic, rate, frame):
    node = ET.SubElement(link, "sensor", name=name, type=kind)
    for key, content in (("topic", topic), ("update_rate", rate), ("always_on", "true"),
                         ("visualize", "false"), ("ignition_frame_id", frame)):
        value(node, key, content)
    return node


def wingbot_package():
    """Resolve the model authority in a source tree or an installed ROS workspace."""
    source = Path(__file__).resolve().parents[2]/"wingbot_description"
    if (source/"urdf/wingbot.urdf.xacro").is_file():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("wingbot_description"))


def instantiate_wingbot(name, team, namespace, sensors, wheels, simulator, effective_separation=None):
    """Convert the authoritative xacro; never rebuild its mounts in the arena."""
    if simulator != "fortress":
        raise ValueError("WINGBOT currently supports Gazebo Fortress only; its plugins are not Harmonic plugins")
    root = wingbot_package()
    command = ["xacro", str(root/"urdf/wingbot.urdf.xacro"), f"model_name:={name}",
               f"team:={team}", f"namespace:={namespace}", f"sensors:={sensors}",
               f"wheels:={wheels}", "depth:=false"]
    if effective_separation is not None:
        command.append(f"effective_separation:={float(effective_separation)}")
    urdf = subprocess.run(command, check=True, capture_output=True, text=True).stdout
    with tempfile.TemporaryDirectory(prefix="maicon_wingbot_") as folder:
        path = Path(folder)/"robot.urdf"
        path.write_text(urdf, encoding="utf-8")
        result = subprocess.run(["ign" if simulator == "fortress" else "gz", "sdf", "-p", str(path)],
                                check=True, capture_output=True, text=True)
    model = ET.fromstring(result.stdout[result.stdout.find("<sdf"):]).find("model")
    if model is None:
        raise ValueError("WINGBOT xacro did not produce a model")
    model.set("name", name)
    return model


def add_wingbots(world, spec, profile, ugvs, ego_slot, simulator="fortress", full_sensors=False):
    if simulator != "fortress":
        raise ValueError("WINGBOT currently supports Gazebo Fortress only; its plugins are not Harmonic plugins")
    poses = spec["bases"]["start_pose"]
    count = int(ugvs)
    if count not in range(len(poses)+1) or ego_slot not in poses:
        raise ValueError("ugvs must be 0..6 and ego_slot must be a specified start slot")
    chosen = ([ego_slot]+[slot for slot in poses if slot != ego_slot])[:count]
    metadata = {"robot_models": [], "sensors": {"camera": 0, "gpu_lidar": 0, "imu": 0},
                "ego_model": None, "slots": {}, "model_authority": "wingbot_description",
                "frames_file": str(wingbot_package()/"config/frames.yaml"),
                "lidar_mode": "single360", "lidar_crop_location": "mil_emulator",
                "full_sensors": bool(full_sensors), "namespaces": {}}
    for slot in chosen:
        name, ego = "ugv_"+slot.replace("-", ""), slot == ego_slot
        namespace = ("/arena/robots/"+slot.replace("-", "") if full_sensors else
                     "/arena/ego" if ego else "/"+name)
        owns_sensors = ego or full_sensors
        model = instantiate_wingbot(name, "BLUE" if slot[0] == "B" else "RED", namespace,
                                    "full" if owns_sensors else "none", int(profile["ugv"]["wheel_count"]),
                                    simulator, profile["ugv"].get("effective_separation"))
        pose = model.find("pose")
        if pose is None:
            pose = ET.SubElement(model, "pose")
        x, y, yaw = poses[slot]
        pose.text = " ".join(map(str, (x, y, 0, 0, 0, yaw)))
        # Topic aliases are integration wiring only. Geometry and dynamics stay
        # exactly as generated by the one shared xacro for ego and dummies.
        aliases = {namespace+"/camera/image": namespace+"/image" if full_sensors else "/arena/ego/image",
                   namespace+"/scan": namespace+"/scan_raw" if full_sensors else "/arena/ego/scan_raw",
                   namespace+"/imu": namespace+"/imu/data" if full_sensors else "/imu/data",
                   namespace+"/pan/cmd_pos": namespace+"/pan_cmd",
                   namespace+"/tilt/cmd_pos": namespace+"/tilt_cmd"}
        for element in model.iter():
            if element.text in aliases:
                element.text = aliases[element.text]
        sensors_found = model.findall(".//sensor")
        if not owns_sensors and sensors_found:
            raise ValueError("WINGBOT sensors=none unexpectedly contains sensors")
        for sensor_node in sensors_found:
            kind = sensor_node.get("type")
            if kind in metadata["sensors"]:
                metadata["sensors"][kind] += 1
        world.append(model)
        metadata["robot_models"].append(name)
        metadata["slots"][slot] = name
        metadata["namespaces"][slot] = namespace
        if ego:
            metadata["ego_model"] = name
    metadata["sensor_count"] = sum(metadata["sensors"].values())
    return metadata


def add_ugvs(world, spec, profile, ugvs, ego_slot, simulator="fortress", *, system_adder=None, render_engine="ogre2", robot_model="wingbot", full_sensors=False):
    """Return names, sensor counts and ROS/Gazebo topic metadata for one ego."""
    if robot_model == "wingbot":
        if int(ugvs) and render_engine != "ogre2":
            raise ValueError("WINGBOT full360 LiDAR requires the verified Ogre2 sensor server")
        return add_wingbots(world, spec, profile, ugvs, ego_slot, simulator, full_sensors=full_sensors)
    if full_sensors:
        raise ValueError("full_sensors requires the authoritative WINGBOT model")
    if robot_model != "legacy":
        raise ValueError("robot_model must be wingbot or explicit legacy")
    if system_adder is None:
        from gen_world import add_system
        system_adder = add_system
    count = int(ugvs)
    poses = spec["bases"]["start_pose"]
    if count not in range(len(poses)+1) or ego_slot not in poses:
        raise ValueError("ugvs must be 0..6 and ego_slot must be a specified start slot")
    root = Path(__file__).resolve().parents[1]
    if not (root/"config/robot_assumptions.yaml").is_file():
        from ament_index_python.packages import get_package_share_directory
        root = Path(get_package_share_directory("maicon_arena_gz"))
    a = yaml.safe_load((root/"config/robot_assumptions.yaml").read_text(encoding="utf-8"))
    settings, robot = profile["ugv"], spec["ugv"]
    chosen = ([ego_slot]+[slot for slot in poses if slot != ego_slot])[:count]
    metadata = {"robot_models": [], "sensors": {"camera": 0, "gpu_lidar": 0, "imu": 0},
                "ego_model": None, "slots": {}, "assumptions": "config/robot_assumptions.yaml"}
    for slot in chosen:
        ego = slot == ego_slot
        name = "ugv_"+slot.replace("-", "")
        metadata["robot_models"].append(name)
        metadata["slots"][slot] = name
        model = ET.SubElement(world, "model", name=name)
        x, y, yaw = poses[slot]
        value(model, "pose", [x, y, 0, 0, 0, yaw])
        value(model, "static", "false")
        base = ET.SubElement(model, "link", name="base_link")
        if not ego:
            value(base, "gravity", "false")
        body = a["body"]
        size = body["size"] if ego else robot["footprint_lwh"]
        center = body["center"] if ego else [0, 0, size[2]/2]
        value(base, "pose", [*center, 0, 0, 0])
        mass = robot["mass"]
        wheel_count = int(settings["wheel_count"])
        attachments = wheel_count*a["wheel"]["mass_fraction_each"] + 2*a["gimbal"]["mass_fraction_each"]
        if wheel_count == 2:
            attachments += len(a["caster"]["x_positions"])*a["caster"]["mass_fraction_each"]
        inertial(base, mass*(1-attachments) if ego else mass, size)
        collision(base, "body", "box", size, spec["floor"]["friction_mu"])
        visual(base, "body", "box", body["size"], a["colors"]["body"],
               [body["center"][i]-center[i] for i in range(3)]+[0, 0, 0])
        for part, kind, dim, clr, xyz in [
                ("antenna", "box", a["antenna"]["size"], a["colors"]["BLUE" if slot[0] == "B" else "RED"], a["antenna"]["center"]),
                *[(f"receiver_{i}", "sphere", a["receiver"]["radius"], a["colors"]["receiver"], p)
                  for i, p in enumerate(a["receiver"]["centers"])]]:
            visual(base, part, kind, dim, clr, [xyz[i]-center[i] for i in range(3)]+[0, 0, 0])
        wheel = a["wheel"]
        wheel_x = wheel["x_positions"] if settings["wheel_count"] == 6 else [0]
        for side, sign in (("left", 1), ("right", -1)):
            for i, xx in enumerate(wheel_x):
                pos = [xx, sign*wheel["track"]/2, wheel["radius"]]
                if ego:
                    link_name = f"{side}_wheel_{i}"
                    link = ET.SubElement(model, "link", name=link_name)
                    value(link, "pose", [*pos, math.pi/2, 0, 0])
                    inertial(link, mass*wheel["mass_fraction_each"], [2*wheel["radius"], 2*wheel["radius"], wheel["width"]])
                    collision(link, "wheel", "cylinder", [wheel["radius"], wheel["width"]], wheel["friction"])
                    visual(link, "wheel", "cylinder", [wheel["radius"], wheel["width"]], a["colors"]["wheel"])
                    joint(model, link_name+"_joint", "base_link", link_name, [0, 0, -1], settings=a["joint"])
                else:
                    visual(base, f"{side}_wheel_{i}", "cylinder", [wheel["radius"], wheel["width"]], a["colors"]["wheel"],
                           [pos[j]-center[j] for j in range(3)]+[math.pi/2, 0, 0])
        if not ego:
            drive = system_adder(model, "VelocityControl", simulator)
            value(drive, "topic", f"/{name}/cmd_vel")
            continue
        metadata["ego_model"] = name
        if settings["wheel_count"] == 2:
            for i, xx in enumerate(a["caster"]["x_positions"]):
                rad = a["caster"]["radius"]
                link = ET.SubElement(model, "link", name=f"caster_{i}")
                value(link, "pose", [xx, 0, rad, 0, 0, 0])
                inertial(link, mass*a["caster"]["mass_fraction_each"], [2*rad]*3)
                collision(link, "caster", "sphere", rad, a["caster"]["friction"])
                joint(model, f"caster_{i}_joint", "base_link", f"caster_{i}")
        drive = system_adder(model, "DiffDrive", simulator)
        for side in ("left", "right"):
            for i in range(len(wheel_x)):
                value(drive, side+"_joint", f"{side}_wheel_{i}_joint")
        for key, amount in {"wheel_separation": wheel["track"], "wheel_radius": wheel["radius"],
                            "odom_publish_frequency": robot["odom_rate"], "topic": "/arena/ego/cmd_vel",
                            "odom_topic": "/arena/ego/odom", "tf_topic": "/arena/ego/odom_tf",
                            "frame_id": "odom", "child_frame_id": "base_link"}.items():
            value(drive, key, amount)
        g = a["gimbal"]
        for axis_name, parent, vec, limit_deg in (("pan", "base_link", [0, 0, 1], 100*robot["pan_tilt"]["pan_deg_per_unit"]),
                                                 ("tilt", "pan_link", [0, -1, 0], 100*robot["pan_tilt"]["tilt_deg_per_unit"])):
            link = ET.SubElement(model, "link", name=axis_name+"_link")
            value(link, "pose", [*g["pivot"], 0, 0, 0])
            inertial(link, mass*g["mass_fraction_each"], g[axis_name+"_size"])
            visual(link, axis_name, "box", g[axis_name+"_size"], a["colors"]["body"])
            joint(model, axis_name+"_joint", parent, axis_name+"_link", vec,
                  {"lower": -math.radians(limit_deg), "upper": math.radians(limit_deg), "effort": g["effort"], "velocity": g["velocity"]}, a["joint"])
            plugin = system_adder(model, "JointPositionController", simulator)
            value(plugin, "joint_name", axis_name+"_joint")
            value(plugin, "topic", "/arena/ego/"+axis_name+"_cmd")
            value(plugin, "p_gain", g["p_gain"])
        state_pub = system_adder(model, "JointStatePublisher", simulator)
        value(state_pub, "topic", "/arena/ego/joint_states")
        value(state_pub, "joint_name", "pan_joint")
        value(state_pub, "joint_name", "tilt_joint")
        laser = ET.SubElement(model, "frame", name="laser_frame_emitter", attached_to="tilt_link")
        value(laser, "pose", [*a["camera"]["offset"], 0, 0, 0])
        tilt = model.find("link[@name='tilt_link']")
        cam = sensor(tilt, "front_camera", "camera", "/arena/ego/image", settings["camera_rate"], "camera_optical_frame")
        value(cam, "pose", [*a["camera"]["offset"], 0, 0, 0])
        cp = ET.SubElement(cam, "camera")
        value(cp, "horizontal_fov", a["camera"]["hfov"])
        im = ET.SubElement(cp, "image")
        for key, amount in zip(("width", "height"), robot["camera"]["resolution"]):
            value(im, key, amount)
        value(im, "format", "R8G8B8")
        clip = ET.SubElement(cp, "clip")
        for key, amount in zip(("near", "far"), a["camera"]["clip"]):
            value(clip, key, amount)
        distortion = ET.SubElement(cp, "distortion")
        for key, amount in a["camera"]["distortion"].items():
            value(distortion, key, amount)
        pos = a["lidar"]["pose"]
        total_samples = settings.get("lidar_samples", robot["lidar"]["sim_samples_assumed"])
        sector_mode = render_engine == "ogre"
        yaws = a["lidar"]["sector_yaws"] if sector_mode else [0]
        span = a["lidar"]["sector_span"] if sector_mode else 2*math.pi
        ray_count = math.ceil(total_samples/len(yaws))+1 if sector_mode else total_samples
        for index, yaw in enumerate(yaws):
            topic = f"/arena/scan_sector_{index}" if sector_mode else "/scan"
            frame = f"laser_sector_{index}" if sector_mode else "laser_frame"
            scan = sensor(base, f"lidar_{index}", "gpu_lidar", topic, settings["lidar_rate"], frame)
            value(scan, "pose", [pos[i]-center[i] for i in range(3)]+[pos[3], pos[4], pos[5]+yaw])
            lidar = ET.SubElement(scan, "lidar")
            horizontal = ET.SubElement(ET.SubElement(lidar, "scan"), "horizontal")
            for key, amount in (("samples", ray_count), ("resolution", 1), ("min_angle", -span/2), ("max_angle", span/2)):
                value(horizontal, key, amount)
            rg = ET.SubElement(lidar, "range")
            for key, amount in (("min", a["lidar"]["range"][0]), ("max", a["lidar"]["range"][1]), ("resolution", a["lidar"]["range_resolution"])):
                value(rg, key, amount)
        sensor(base, "imu", "imu", "/imu/data", settings["imu_rate"], "base_link")
        metadata["sensors"] = {"camera": 1, "gpu_lidar": len(yaws), "imu": 1}
        metadata["lidar_mode"] = "sectors360" if sector_mode else "single360"
        metadata["lidar_output_samples"] = total_samples
    metadata["sensor_count"] = sum(metadata["sensors"].values())
    return metadata
