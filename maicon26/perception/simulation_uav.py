"""Opt-in measured Gazebo image geometry; never enabled by real defaults."""
import math

from .uav_calibration import load_calibration
from .uav_layout import FixedUavLayoutDetector


def load_simulation_uav_detector(calibration_path, arena):
    """Require a measured arena and independently checked matching calibration.

    Both use the nonnegative navigation frame. No centred Gazebo model pose,
    layout YAML, side photographs or M3 labels enter this detector.
    """
    if not getattr(arena, "measured_sectors", None) or not getattr(arena, "measured_starts", None):
        raise ValueError("Simulation UAV calibration requires a measured arena specification")
    calibration = load_calibration(calibration_path)
    if not (math.isclose(calibration.arena_width, arena.W, abs_tol=1e-6, rel_tol=0.) and
            math.isclose(calibration.arena_height, arena.H, abs_tol=1e-6, rel_tol=0.)):
        raise ValueError("Simulation UAV calibration and measured arena dimensions differ")
    if not math.isclose(arena.box_size, .4, abs_tol=1e-6, rel_tol=0.):
        raise ValueError("Fixed simulation UAV detector supports 0.4 metre cubes only")
    if not math.isclose(calibration.object_plane_z, arena.box_size, abs_tol=.001, rel_tol=0.):
        raise ValueError("Simulation UAV calibration object plane differs from cube height")
    if calibration.validation_report()["status"] != "within_tolerance":
        raise ValueError("Simulation UAV calibration requires passing independent checkpoints")
    return FixedUavLayoutDetector(calibration)


def load_simulation_ugv_detector(calibration_path, arena):
    """Image-only team sleeve pairs, with the same validated grid geometry."""
    from .wingbot_vision import WingbotUavDetector
    layout = load_simulation_uav_detector(calibration_path, arena)
    return WingbotUavDetector(layout.calibration)
