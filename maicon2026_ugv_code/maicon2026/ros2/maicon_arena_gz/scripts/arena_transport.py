"""Explicit process-local image transport selection; no shell/OS mutations."""
from pathlib import Path


def image_transport_environment(profile, package_root, environ):
    """Return only environment overrides for this launch's child processes.

    Default ``inherit`` does not touch either Fast DDS profile variable or RMW.
    Explicit ``shm64`` selects the packaged profile, overriding an inherited
    profile intentionally; another explicitly selected middleware is rejected.
    """
    if profile == "inherit":
        return {}
    if profile not in ("shm64", "shm256"):
        raise ValueError("image_transport_profile must be inherit or shm64 or shm256")
    rmw = environ.get("RMW_IMPLEMENTATION", "")
    if rmw and rmw not in ("rmw_fastrtps_cpp", "rmw_fastrtps_dynamic_cpp"):
        raise ValueError(profile+" requires Fast DDS; use inherit for another RMW")
    path = Path(package_root)/("config/fastdds_"+profile+".xml")
    if not path.is_file():
        raise ValueError("Packaged Fast DDS "+profile+" profile is missing")
    return {"FASTRTPS_DEFAULT_PROFILES_FILE": str(path.resolve()),
            "FASTDDS_DEFAULT_PROFILES_FILE": str(path.resolve()),
            "RMW_IMPLEMENTATION": rmw or "rmw_fastrtps_cpp"}
