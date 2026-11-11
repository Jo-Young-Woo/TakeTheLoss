#!/usr/bin/env python3
"""Build an atomic source release without recordings, credentials or build caches.

Uses the standard library and never contacts a network. Known active credential
values are checked case-insensitively in member names and bytes. This is a leak
guard, not a way to discover arbitrary unknown credentials: supply --exclude for
any nonstandard code file and --secret-env for a custom LLM key environment name.
"""
from __future__ import annotations

import argparse
from collections import Counter
import datetime as dt
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import sys
import tempfile
import urllib.parse
import zipfile


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = "docs/RELEASE_MANIFEST.json"
REQUIRED = frozenset({
    "README.md", "pytest.ini", "maicon26/__init__.py", "scripts/run_real.py", "tests/conftest.py",
    "config/default.json", "docs/source/MAICON2026_game-server-api_p0-p12.html",
    "ros2/gazebo/maicon_match_rover.sdf", "ros2/gazebo/uav_grid_calibration.json",
    "ros2/maicon_arena_gz/package.xml", "ros2/maicon_arena_gz/CMakeLists.txt",
    "ros2/maicon_arena_gz/README.md", "ros2/maicon_arena_gz/config/bridge.yaml",
    "ros2/maicon_arena_gz/launch/arena.launch.py", "ros2/maicon_arena_gz/config/arena_spec.yaml",
    "ros2/maicon_arena_gz/config/layouts/layout_default.yaml",
    "ros2/maicon_arena_gz/config/profiles/standard.yaml", "ros2/maicon_arena_gz/config/robot_assumptions.yaml",
    "ros2/maicon_arena_gz/config/uav_grid_calibration.json",
    "ros2/maicon_arena_gz/scripts/gen_world.py", "ros2/maicon_arena_gz/scripts/robot_models.py",
    "ros2/maicon_arena_gz/scripts/arena_common.py", "ros2/maicon_arena_gz/scripts/make_atlas.py",
    "ros2/maicon_arena_gz/scripts/arena_transport.py", "ros2/maicon_arena_gz/config/fastdds_shm64.xml",
    "ros2/maicon_arena_gz/scripts/gen_layout.py", "ros2/maicon_arena_gz/scripts/check_world.py",
    "ros2/maicon_arena_gz/scripts/mil_emulator.py", "ros2/maicon_arena_gz/scripts/lidar_sector_merger.py",
    "ros2/maicon_arena_gz/scripts/arena_referee.py", "ros2/maicon_arena_gz/src/opaque_material_system.cc",
    "ros2/mil_interfaces_sim/package.xml", "ros2/mil_interfaces_sim/CMakeLists.txt",
    "ros2/mil_interfaces_sim/msg/Match.msg", "ros2/mil_interfaces_sim/srv/Fire.srv",
    "scripts/build_arena_gz.sh", "scripts/run_arena_mock_api.py", "maicon26/sim/gazebo_api.py",
    "ros2/maicon_arena_gz/media/textures/floor_print_2400x1920.png",
    "ros2/maicon_arena_gz/media/textures/floor_print_1200x960_lite.png",
    "ros2/maicon_arena_gz/models/cube_atlas/meshes/cube.obj",
    "ros2/maicon_arena_gz/models/floor/meshes/floor.obj",
    *("ros2/maicon_arena_gz/media/textures/faces/" + name + ".jpg" for name in (
        "m3_tank", "m3_mortar", "m3_missile", "m3_military_truck", "m3_ammo_box", "m3_drum",
        "target_comms", "target_outpost", "target_radar",
        "deco_barrier_wall", "deco_city_tree", "deco_field_bunker", "deco_ruined_building")),
})
OMIT_DIRS = frozenset({
    ".git", ".hg", ".svn", "__pycache__", ".pytest_cache", ".mypy_cache", ".ruff_cache",
    ".cache", ".tox", ".nox", ".venv", "venv", "env", "node_modules", "tmp", "temp", "logs",
    "log", "recordings", "build", "dist", "install", ".secrets", ".ssh",
})
OMIT_SUFFIXES = frozenset({
    ".pyc", ".pyo", ".log", ".out", ".prof", ".tmp", ".temp", ".bak", ".swp",
    ".zip", ".7z", ".rar", ".tar", ".gz", ".bz2", ".xz", ".tgz",
    ".mp4", ".mkv", ".avi", ".mov", ".webm", ".bag", ".mcap", ".db3",
})
SECRET_ENV_NAMES = frozenset({
    "MAICON_CODE", "MAICON_MISSION_CODE", "MAICON_LLM_API_KEY", "MAICON_LLM_KEY",
    "LLM_API_KEY", "OPENAI_API_KEY", "ANTHROPIC_API_KEY",
})
SECRET_STEM = re.compile(
    r"^(?:mission[_-]?code|maicon[_-]?(?:mission[_-]?)?code|code[_-]?file|"
    r"credentials?|secrets?|tokens?|api[_-]?keys?|passwords?)(?:[._-].*)?$", re.I)


class ReleaseError(ValueError):
    """Safe diagnostic. Never contains credential values, paths or raw OS errors."""


def known_secrets(environ=None, extra_names=()):
    """Read values in memory only; callers never receive diagnostic value echoes."""
    env = os.environ if environ is None else environ
    names = set(SECRET_ENV_NAMES) | {str(name) for name in extra_names}
    names.update(name for name in env if re.search(r"(?:LLM|OPENAI|ANTHROPIC).*_(?:KEY|TOKEN)$", name, re.I))
    values = set()
    for name in names:
        value = env.get(name)
        if value:
            values.update((str(value), str(value).strip()))
    # Encoded query/JSON copies are also possible leak locations.
    return tuple(sorted({variant for value in values if value for variant in (
        value, urllib.parse.quote(value, safe=""), urllib.parse.quote_plus(value),
        json.dumps(value, ensure_ascii=True)[1:-1])}, key=len, reverse=True))


def _contains_secret(data, values):
    if not values:
        return False
    raw = data if isinstance(data, bytes) else str(data).encode("utf-8")
    # Byte matching covers binary files and UTF-16 credential files too.
    lowered = raw.lower()
    for value in values:
        for encoding in ("utf-8", "utf-16-le", "utf-16-be"):
            if value.encode(encoding).lower() in lowered:
                return True
    return any(value.casefold() in raw.decode("utf-8", errors="ignore").casefold() for value in values)


def _credential_name(path):
    name = path.name.lower()
    if name == ".env" or name.startswith(".env.") or name in (".netrc", "_netrc", ".pypirc"):
        return True
    if path.suffix.lower() in (".pem", ".key", ".p12", ".pfx") or name.startswith("id_rsa") or name.startswith("id_ed25519"):
        return True
    # credentials.py / link/secrets.py are runtime code, not credential files.
    return path.suffix.lower() != ".py" and bool(SECRET_STEM.match(path.stem.lstrip(".")))


def _relative_exclusions(exclude):
    result = []
    for item in exclude:
        value = str(item).replace("\\", "/")
        path = PurePosixPath(value)
        if not value or value == "." or path.is_absolute() or ".." in path.parts or ":" in value:
            raise ReleaseError("Exclusions must be relative paths inside the release root")
        result.append(path.as_posix().rstrip("/"))
    return tuple(result)


def exclusion_reason(relative, *, is_dir=False, exclude=()):
    """Return a stable reason; never store excluded file names in a manifest."""
    path = PurePosixPath(relative)
    lower_parts = tuple(part.lower() for part in path.parts)
    if any(relative == name or relative.startswith(name + "/") for name in exclude):
        return "explicit_path"
    if any(part in OMIT_DIRS for part in lower_parts):
        return "generated_directory"
    if any(part.endswith(".egg-info") for part in lower_parts):
        return "build_metadata"
    if path.as_posix() == MANIFEST:
        return "generated_manifest"
    if _credential_name(path):
        return "credential_name"
    if is_dir:
        return None
    if path.suffix.lower() in OMIT_SUFFIXES or path.name.endswith("~"):
        return "generated_or_archive_file"
    if path.name.lower() in (".coverage", "coverage.xml", "thumbs.db", ".ds_store"):
        return "generated_file"
    return None


def _unsafe_link(path, info):
    # Junctions are not always reported as symbolic links on Windows Python.
    return (stat.S_ISLNK(info.st_mode) or bool(getattr(info, "st_file_attributes", 0) & 0x400)
            or getattr(path, "is_junction", lambda: False)())


def _inside(path, root):
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def _read_source(path, root):
    """Reject links/special files and detect replacement/modification while read."""
    before = path.lstat()
    if _unsafe_link(path, before) or not stat.S_ISREG(before.st_mode):
        raise ReleaseError("Release input contains a link or special file")
    if not _inside(path.resolve(strict=True), root):
        raise ReleaseError("Release input resolves outside the source root")
    fd = os.open(path, os.O_RDONLY | getattr(os, "O_BINARY", 0) | getattr(os, "O_NOFOLLOW", 0))
    with os.fdopen(fd, "rb") as stream:
        opened = os.fstat(stream.fileno())
        if (opened.st_dev, opened.st_ino) != (before.st_dev, before.st_ino):
            raise ReleaseError("Release input changed while packaging")
        data = stream.read()
        after = os.fstat(stream.fileno())
    final = path.lstat()
    if (_unsafe_link(path, final) or not _inside(path.resolve(strict=True), root)
            or (final.st_dev, final.st_ino) != (before.st_dev, before.st_ino)
            or (after.st_size, after.st_mtime_ns) != (before.st_size, before.st_mtime_ns)
            or (final.st_size, final.st_mtime_ns) != (before.st_size, before.st_mtime_ns)):
        raise ReleaseError("Release input changed while packaging")
    return data, before


def _walk_sources(root, exclude, counts):
    pending = [root]
    while pending:
        directory = pending.pop()
        for path in sorted(directory.iterdir(), key=lambda p: p.name):
            relative = path.relative_to(root).as_posix()
            info = path.lstat()
            is_dir = stat.S_ISDIR(info.st_mode)
            reason = exclusion_reason(relative, is_dir=is_dir, exclude=exclude)
            if reason:
                counts[reason] += 1  # Counts excluded roots/files; never follow excluded directories.
                continue
            if _unsafe_link(path, info):
                raise ReleaseError("Release input contains a symbolic link or junction")
            if not _inside(path.resolve(strict=True), root):
                raise ReleaseError("Release input resolves outside the source root")
            if is_dir:
                pending.append(path)
            elif stat.S_ISREG(info.st_mode):
                yield relative, path
            else:
                raise ReleaseError("Release input contains a special file")


def _member(name, data, mode=0o644):
    info = zipfile.ZipInfo(name, date_time=(2026, 11, 4, 0, 0, 0))
    info.compress_type = zipfile.ZIP_DEFLATED
    info.create_system = 3
    info.external_attr = (stat.S_IFREG | mode) << 16
    return info, data


def build_release(root, output, *, overwrite=False, environ=None, exclude=(), secret_env=()):
    """Write a validated ZIP atomically; on failure preserve any existing output.

    Archive members are relative to the project root (no extra directory layer).
    Returns a manifest without secret values or excluded source paths. Source
    files are only read; the sole persistent output is the requested ZIP.
    """
    temporary = None
    try:
        root = Path(root).absolute()
        if not root.is_dir() or _unsafe_link(root, root.lstat()):
            raise ReleaseError("Release root must be a regular source directory")
        root = root.resolve(strict=True)
        output = Path(output).absolute()
        if output.suffix.lower() != ".zip" or not output.parent.is_dir():
            raise ReleaseError("Output must be a ZIP path in an existing directory")
        if output.is_symlink() or (output.exists() and _unsafe_link(output, output.lstat())):
            raise ReleaseError("Output must not be a link")
        if output.exists() and not overwrite:
            raise ReleaseError("Output already exists; explicit overwrite is required")
        excludes = _relative_exclusions(exclude)
        values = known_secrets(environ, secret_env)
        if _contains_secret(output.name, values):
            raise ReleaseError("Active credential detected in release output name")
        counts = Counter()
        candidates = sorted(_walk_sources(root, excludes, counts))
        paths = {relative for relative, _ in candidates}
        if not REQUIRED <= paths:
            raise ReleaseError("Required source, test, specification or Gazebo assets are missing")
        # Case collisions cannot be safely extracted on Windows.
        if len({name.casefold() for name in paths}) != len(paths):
            raise ReleaseError("Release paths collide on a case-insensitive filesystem")
        fd, temporary = tempfile.mkstemp(prefix=".maicon-release-", suffix=".tmp", dir=output.parent)
        os.close(fd)
        members = []
        with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
            for relative, path in candidates:
                data, info = _read_source(path, root)
                if _contains_secret(relative, values) or _contains_secret(data, values):
                    raise ReleaseError("Active credential detected in release input; archive was not published")
                member, content = _member(relative, data, 0o755 if info.st_mode & 0o111 else 0o644)
                archive.writestr(member, content)
                members.append({"path": relative, "size": len(data), "sha256": hashlib.sha256(data).hexdigest()})
            manifest = {
                "schema": "maicon_source_release_v1", "files": members,
                "file_count": len(members), "source_bytes": sum(m["size"] for m in members),
                "excluded_entry_counts": dict(sorted(counts.items())),
                "exclusions": {"directories": sorted(OMIT_DIRS), "suffixes": sorted(OMIT_SUFFIXES),
                               "credential_names": True, "explicit_path_count": len(excludes)},
                "recordings_included": False,
                "notes": ["Recordings and runtime logs are provided separately; README media links may point outside this release.",
                          "Active environment credential values were scanned case-insensitively; unknown credentials require explicit exclusion.",
                          "File hashes describe source entries; this manifest does not hash itself.",
                          "Existing dependencies documented in README are required; no third-party environment is bundled."],
            }
            data = json.dumps(manifest, ensure_ascii=False, indent=2).encode("utf-8") + b"\n"
            if _contains_secret(data, values):
                raise ReleaseError("Active credential detected in generated manifest")
            member, content = _member(MANIFEST, data)
            archive.writestr(member, content)
        with zipfile.ZipFile(temporary) as archive:
            if archive.testzip() is not None:
                raise ReleaseError("Release archive integrity validation failed")
        with open(temporary, "rb") as stream:
            os.fsync(stream.fileno())
        if overwrite:
            os.replace(temporary, output)
            temporary = None
        else:
            # Atomic no-clobber publication, including a competing output creator.
            # The temporary file and destination share a filesystem.
            os.link(temporary, output)
        return manifest
    except ReleaseError:
        raise
    except (OSError, ValueError, TypeError, zipfile.BadZipFile):
        raise ReleaseError("Release packaging failed; no source files were modified") from None
    finally:
        if temporary is not None:
            try:
                Path(temporary).unlink(missing_ok=True)
            except OSError:
                pass


class _SafeParser(argparse.ArgumentParser):
    def error(self, message):
        # argparse's ordinary error echoes unknown arguments. Never echo argv.
        raise ReleaseError("Invalid packaging arguments; use --help for accepted options")


def main(argv=None):
    parser = _SafeParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path,
                        help="ZIP path; default is the project parent/maicon2026_ugv_code_YYYYMMDD.zip")
    parser.add_argument("--overwrite", action="store_true", help="Explicitly replace an existing release ZIP after validation")
    parser.add_argument("--exclude", action="append", default=[], help="Extra relative credential file/directory to omit")
    parser.add_argument("--secret-env", action="append", default=[], help="Additional environment VARIABLE NAME to scan, never a secret value")
    try:
        args = parser.parse_args(argv)
        output = args.output or args.root.parent / f"maicon2026_ugv_code_{dt.date.today():%Y%m%d}.zip"
        manifest = build_release(args.root, output, overwrite=args.overwrite,
                                 exclude=args.exclude, secret_env=args.secret_env)
    except ReleaseError as error:
        print(str(error), file=sys.stderr)
        return 1
    print(f"Release ZIP validated: {manifest['file_count']} source files; recordings excluded")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
