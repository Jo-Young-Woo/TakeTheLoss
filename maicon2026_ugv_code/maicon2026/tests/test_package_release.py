"""Release contents, leak guards and atomic publication using invented fixtures."""
import hashlib
import json
import os
from pathlib import Path
import zipfile

import pytest

from scripts import package_release as release


def source_tree(tmp_path):
    root = tmp_path / "source"
    root.mkdir()
    for relative in release.REQUIRED:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("source fixture\n", encoding="utf-8")
    return root


def write(root, relative, content=b"fixture"):
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)
    return path


@pytest.mark.spec("S0.4", "S0.13")
def test_release_keeps_runtime_source_assets_calibration_and_specification_but_omits_generated_and_secrets(tmp_path):
    root = source_tree(tmp_path)
    retained = ["pyproject.toml", "requirements.txt", "maicon26/credentials.py", "maicon26/link/secrets.py",
                "tests/test_spec_credentials.py", "team/team_node.py", "ros2/launch/robot.launch.py",
                "ros2/gazebo/assets/ugv_rover/base_link.stl", "ros2/gazebo/assets/m3_photos/TANK.png",
                "ros2/gazebo/uav_grid_measurements.json", "config/robot_calibrated.json", "docs/API_TRACEABILITY.md",
                "ros2/maicon_arena_gz/hooks/arena_resources.dsv.in",
                "ros2/maicon_arena_gz/config/profiles/lite.yaml",
                "ros2/maicon_arena_gz/config/profiles/high.yaml",
                "ros2/maicon_arena_gz/media/textures/floor_print_2400x1920.png",
                "ros2/maicon_arena_gz/media/textures/faces/m3_tank.jpg",
                "ros2/maicon_arena_gz/models/cube_atlas/meshes/cube.obj",
                "ros2/maicon_arena_gz/validation/uav_grid_overlay.png",
                "ros2/maicon_arena_gz/validation/runtime_gpu_gl43.json",
                "ros2/mil_interfaces_sim/msg/State.msg", "ros2/mil_interfaces_sim/srv/Gimbal.srv"]
    removed = ["__pycache__/module.pyc", "maicon26/__pycache__/x.pyc", ".git/config", ".env",
               ".env.production", ".mission_code", ".credentials", "config/mission_code.txt", "code-file", "config/credentials.json",
               "config/secrets.local.yaml", "token.txt", "password.txt", "private.key", "id_rsa",
               "tmp/report.json", "logs/run.log", "foo.log", "prof.out", "old.zip", "old.tar.gz",
               "ros2/gazebo/recordings/movie.mp4", "ros2/gazebo/recordings/report.json", "demo.webm",
               ".pytest_cache/v/cache/nodeids", "build/generated.py", "dist/old.whl",
               "ros2/maicon_arena_gz/build/libmaicon_opaque_material_system.so",
               "ros2/maicon_arena_gz/install/local_setup.bash",
               "ros2/maicon_arena_gz/recordings/new_arena.mp4"]
    for name in retained + removed:
        write(root, name)
    before = {p.relative_to(root).as_posix(): p.read_bytes() for p in root.rglob("*") if p.is_file()}
    output = tmp_path / "release.zip"
    manifest = release.build_release(root, output, environ={})
    with zipfile.ZipFile(output) as archive:
        names = set(archive.namelist())
        assert set(retained) | release.REQUIRED | {release.MANIFEST} <= names
        assert names.isdisjoint(removed)
        assert archive.testzip() is None
        embedded = json.loads(archive.read(release.MANIFEST))
        assert embedded == manifest
        for member in embedded["files"]:
            assert hashlib.sha256(archive.read(member["path"])).hexdigest() == member["sha256"]
        assert not embedded["recordings_included"]
    assert before == {p.relative_to(root).as_posix(): p.read_bytes() for p in root.rglob("*") if p.is_file()}


@pytest.mark.spec("S0.4")
@pytest.mark.parametrize("key", ["MAICON_CODE", "MAICON_MISSION_CODE", "MAICON_LLM_API_KEY", "OPENAI_API_KEY"])
@pytest.mark.parametrize("encoding", ["utf-8", "utf-16-le", "utf-16-be"])
def test_active_environment_secret_in_text_or_binary_aborts_without_value_echo(tmp_path, key, encoding, capsys):
    root = source_tree(tmp_path)
    secret = "FiXtUrE9"
    write(root, "config/unexpected.dat", ("prefix " + secret.swapcase() + " suffix").encode(encoding))
    output = tmp_path / "release.zip"
    with pytest.raises(release.ReleaseError) as error:
        release.build_release(root, output, environ={key: secret})
    assert secret.casefold() not in str(error.value).casefold()
    captured = capsys.readouterr()
    assert secret.casefold() not in (captured.out + captured.err).casefold()
    assert not output.exists()
    assert not list(tmp_path.glob(".maicon-release-*"))


@pytest.mark.spec("S0.4")
def test_known_secret_in_filename_aborts_and_manifest_never_records_excluded_secret_names(tmp_path):
    root = source_tree(tmp_path)
    secret = "SeCrEt27"
    leak = write(root, "config/" + secret.lower() + ".txt")
    with pytest.raises(release.ReleaseError) as error:
        release.build_release(root, tmp_path / "release.zip", environ={"MAICON_CODE": secret})
    assert secret.casefold() not in str(error.value).casefold()
    manifest = release.build_release(root, tmp_path / "safe.zip", environ={"MAICON_CODE": secret},
                                     exclude=(leak.relative_to(root).as_posix(),))
    assert secret.casefold() not in json.dumps(manifest).casefold()


@pytest.mark.spec("S0.4")
def test_custom_llm_environment_and_url_encoded_value_are_scanned(tmp_path):
    root = source_tree(tmp_path)
    secret = "fixture/key+with spaces"
    write(root, "config/accidental.txt", b"fixture%2fkey%2bwith%20spaces")
    with pytest.raises(release.ReleaseError):
        release.build_release(root, tmp_path / "release.zip", environ={"CUSTOM_PROVIDER_CREDENTIAL": secret},
                              secret_env=("CUSTOM_PROVIDER_CREDENTIAL",))
    assert not (tmp_path / "release.zip").exists()


@pytest.mark.spec("S0.4")
def test_common_credential_file_is_excluded_even_if_its_value_is_known(tmp_path):
    root = source_tree(tmp_path)
    secret = "FiXtUrE7"
    write(root, "config/mission-code.txt", secret.encode())
    write(root, ".env", ("MAICON_CODE=" + secret).encode())
    output = tmp_path / "release.zip"
    release.build_release(root, output, environ={"MAICON_CODE": secret})
    with zipfile.ZipFile(output) as archive:
        assert "config/mission-code.txt" not in archive.namelist()
        assert all(secret.casefold() not in archive.read(name).decode("utf-8", errors="ignore").casefold()
                   for name in archive.namelist())


@pytest.mark.spec("S0.4")
@pytest.mark.parametrize("directory", [False, True], ids=["file", "directory"])
def test_symlinks_outside_root_are_rejected_without_reading_external_content(tmp_path, directory):
    root = source_tree(tmp_path)
    external = tmp_path / "external"
    if directory:
        external.mkdir()
        write(external, "sensitive.txt", b"outside fixture")
    else:
        external.write_bytes(b"outside fixture")
    link = root / "linked-input"
    try:
        link.symlink_to(external, target_is_directory=directory)
    except OSError:
        pytest.skip("Creating symlinks is unavailable in this filesystem")
    with pytest.raises(release.ReleaseError, match="link|junction"):
        release.build_release(root, tmp_path / "release.zip", environ={})
    assert not (tmp_path / "release.zip").exists()


@pytest.mark.spec("S0.4")
def test_output_symlink_does_not_overwrite_its_target(tmp_path):
    root = source_tree(tmp_path)
    target = tmp_path / "existing.txt"
    target.write_bytes(b"keep me")
    output = tmp_path / "release.zip"
    try:
        output.symlink_to(target)
    except OSError:
        pytest.skip("Creating symlinks is unavailable in this filesystem")
    with pytest.raises(release.ReleaseError):
        release.build_release(root, output, overwrite=True, environ={})
    assert target.read_bytes() == b"keep me"


def test_existing_archive_requires_explicit_overwrite_and_failure_preserves_old_release(tmp_path):
    root = source_tree(tmp_path)
    output = tmp_path / "release.zip"
    release.build_release(root, output, environ={})
    previous = output.read_bytes()
    write(root, "new.py", b"new fixture")
    with pytest.raises(release.ReleaseError, match="explicit overwrite"):
        release.build_release(root, output, environ={})
    assert output.read_bytes() == previous
    secret = "FiXtUrE3"
    leak = write(root, "accidental.txt", secret.lower().encode())
    with pytest.raises(release.ReleaseError):
        release.build_release(root, output, overwrite=True, environ={"MAICON_CODE": secret})
    assert output.read_bytes() == previous
    leak.unlink()
    release.build_release(root, output, overwrite=True, environ={})
    with zipfile.ZipFile(output) as archive:
        assert archive.read("new.py") == b"new fixture"


def test_output_inside_source_is_not_recursively_bundled(tmp_path):
    root = source_tree(tmp_path)
    output = root / "release.zip"
    release.build_release(root, output, environ={})
    release.build_release(root, output, overwrite=True, environ={})
    with zipfile.ZipFile(output) as archive:
        assert "release.zip" not in archive.namelist()
        assert not any(".maicon-release-" in name for name in archive.namelist())


def test_no_clobber_publication_preserves_a_concurrently_created_destination(tmp_path, monkeypatch):
    root = source_tree(tmp_path)
    output = tmp_path / "release.zip"
    original_link = os.link

    def competing_link(src, dst):
        Path(dst).write_bytes(b"concurrent publisher")
        original_link(src, dst)

    monkeypatch.setattr(release.os, "link", competing_link)
    with pytest.raises(release.ReleaseError):
        release.build_release(root, output, environ={})
    assert output.read_bytes() == b"concurrent publisher"
    assert not list(tmp_path.glob(".maicon-release-*"))


@pytest.mark.spec("S0.4")
@pytest.mark.parametrize("excluded", ["../outside", "/outside", "C:/outside", "", "."])
def test_explicit_exclusion_cannot_escape_root(tmp_path, excluded):
    root = source_tree(tmp_path)
    with pytest.raises(release.ReleaseError):
        release.build_release(root, tmp_path / "release.zip", exclude=(excluded,), environ={})


@pytest.mark.spec("S0.13")
def test_required_original_specification_cannot_be_omitted(tmp_path):
    root = source_tree(tmp_path)
    with pytest.raises(release.ReleaseError, match="Required"):
        release.build_release(root, tmp_path / "release.zip", exclude=("docs/source",), environ={})


@pytest.mark.spec("S0.4")
def test_cli_never_echoes_unknown_secret_bearing_arguments(capsys):
    secret = "FiXtUrE2"
    assert release.main(["--mission-code", secret]) == 1
    captured = capsys.readouterr()
    assert secret.casefold() not in (captured.out + captured.err).casefold()
    assert "Invalid packaging arguments" in captured.err


def test_source_change_during_read_aborts_before_publication(tmp_path, monkeypatch):
    root = source_tree(tmp_path)
    source = root / "README.md"
    inode = source.stat().st_ino
    original = os.fdopen

    class ChangingStream:
        def __init__(self, stream):
            self.stream = stream

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return self.stream.__exit__(*args)

        def fileno(self):
            return self.stream.fileno()

        def read(self):
            data = self.stream.read()
            if os.fstat(self.stream.fileno()).st_ino == inode:
                # A real concurrent edit, not an injected validation exception.
                source.write_bytes(data + b"changed after the read")
            return data

    monkeypatch.setattr(release.os, "fdopen", lambda *args, **kwargs: ChangingStream(original(*args, **kwargs)))
    with pytest.raises(release.ReleaseError, match="changed"):
        release.build_release(root, tmp_path / "release.zip", environ={})
    assert not (tmp_path / "release.zip").exists()
    assert not list(tmp_path.glob(".maicon-release-*"))
