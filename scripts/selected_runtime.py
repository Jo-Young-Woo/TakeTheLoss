"""Select the verified candidate without discarding development source/tests."""
from pathlib import Path
import hashlib
import json
import sys


def selection(root):
    root=Path(root).resolve()
    profile_path=root/'config/active_candidate.json'
    if not profile_path.is_file():return root,None
    profile=json.loads(profile_path.read_text(encoding='utf-8'))
    source=(root/profile['source_root']).resolve()
    manifest=(root/profile['source_manifest']).resolve()
    if not source.is_relative_to(root) or not manifest.is_relative_to(root):
        raise RuntimeError('Selected runtime must stay inside this project')
    return source,profile


def verify(root):
    root=Path(root).resolve()
    source,profile=selection(root)
    if profile is None:return source,None
    expected=json.loads((root/profile['source_manifest']).read_text(encoding='utf-8'))
    actual={str(path.relative_to(source)).replace('\\','/'):hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted((source/'maicon26').rglob('*.py'))}
    if actual!=expected:
        changed=sorted(key for key in actual.keys()|expected.keys() if actual.get(key)!=expected.get(key))
        raise RuntimeError('Selected candidate source differs from the measured version: '+', '.join(changed[:12]))
    # A Git checkout cannot carry the local Windows junctions used by the
    # frozen package. Recreate them from the shared, versioned project assets.
    for name in ('config','field','ros2','team'):
        target=root/name
        link=source/name
        if not target.is_dir():
            raise RuntimeError('Missing shared selected runtime directory: '+name)
        if link.is_symlink() or link.exists():
            if link.resolve()!=target.resolve():
                raise RuntimeError('Selected runtime asset link points elsewhere: '+name)
        else:
            try:
                link.symlink_to(Path('..')/'..'/name,target_is_directory=True)
            except OSError as exc:
                raise RuntimeError('Cannot create selected runtime asset link: '+name) from exc
    for asset in ('config/wingbot_sim_camera.json','ros2/maicon_arena_gz/media/textures/faces',
                  'ros2/wingbot_description/config'):
        if not (source/asset).exists():raise RuntimeError('Missing selected runtime asset: '+asset)
    return source,profile


def activate(root):
    source,profile=verify(root)
    existing=sys.modules.get('maicon26')
    if existing is not None and Path(existing.__file__).resolve().parent!=(source/'maicon26').resolve():
        raise RuntimeError('A different Agent package was already imported; restart with the selected launcher')
    sys.path.insert(0,str(source))
    return profile


if __name__=='__main__':
    root=Path(__file__).resolve().parents[1]
    profile=activate(root)
    import maicon26
    from maicon26.config import Config
    from maicon26.mission.manager import Agent
    from maicon26.sim.image_detector import make_wingbot_detector
    print(json.dumps(dict(candidate=profile['candidate'] if profile else None,
        package=str(Path(maicon26.__file__).resolve().parent),verified=True,
        activation='Gazebo runner uses --variant progress; no simulation was started'),ensure_ascii=False,indent=2))
