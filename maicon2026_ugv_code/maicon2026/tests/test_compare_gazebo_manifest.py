import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import pytest

spec=importlib.util.spec_from_file_location('compare_gazebo',Path(__file__).resolve().parents[1]/'scripts/compare_gazebo.py')
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)


def args(root):
    return SimpleNamespace(output=root,variants=['v2'],scenarios=['selfplay'],seeds=[0])


def test_source_pin_rejects_revision_drift(tmp_path,monkeypatch):
    monkeypatch.setattr(mod,'source_fingerprint',lambda:{'controller.py':'one'})
    mod.assert_source_pin(args(tmp_path))
    mod.assert_source_pin(args(tmp_path))
    monkeypatch.setattr(mod,'source_fingerprint',lambda:{'controller.py':'two'})
    with pytest.raises(RuntimeError,match='Source changed'):
        mod.assert_source_pin(args(tmp_path))


def test_process_success_does_not_mean_acceptance(tmp_path):
    folder=tmp_path/'v2/selfplay/seed_00/attempt_000';folder.mkdir(parents=True)
    (folder/'selfplay_report.json').write_text(json.dumps({'completed':True,'errors':[]}))
    case=mod.inventory(args(tmp_path))[0]
    assert case['completed'] is True and case['accepted'] is None
    (folder/'selfplay_analysis.json').write_text(json.dumps({'acceptance':{'overall_passed':False}}))
    assert mod.inventory(args(tmp_path))[0]['accepted'] is False


def test_midtrial_source_drift_invalidates_comparison(tmp_path):
    folder=tmp_path/'v2/selfplay/seed_00/attempt_000';folder.mkdir(parents=True)
    (folder/'selfplay_report.json').write_text(json.dumps({'completed':True,'errors':[]}))
    (folder.parent/'attempt_000_source_drift.json').write_text(json.dumps({'valid_for_comparison':False}))
    case=mod.inventory(args(tmp_path))[0]
    assert case['completed'] is False and case['attempts'][0]['source_drift']
