from types import SimpleNamespace
import pytest
from scripts import run_gazebo_match as trial


def test_topic_delivery_gap_halts_then_resumes_with_full_elapsed_time(monkeypatch):
    state = {"wall": 0., "clock": 5.1, "ready": False, "stops": 0}
    def stop():
        state["stops"] += 1
    def sleep(dt):
        assert state["stops"] > 0  # No driving during data recovery.
        state["wall"] += dt
        state["clock"] += dt
        state["ready"] = state["wall"] >= .08
    monkeypatch.setattr(trial.time, "monotonic", lambda: state["wall"])
    monkeypatch.setattr(trial.time, "sleep", sleep)
    robot = SimpleNamespace(now=lambda: state["clock"], ready=lambda: state["ready"], stop=stop)
    current, elapsed = trial.ready_step({"B2": robot}, 5., recover_s=.2)
    assert elapsed == pytest.approx(current-5.) and elapsed >= .18-1e-9
    assert state["stops"] == 1 and robot.sensor_recoveries == 1


def test_persistent_invalid_data_stays_stopped_and_fails(monkeypatch):
    state = {"wall": 0., "stops": 0}
    monkeypatch.setattr(trial.time, "monotonic", lambda: state["wall"])
    monkeypatch.setattr(trial.time, "sleep", lambda dt: state.update(wall=state["wall"]+dt))
    robot = SimpleNamespace(now=lambda: 5.1, ready=lambda: False,
                            stop=lambda: state.update(stops=state["stops"]+1))
    with pytest.raises(RuntimeError, match="stale"):
        trial.ready_step({"B2": robot}, 5., recover_s=.1)
    assert state["stops"] == 1
    assert not hasattr(robot, "sensor_recoveries")
