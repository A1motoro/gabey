"""WOMD regression tests: frozen REAL clips; no TensorFlow/protobuf/raw shards needed."""

import hashlib
import importlib.util
import json
import os
import struct
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from pbs.data import load_womd_tracks
from pbs.data.womd import (
    find_candidates,
    iter_tfrecords,
    load_clip,
    load_evidence,
    local_xy,
    real_scenario,
    save_clip,
    sdc_index,
    track_arrays,
)
from pbs.eval.counterfactual import is_benign
from pbs.risk.assessors import NullAssessor
from pbs.sim.policy import NeverBrakePolicy
from pbs.sim.simulator import run_sim

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests/fixtures"
MANIFEST = json.loads((FIXTURES / "womd_manifest.json").read_text(encoding="utf-8"))
ITEMS = MANIFEST["selected"]
CLIPS = [FIXTURES / item["filename"] for item in ITEMS]


@pytest.mark.parametrize("path", CLIPS, ids=lambda p: p.stem)
def test_real_fixture_contract_and_provenance(path):
    scenario = load_clip(path)
    metadata = next(item for item in ITEMS if item["filename"] == path.name)
    assert path.stat().st_size < 200_000
    assert hashlib.sha256(path.read_bytes()).hexdigest() == metadata["sha256"]
    assert scenario.ego_start[:3] == (0, 0, 0)
    np.testing.assert_allclose(scenario.ego_path[0], (0, 0))
    assert scenario.params["source"] == "Waymo-WOMD-v1.2-validation"
    assert scenario.kind == f"real_{scenario.params['group']}"
    assert len(scenario.agents) == 1  # explicitly selected-target scope
    assert scenario.params["agent_scope"] == "selected_target_only"
    assert scenario.agents[0].agent_id != f"track{scenario.params['sdc_track_id']}"
    assert len({a.agent_id for a in scenario.agents}) == len(scenario.agents)
    assert scenario.params["ego_max_lat_m"] < 1
    assert scenario.params["ego_max_yaw_rate_deg_s"] < 5
    for agent in scenario.agents:
        assert len(agent.t) == 81
        assert agent.t[0] == pytest.approx(-2)
        assert agent.t[-1] == pytest.approx(6)
        np.testing.assert_allclose(np.diff(agent.t), 0.1, atol=1e-12)
        assert np.count_nonzero(agent.t <= 0) == 21
        assert agent.accel is None
        assert all(np.isfinite(getattr(agent, k)).all() for k in ("x", "y", "v", "heading"))
    evidence = load_evidence(path)
    assert evidence["ego_valid"].all() and evidence["agent_valid"].all()
    assert evidence["observed_ego_xy"].shape == (81, 2)
    np.testing.assert_allclose(evidence["observed_ego_xy"][20], (0, 0), atol=1e-9)
    error = np.max(np.abs(evidence["observed_timestamps_s"] - scenario.agents[0].t))
    assert error == pytest.approx(scenario.params["timestamp_max_grid_error_s"])
    assert error < 0.006
    assert load_womd_tracks(path)[0].agent_id == scenario.agents[0].agent_id


@pytest.mark.parametrize("path", CLIPS, ids=lambda p: p.stem)
def test_real_fixtures_pass_simulator_and_control_counterfactual(path):
    scenario = load_clip(path)
    log = run_sim(scenario, NullAssessor(), NeverBrakePolicy(), duration=6)
    assert len(log.t) == 61 and log.executed_steps == 60
    assert all(action == "cruise" for action in log.action)
    if scenario.params["group"] == "control":
        assert is_benign(scenario, duration=6)
    # Geometric cut-in does NOT imply a collision; never assert fabricated danger.


def test_cohorts_are_distinct_and_selected_without_system_outcomes():
    assert MANIFEST["records_scanned"] == 2321
    assert MANIFEST["crc32c_verified"]
    assert MANIFEST["selected_by_group"] == {"cut_in": 5, "control": 5}
    assert len({item["clip"] for item in ITEMS}) == len(ITEMS)
    assert "no system outcomes" in MANIFEST["selection"]
    for item in ITEMS:
        if item["group"] == "cut_in":
            assert 5 <= item["lon_m"] <= 40
            assert 0 < item["t_cross_s"] <= 6
        else:
            assert item["t_cross_s"] is None


def test_npz_roundtrip_without_pickle(tmp_path):
    source = CLIPS[0]
    scenario = load_clip(source)
    output = tmp_path / "roundtrip.npz"
    save_clip(scenario, output, evidence=load_evidence(source))
    loaded = load_clip(output)
    assert loaded.params == scenario.params and loaded.ego_start == scenario.ego_start
    np.testing.assert_array_equal(loaded.ego_path, scenario.ego_path)
    for before, after in zip(scenario.agents, loaded.agents, strict=True):
        for key in ("t", "x", "y", "v", "heading"):
            np.testing.assert_array_equal(getattr(before, key), getattr(after, key))
    with np.load(output, allow_pickle=False) as arrays:
        assert all(not arrays[key].dtype.hasobject for key in arrays.files)


def fake_scene(*, yaw=0.6, offset=(35.0, -17.0), control=False):
    """Rotated/translated mathematical input; tests adapter invariance, not evidence."""
    t = np.arange(91) * 0.1
    ego = np.column_stack((15 * t, np.zeros(91)))
    y = np.full(91, 3.5) if control else 3.5 * (1 - np.clip((t - 3) / 3, 0, 1))
    target = np.column_stack((25 + 12 * t, y))
    rotation = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])

    def track(points, track_id, speed):
        world = points @ rotation + np.array(offset)
        return SimpleNamespace(
            id=track_id,
            object_type=1,
            states=[
                SimpleNamespace(
                    center_x=x,
                    center_y=y,
                    heading=yaw,
                    velocity_x=speed * np.cos(yaw),
                    velocity_y=speed * np.sin(yaw),
                    valid=True,
                )
                for x, y in world
            ],
        )

    scene = SimpleNamespace(
        scenario_id="test-only",
        timestamps_seconds=t,
        tracks=[track(target, 42, 12), track(ego, 99, 15)],
        sdc_track_index=1,
    )
    scene.HasField = lambda field: field == "sdc_track_index"
    return scene


def test_sdc_uses_metadata_and_fails_when_missing_or_out_of_range():
    scene = fake_scene()
    assert sdc_index(scene) == 1
    scene.sdc_track_index = 9
    with pytest.raises(ValueError, match="out of range"):
        sdc_index(scene)
    scene.HasField = lambda field: False
    with pytest.raises(ValueError, match="refusing to guess"):
        sdc_index(scene)


def test_fixed_transform_rotation_invariance_and_invalid_rows():
    scene = fake_scene()
    scenario = real_scenario(scene, 0, anchor=20)
    plain = real_scenario(fake_scene(yaw=0, offset=(0.0, 0.0)), 0, anchor=20)
    for key in ("x", "y", "v", "heading"):
        np.testing.assert_allclose(
            getattr(scenario.agents[0], key), getattr(plain.agents[0], key), atol=1e-12
        )
    np.testing.assert_allclose(local_xy(np.array([[2.0, 3.0]]), (2.0, 3.0), 0.6), 0)
    scene.tracks[0].states[12].valid = False
    xy, heading, velocity, valid = track_arrays(scene.tracks[0])
    assert not valid[12]
    assert np.isnan(xy[12]).all() and np.isnan(heading[12]) and np.isnan(velocity[12]).all()
    with pytest.raises(ValueError, match="continuously valid"):
        real_scenario(scene, 0, anchor=20)


def test_candidate_filter_discriminates_cut_in_control_and_ego_turns():
    cut_in = find_candidates(fake_scene())
    assert any(r["group"] == "cut_in" and r["track_id"] == 42 for r in cut_in)
    controls = find_candidates(fake_scene(control=True))
    assert all(r["group"] == "control" for r in controls)
    scene = fake_scene()
    for k, state in enumerate(scene.tracks[1].states):
        state.heading += 0.02 * k
    assert find_candidates(scene) == []


def test_invalid_history_or_future_is_not_filled():
    for frame in (0, 20, 80):
        scene = fake_scene()
        scene.tracks[0].states[frame].valid = False
        with pytest.raises(ValueError, match="continuously valid"):
            real_scenario(scene, 0, anchor=20)
    with pytest.raises(ValueError, match="Insufficient"):
        real_scenario(fake_scene(), 0, anchor=10)
    with pytest.raises(ValueError, match="Insufficient"):
        real_scenario(fake_scene(), 0, anchor=31)
    with pytest.raises(ValueError, match="non-ego"):
        real_scenario(fake_scene(), 1)


def test_timestamp_jitter_is_bounded_but_gaps_are_rejected():
    scene = fake_scene()
    scene.timestamps_seconds = (
        np.asarray(scene.timestamps_seconds) + np.sin(np.arange(91)) * 0.00005
    )
    assert real_scenario(scene, 0).params["timestamp_max_grid_error_s"] < 0.0001
    scene.timestamps_seconds[45] += 0.01
    with pytest.raises(ValueError, match="10 Hz"):
        real_scenario(scene, 0)


@pytest.mark.parametrize(
    "mutation,error",
    [
        ("gap", "time gap"),
        ("nan", "finite"),
        ("ego", "Ego cannot"),
        ("invalid", "invalid states"),
        ("short_future", "full future"),
    ],
)
def test_bad_snapshots_fail_loudly(tmp_path, mutation, error):
    with np.load(CLIPS[0], allow_pickle=False) as z:
        arrays = {k: z[k].copy() for k in z.files}
    if mutation == "gap":
        arrays["t"][0, 12] += 0.05
    elif mutation == "nan":
        arrays["x"][0, 12] = np.nan
    elif mutation == "ego":
        params = json.loads(str(arrays["params_json"]))
        arrays["agent_id"] = np.array([f"track{params['sdc_track_id']}"])
    elif mutation == "invalid":
        arrays["ego_valid"][12] = False
    else:
        for k in ("t", "x", "y", "v", "heading"):
            arrays[k] = arrays[k][:, :-1]
    output = tmp_path / "broken.npz"
    np.savez_compressed(output, **arrays)
    with pytest.raises(ValueError, match=error):
        load_clip(output)


def test_tfrecord_truncation_is_not_silent(tmp_path):
    path = tmp_path / "record.bin"
    path.write_bytes(b"abc")
    with pytest.raises(ValueError, match="header"):
        list(iter_tfrecords(path, verify_crc=False))
    path.write_bytes(struct.pack("<QI", 3, 0) + b"ab")
    with pytest.raises(ValueError, match="payload"):
        list(iter_tfrecords(path, verify_crc=False))
    path.write_bytes(struct.pack("<QI", 3, 0) + b"abc" + struct.pack("<I", 0))
    assert list(iter_tfrecords(path, verify_crc=False)) == [b"abc"]


def test_npz_loading_with_google_packages_and_raw_access_blocked(tmp_path):
    code = f"""
import sys, importlib.abc
from pathlib import Path
class BlockGoogle(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'google' or fullname.startswith('google.'):
            raise ImportError('protobuf deliberately unavailable')
sys.meta_path.insert(0, BlockGoogle())
from pbs.data.womd import load_clip
original = Path.open
def no_raw(self, *args, **kwargs):
    if 'waymo' in self.parts or '.tfrecord' in self.name:
        raise RuntimeError('raw data deliberately unavailable')
    return original(self, *args, **kwargs)
Path.open = no_raw
assert load_clip({str(CLIPS[0])!r}).agents
print('npz-only-ok')
"""
    result = subprocess.run(
        [sys.executable, "-c", code],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        env=dict(os.environ, PYTHONPATH=str(ROOT / "src")),
        timeout=20,
    )
    assert result.returncode == 0, result.stderr
    assert "npz-only-ok" in result.stdout


def test_forecast_windows_use_exactly_20_frames_without_future_leakage():
    spec = importlib.util.spec_from_file_location(
        "real_experiment", ROOT / "experiments/exp07_real_clip.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    scenario = load_clip(CLIPS[0])
    windows = list(module.forecast_windows(scenario, 30))
    assert len(windows) == 7
    assert windows[0][0].t[-1] == pytest.approx(0)
    assert windows[-1][0].t[-1] == pytest.approx(3)
    for history, truth in windows:
        assert len(history.t) == 20
        assert history.t[-1] - history.t[0] == pytest.approx(1.9)
        assert truth.shape == (30, 2)
        next_index = np.flatnonzero(scenario.agents[0].t > history.t[-1] + 1e-9)[0]
        np.testing.assert_array_equal(
            truth[0], [scenario.agents[0].x[next_index], scenario.agents[0].y[next_index]]
        )


@pytest.mark.parametrize(
    "item",
    json.loads((FIXTURES / "womd_cases_manifest.json").read_text(encoding="utf-8"))["selected"],
    ids=lambda item: item["case_role"],
)
def test_illustrative_cases_preserve_real_danger_and_false_brake(item):
    from pbs.eval.metrics import evaluate
    from pbs.sim.policy import HysteresisPolicy
    from pbs.systems import make_system

    path = FIXTURES / item["filename"]
    assert hashlib.sha256(path.read_bytes()).hexdigest() == item["sha256"]
    scenario = load_clip(path)
    assert scenario.params["cohort"] == "illustrative"
    assert "excluded from primary" in scenario.params["selection"]
    assert len(scenario.agents[0].t) == 81
    benign = is_benign(scenario, duration=6)
    outcomes = {}
    for system in ("reactive", "cv"):
        outcomes[system] = evaluate(
            run_sim(scenario, make_system(system), HysteresisPolicy(), duration=6), benign
        )
        assert not outcomes[system]["collision"]
    if item["case_role"] == "dangerous":
        assert not benign
        assert outcomes["reactive"]["brake_onset_t"] == pytest.approx(3.2)
        assert outcomes["cv"]["brake_onset_t"] == pytest.approx(2.9)
    else:
        assert benign
        assert all(outcome["false_brake"] for outcome in outcomes.values())
        assert outcomes["cv"]["brake_onset_t"] == pytest.approx(4.8)


def test_full_candidate_audit_is_disclosed_without_relabeling_primary_cohort():
    audit = json.loads((FIXTURES / "womd_cut_in_audit.json").read_text(encoding="utf-8"))
    assert audit["candidate_count"] == 40 == len(audit["results"])
    assert audit["unique_scenarios"] == 39
    assert audit["benign_count"] == 39 and audit["dangerous_count"] == 1
    assert audit["systems"]["cv"]["false_brakes"] == 1
    assert audit["systems"]["reactive"]["false_brakes"] == 1
