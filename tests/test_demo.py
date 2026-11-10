"""Demo output must be faithful to the Python simulation, not a scripted animation."""

import json
import subprocess
import sys
import threading
from dataclasses import replace
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np
import pytest

from pbs.demo.scene import DemoScene, SyntheticCutInAdapter, resolve_config
from pbs.demo.server import DemoHandler, ThreadingHTTPServer
from pbs.demo.service import simulate
from pbs.sim.policy import HysteresisPolicy
from pbs.sim.simulator import run_sim
from pbs.systems import make_system


@pytest.mark.parametrize("rel_v", [5, -2])
def test_demo_matches_simulation_and_is_strict_json(rel_v):
    result = simulate({"rel_v": rel_v})
    json.loads(json.dumps(result, allow_nan=False))
    scenario = SyntheticCutInAdapter().load({"rel_v": rel_v}).scenario
    for name in ("reactive", "cv"):
        direct = run_sim(scenario, make_system(name), HysteresisPolicy())
        shown = result["systems"][name]
        np.testing.assert_array_equal(shown["x"], direct.ego_x)
        np.testing.assert_array_equal(shown["v"], direct.ego_v)
        np.testing.assert_array_equal(shown["a"], direct.ego_a)
        assert shown["action"] == direct.action
        assert shown["summary"]["collision"] == direct.collision
        assert shown["summary"]["min_dist"] == direct.min_dist
        for frame in shown["trace"]:
            assert max(frame["observed"]["t"]) <= frame["t"] + 1e-9
        assert shown["ttc"] == [float(t) if np.isfinite(t) else None for t in direct.ttc]
    if rel_v == 5:
        assert result["systems"]["reactive"]["summary"]["brake_onset_t"] == pytest.approx(2)
        assert result["systems"]["cv"]["summary"]["brake_onset_t"] == pytest.approx(1.7)
        assert result["lead_time_s"] == pytest.approx(0.3)
        assert result["systems"]["reactive"]["summary"]["brake_count"] == 3
        assert result["systems"]["cv"]["summary"]["brake_count"] == 5
        for shown in result["systems"].values():
            assert shown["summary"]["brake_duration_s"] == pytest.approx(0.8)
            assert shown["braking"]["count"][-1] == shown["summary"]["brake_count"]
            assert shown["braking"]["duration_s"][-1] == pytest.approx(0.8)
    else:
        assert result["lead_time_s"] is None
        assert all(s["summary"]["brake_count"] == 0 for s in result["systems"].values())
        assert all(s["summary"]["brake_onset_t"] is None for s in result["systems"].values())


def test_recorded_forecast_is_actual_cv_output():
    result = simulate({})
    frame = result["systems"]["cv"]["trace"][17]
    o = frame["observed"]
    xy = np.column_stack([o["x"], o["y"]])
    velocity = (xy[-1] - xy[-2]) / (o["t"][-1] - o["t"][-2])
    expected = xy[-1] + np.arange(1, 31)[:, None] * 0.1 * velocity
    np.testing.assert_allclose(frame["predicted"], expected)


@pytest.mark.parametrize(
    "params",
    [{"gap_d": 0}, {"rel_v": float("nan")}, {"gap_d": True}, {"lc_T": "3"}, {"delay": 1}, [], None],
)
def test_bad_parameters(params):
    with pytest.raises(ValueError):
        resolve_config(params)


class Identity:
    def observe(self, history, *, t_now):
        return history


class ShiftObservation:
    def observe(self, history, *, t_now):
        # Huge lateral shift should alter decisions, never physical collision checks.
        history.y += 100
        return history


def test_observation_boundary_preserves_truth_and_identity_result():
    scene = SyntheticCutInAdapter().load({})
    original = scene.scenario.agents[0].y.copy()
    plain = run_sim(scene.scenario, make_system("reactive"), HysteresisPolicy())
    identity = run_sim(
        scene.scenario, make_system("reactive"), HysteresisPolicy(), observation_model=Identity()
    )
    np.testing.assert_array_equal(plain.ego_x, identity.ego_x)
    np.testing.assert_array_equal(plain.ttc, identity.ttc)
    shifted = run_sim(
        scene.scenario,
        make_system("reactive"),
        HysteresisPolicy(),
        observation_model=ShiftObservation(),
    )
    assert shifted.collision  # Truth still causes collision despite the distant measurement.
    assert set(shifted.action) == {"cruise"}
    np.testing.assert_array_equal(scene.scenario.agents[0].y, original)


def test_observation_must_preserve_timestamps():
    class FutureObservation:
        def observe(self, history, *, t_now):
            history.t += 0.1
            return history

    scene = SyntheticCutInAdapter().load({})
    with pytest.raises(ValueError, match="timestamps"):
        run_sim(
            scene.scenario,
            make_system("cv"),
            HysteresisPolicy(),
            observation_model=FutureObservation(),
        )


def test_adapter_context_and_observations_are_carried_without_mutation():
    base = SyntheticCutInAdapter().load({})
    reference = replace(base.scenario.agents[0], agent_id="recorded_ego_reference")

    class CustomAdapter:
        def load(self, config):
            return DemoScene(
                base.scenario, {"kind": "recorded", "reference_kind": "measured"}, None, reference
            )

    first = simulate({}, adapter=CustomAdapter(), observation_factory=Identity)
    second = simulate({}, adapter=CustomAdapter(), observation_factory=Identity)
    assert first == second
    assert first["road_lines"] is None
    assert first["source"]["reference_kind"] == "measured"
    assert first["ego_reference"]["agent_id"] == reference.agent_id
    assert first["systems"]["cv"]["trace"][0]["observed"]["x"] == [20]


def test_collision_case_is_not_relabelled_safe():
    result = simulate({"gap_d": 10, "rel_v": 4})
    assert any(s["summary"]["collision"] for s in result["systems"].values())


def test_http_assets_errors_and_results():
    try:
        server = ThreadingHTTPServer(("127.0.0.1", 0), DemoHandler)
    except PermissionError:
        pytest.skip("Local socket binding is disabled in this sandbox.")
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        for path in (
            "/",
            "/style.css",
            "/app.js",
            "/api/config",
            "/presentation",
            "/presentation/",
            "/presentation.css",
            "/presentation.js",
            "/presentation-events.js",
        ):
            with urlopen(base + path) as response:
                assert response.status == 200
                assert response.read()
        with urlopen(base + "/") as response:
            classic = response.read().decode()
            assert "/presentation.js" not in classic
            assert 'href="/presentation"' in classic
        with urlopen(base + "/presentation") as response:
            presentation = response.read().decode()
            assert "/presentation.js" in presentation
            assert 'name="playback-mode"' in presentation
            assert 'id="mode-events"' in presentation
            assert 'id="mode-full"' in presentation
            assert '<details id="advanced-parameters" open>' in presentation
            assert 'href="/"' in presentation
        with urlopen(Request(base + "/api/simulate", data=b"{}")) as response:
            result = json.load(response)
            assert result["lead_time_s"] == pytest.approx(0.3)
        for path, body, status in [
            ("/api/simulate", b'{"gap_d":0}', 400),
            ("/api/simulate", b'{"gap_d":NaN}', 400),
            ("/api/simulate", b"[]", 400),
            ("/api/simulate", b"broken", 400),
            ("/unknown", b"{}", 404),
        ]:
            with pytest.raises(HTTPError) as exc:
                urlopen(Request(base + path, data=body))
            assert exc.value.code == status
            assert "error" in json.load(exc.value)
        with pytest.raises(HTTPError) as exc:
            urlopen(
                Request(
                    base + "/api/simulate", data=b"{}", headers={"Origin": "https://example.com"}
                )
            )
        assert exc.value.code == 403
        with pytest.raises(HTTPError):
            urlopen(base + "/../pyproject.toml")
        root = Path(__file__).resolve().parents[1]
        completed = subprocess.run(
            [
                sys.executable,
                str(root / "examples/cut_in_demo.py"),
                "--no-browser",
                "--port",
                str(server.server_port),
            ],
            cwd="/tmp",
            capture_output=True,
            text=True,
            timeout=10,
        )
        assert completed.returncode == 1
        assert "Try --port" in completed.stderr
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


def test_demo_keeps_braking_inside_hysteresis_band():
    result = simulate({})
    assert result["fixed"]["policy"] == "hysteresis"
    assert result["fixed"]["t_on"] == 2.0
    assert result["fixed"]["t_off"] == 3.0
    for shown in result["systems"].values():
        state = "cruise"
        for ttc, action in zip(shown["ttc"][:100], shown["action"][:100], strict=True):
            if state == "cruise" and ttc is not None and ttc < 2:
                state = "brake"
            elif state == "brake" and (ttc is None or ttc > 3):
                state = "cruise"
            assert action == state
    reactive = result["systems"]["reactive"]
    assert 2 <= reactive["ttc"][24] <= 3
    assert reactive["action"][24] == "brake"
