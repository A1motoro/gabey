"""A runnable export must preserve the perception/truth boundary and need no server."""

import csv
import json

from experiments.exp08_lidar_perception import main


def test_quick_lidar_export_is_self_contained_and_records_finite_causal_frames(tmp_path):
    payload = main(["--quick", "--systems", "cv", "--output-dir", str(tmp_path)])
    html = (tmp_path / "lidar_demo.html").read_text(encoding="utf-8")
    assert "__LIDAR_DATA__" not in html
    assert "fetch(" not in html
    assert json.loads((tmp_path / "scans.json").read_text(encoding="utf-8")) == payload
    with (tmp_path / "summary.csv").open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 8
    assert (tmp_path / "overview.png").stat().st_size > 1000
    for run in payload["runs"].values():
        frames = run["systems"]["cv"]["frames"]
        assert frames[0]["t"] == 0
        assert not frames[0]["tracks"]  # velocity is unobservable from one detection
        assert frames[-1]["t"] == run["systems"]["cv"]["summary"]["duration_s"]
        for frame in frames:
            truth_ids = {body["id"] for body in frame["truth"]}
            assert all(track["id"] not in truth_ids for track in frame["tracks"])
