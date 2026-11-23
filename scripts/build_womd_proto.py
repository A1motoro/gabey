"""Build a pinned official WOMD descriptor without TensorFlow (one-time maintainer tool).

Run: pip install grpcio-tools; python scripts/build_womd_proto.py
The resulting descriptor is packaged with pbs; ordinary readers only need protobuf.
"""

import re
import urllib.request
from pathlib import Path

from grpc_tools import protoc

ROOT = Path(__file__).resolve().parents[1]
REVISION = "508fb261f3b7dee9ebc969d05ff1311b5ea69dc1"
BASE = f"https://raw.githubusercontent.com/waymo-research/waymo-open-dataset/{REVISION}/src/"


def main():
    vendor = ROOT / "vendor"
    pending = ["waymo_open_dataset/protos/scenario.proto"]
    seen = set()
    while pending:
        name = pending.pop()
        if name in seen:
            continue
        seen.add(name)
        path = vendor / name
        path.parent.mkdir(parents=True, exist_ok=True)
        source = urllib.request.urlopen(BASE + name, timeout=60).read()
        path.write_bytes(source)
        pending.extend(re.findall(r'import "([^"]+)";', source.decode("utf-8")))
        print(name, len(source), flush=True)
    output = ROOT / "src/pbs/data/womd.desc"
    result = protoc.main(
        [
            "protoc",
            f"-I{vendor}",
            f"--descriptor_set_out={output}",
            "--include_imports",
            str(vendor / "waymo_open_dataset/protos/scenario.proto"),
        ]
    )
    if result:
        raise RuntimeError(f"protoc failed: {result}")
    license_data = urllib.request.urlopen(
        BASE.rsplit("/src/", 1)[0] + "/LICENSE", timeout=60
    ).read()
    (vendor / "LICENSE").write_bytes(license_data)
    (vendor / "README.md").write_text(
        f"# Official Waymo protobuf sources\n\nUpstream: {BASE}\n\n"
        f"Pinned commit: `{REVISION}` (Apache-2.0; see LICENSE).\n"
        "Regenerate src/pbs/data/womd.desc with scripts/build_womd_proto.py.\n"
        "New optional sensor fields are ignored by the track-level adapter.\n",
        encoding="utf-8",
    )
    print(f"Built {output} ({output.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
