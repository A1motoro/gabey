"""Run from any directory: python /path/to/gabey/examples/cut_in_demo.py."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pbs.demo.server import main  # noqa: E402

if __name__ == "__main__":
    main()
