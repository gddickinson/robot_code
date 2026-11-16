"""The laser scan recorded in plotLaserScan.py (360 XV-11 lidar ranges), read from the script's own literals with ast
(the script itself plots at import, so it is not imported), and converted to x, y exactly as the script does with numpy.
"""
import ast
from pathlib import Path

import numpy as np

SCRIPT = Path(__file__).resolve().parent.parent / "plotLaserScan.py"


def literals():
    tree = ast.parse(SCRIPT.read_text())
    out = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name) \
                and node.targets[0].id in ("ranges", "angle_increment"):
            out[node.targets[0].id] = ast.literal_eval(node.value)
    return out["ranges"], out["angle_increment"]


def scan():
    ranges, inc = literals()
    radians = np.multiply(inc, range(360))          # as in plotLaserScan.py: degrees = range(360)
    xs = np.multiply(ranges, np.cos(radians))
    ys = np.multiply(ranges, np.sin(radians))
    return {"ranges": ranges, "angle_increment": inc, "x": xs.tolist(), "y": ys.tolist(), "n": len(ranges)}
