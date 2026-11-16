"""Write tests/fixtures/kinematics_cases.json: outputs of robot_kinematics.py (the module the ROS nodes import) for a
grid of twists and magnetometer readings, used by web/static/kinematics_engine.test.mjs to check the JS port.

Run: python scripts/make_kinematics_fixture.py
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import robot_kinematics as rk  # noqa: E402
from web.laserscan import scan  # noqa: E402

OUT = ROOT / "tests" / "fixtures" / "kinematics_cases.json"
XS = [-1.0, -0.5, -0.25, -0.1, 0.0, 0.05, 0.1, 0.25, 0.333, 0.5, 1.0]
ZS = [-3.0, -1.5, -0.4, 0.0, 0.2, 0.7, 1.0, 2.5]


def make():
    twists = [{"x": x, "z": z, "wheels_serial": rk.wheel_speeds(-x, z, rk.SERIAL_AXLE),
               "serial": list(rk.serial_commands(x, z)), "rrb3": list(rk.rrb3_command(x, z))} for x in XS for z in ZS]
    ties = [{"v": v, "f": "%f" % v} for v in (0.0078125, -0.0078125, 2.5e-7, 0.0000015, 12.3456785, -0.0)]
    words = [{"high": h, "low": lo, "value": rk.twos_complement(h, lo)} for h, lo in
             [(0, 0), (0x7F, 0xFF), (0x80, 0), (0xFF, 0xFF), (0x12, 0x34), (0xF0, 0x0F)]]
    compass = [{"x": x, "y": y, "bearing": rk.compass_bearing(x, y)} for x, y in
               [(100, 0), (0, 100), (-100, 0), (0, -100), (120, 40), (-300, 15), (7, -260), (-1, -1), (55, 55)]]
    return {"twists": twists, "ties": ties, "words": words, "compass": compass, "scan": scan()}


if __name__ == "__main__":
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(make(), indent=0) + "\n")
    print("wrote", OUT.relative_to(ROOT))
