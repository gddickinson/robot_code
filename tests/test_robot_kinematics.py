"""robot_kinematics.py gives the same numbers as the ROS callbacks it was factored out of (the callbacks are run with
stand-in ROS modules), plus hand-checked values."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import robot_kinematics as rk  # noqa: E402
from ros_stubs import Twist, Recorder, load  # noqa: E402

TWISTS = [(0.0, 0.0), (0.5, 0.0), (-0.2, 0.0), (0.0, 1.0), (0.0, -2.0), (0.3, 0.8), (1.0, -1.5), (-0.4, 0.4)]


@pytest.mark.parametrize("x,z", TWISTS)
def test_serial_commands_match_twist_serial_callback(x, z):
    calls = []
    node = load("twist_serial.py", calls)
    node.callback(Twist(x, z))
    writes = [c[1] for c in calls if c[0] == "ser.write"]
    assert tuple(writes) == rk.serial_commands(x, z)


@pytest.mark.parametrize("x,z", TWISTS)
def test_rrb3_command_matches_twist_piRobot_callback(x, z):
    calls = []
    node = load("twist_piRobot.py", calls)
    node.piRobot = Recorder("piRobot", calls)
    node.getWheelSpeed = lambda: (0.0, 0.0)
    node.callback(Twist(x, z))
    sets = [c[1:] for c in calls if c[0] == "piRobot.set_motors"]
    assert sets == [rk.rrb3_command(x, z)]


def test_hand_values():
    assert rk.wheel_speeds(0.5, 1.0, 0.36) == pytest.approx((0.68, 0.32))
    assert rk.serial_commands(0.25, 0.0) == ("$L-25.000000Z", "$R-25.000000Z")
    assert rk.rrb3_command(0.5, 0.0) == (1, 0, 1, 0)                 # 0.5*3 = 1.5, capped at 1
    assert rk.rrb3_command(-0.1, 0.0) == pytest.approx((0.3, 1, 0.3, 1))
    assert rk.twos_complement(0xFF, 0xFF) == -1 and rk.twos_complement(0x01, 0x00) == 256
    assert rk.compass_bearing(0, 100) == pytest.approx(90.0)
    assert rk.compass_bearing(0, -100) == pytest.approx(270.0)
    assert rk.compass_bearing(-5, 0) == pytest.approx(180.0)


def test_compass_matches_bearing_py_formula():
    for x, y in [(120, 40), (-300, 15), (7, -260), (-1, -1)]:
        b = math.atan2(y * 0.92, x * 0.92)
        assert rk.compass_bearing(x, y) == pytest.approx(math.degrees(b + 2 * math.pi if b < 0 else b))
