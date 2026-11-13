"""Regression tests for piRobot/twist_piRobotNode3.py dead reckoning (run with stand-in ROS modules).

The node sets DIRECTION from the motor directions it sends (0 = forward, 1 = reverse on the RRB3 board) and moves
X, Y by speed * time along BEARING, which /robot_bearing publishes in degrees (piRobot/bearingNode.py).
"""
import math
import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros_stubs import Anything, Recorder, Twist, load  # noqa: E402


@pytest.fixture
def node():
    calls = []
    n = load("piRobot/twist_piRobotNode3.py", calls)
    n.piRobot = Recorder("piRobot", calls)
    n.rospy = types.SimpleNamespace(loginfo=lambda *a: None, Time=types.SimpleNamespace(now=lambda: 0))
    return n


@pytest.mark.parametrize("x,z,expected", [(0.3, 0.0, 1), (-0.3, 0.0, -1), (0.0, 2.0, 0), (0.0, 0.0, 0)])
def test_direction_follows_the_motor_directions(node, x, z, expected):
    node.callback(Twist(x, z))
    assert node.DIRECTION == expected


def test_forward_motion_moves_along_the_bearing_in_degrees(node):
    node.callback(Twist(0.3, 0.0))                        # both wheels forward
    node.X = node.Y = 0.0
    node.LWHEEL = node.RWHEEL = 0.5                       # m/s from the encoders
    node.BEARING = 90.0                                   # degrees, as published on /robot_bearing
    node.TIME_LAST_COMMAND = 8.0
    node.time = types.SimpleNamespace(time=lambda: 10.0)  # 2 s since the command
    node.broadcastOdometryInfo(Recorder("tf", []), Recorder("pub", []), node.axle)
    assert node.X == pytest.approx(0.0, abs=1e-9) and node.Y == pytest.approx(1.0)


def test_reverse_motion_moves_backwards(node):
    node.callback(Twist(-0.3, 0.0))
    node.X = node.Y = 0.0
    node.LWHEEL = node.RWHEEL = 0.25
    node.BEARING = 0.0
    node.TIME_LAST_COMMAND = 9.0
    node.time = types.SimpleNamespace(time=lambda: 10.0)
    node.broadcastOdometryInfo(Recorder("tf", []), Recorder("pub", []), node.axle)
    assert node.X == pytest.approx(-0.25) and node.Y == pytest.approx(0.0, abs=1e-9)
