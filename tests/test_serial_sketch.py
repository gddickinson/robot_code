"""Run the real arduino/Arduino_serial_control_WORKS sketch (compiled on the desktop with tests/arduino_harness, the
same stand-in headers as in the Arduino repo) and check its command reader. Regression test for the reader storing -1
for bytes that had not yet arrived.
"""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import serial_harness as sh  # noqa: E402

pytestmark = pytest.mark.skipif(not sh.compiler(), reason="no C++ compiler")


@pytest.fixture(scope="module")
def run_case():
    fn, cleanup = sh.runner()
    yield fn
    cleanup()


def test_twist_serial_strings_are_parsed(run_case):
    # the strings robot_kinematics.serial_commands / twist_serial.py send for linear.x = -0.25, angular.z = 0
    ev = run_case("makeblock", "fixed", "$L25.000000Z$R25.000000Z")
    assert "M1.run(25)" in ev and "M2.run(25)" in ev


def test_command_arriving_byte_by_byte(run_case):
    ev = run_case("makeblock", "fixed", "$L200Z", gap=2)
    assert "M1.run(200)" in ev


def test_every_copy_of_the_reader_waits_for_bytes():
    for p in ROOT.rglob("*.ino"):
        text = p.read_text(errors="replace")
        assert text.count(sh.WAIT_MARK) >= text.count("while(current_char != command_end)"), p
