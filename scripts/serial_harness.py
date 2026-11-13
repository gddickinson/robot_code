"""Compile and run the serial-control sketches on the desktop with the stand-in Arduino headers in
tests/arduino_harness, so their real C++ can be tested and compared with the JS port (the Arduino repo has the JS port).

variant="fixed" compiles the sketch as it is in the repo; variant="original" first removes the one-line wait that was
added to the command reader (the line ending in WAIT_MARK), which reproduces the sketch as it was before the fix.
"""
import shutil
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
HARNESS = ROOT / "tests" / "arduino_harness"
WAIT_MARK = "// wait for the next byte (Serial.read() returns -1 until it arrives)"
SKETCHES = {
    "makeblock": "arduino/Arduino_serial_control_WORKS/Arduino_serial_control_WORKS.ino",

}


def compiler():
    return shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")


def build(sketch: str, variant: str, out_dir: Path) -> Path:
    src = (ROOT / SKETCHES[sketch]).read_text()
    if variant == "original":
        src = "".join(ln for ln in src.splitlines(keepends=True) if WAIT_MARK not in ln)
    ino = out_dir / f"{sketch}_{variant}.ino"
    ino.write_text(src)
    exe = out_dir / f"{sketch}_{variant}"
    subprocess.run([compiler(), "-std=c++11", "-w", "-I", str(HARNESS), "-include", str(HARNESS / "prototypes.h"),
                    f'-DSKETCH="{ino}"', str(HARNESS / "harness.cpp"), "-o", str(exe)], check=True)
    return exe


def run(exe: Path, stream: str, gap: int = 0) -> list:
    r = subprocess.run([str(exe), str(gap), stream], capture_output=True, text=True, timeout=60)
    return r.stdout.splitlines()


def runner():
    """Context manager-free helper: returns (run_case, cleanup) with executables cached per (sketch, variant)."""
    tmp = Path(tempfile.mkdtemp())
    cache = {}

    def run_case(sketch, variant, stream, gap=0):
        key = (sketch, variant)
        if key not in cache:
            cache[key] = build(sketch, variant, tmp)
        return run(cache[key], stream, gap)
    return run_case, lambda: shutil.rmtree(tmp, ignore_errors=True)
