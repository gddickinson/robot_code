# INTERFACE.md - navigation map

## Robot code
- `robot_kinematics.py` - pure maths (no ROS/hardware, Python 2/3): `wheel_speeds`, `serial_commands`,
  `rrb3_command`, `twos_complement`, `compass_bearing`. Imported by `twist_serial.py`, `twist_piRobot.py`, `bearing.py`
  and by the web app.
- `twist_serial.py` - /turtle1/cmd_vel -> `$L..Z`/`$R..Z` over serial to the Arduino robot.
- `twist_piRobot.py` - /turtle1/cmd_vel -> RRB3 motors (Pi robot); `wheelSpeed.py`, `wheelCounter.py`, `bearing.py`,
  `bearingNode.py` (stub), `testAccelerometer.py`, `testPin.py` - sensor scripts.
- `piRobot/` - Pi robot ROS nodes: `twist_piRobotNode*.py` (motors + dead reckoning in Node3), `wheelEncoderNode.py`,
  `bearingNode.py`, `odomNode.py`, `velocityLogger.py`, `rotate90*.py`, `piRobotLaunch.launch`; third-party:
  `odom_ekf.py`, `odom_out_and_back.py`, `timed_out_and_back.py` (Patrick Goebel, GPL).
- `outAndBack.py` (Tk keyboard driver), `robotController.py` (room/tile model), `plotLaserScan.py` (one recorded scan),
  `launchpad_node.py`, `Create2_TetheredDrive.py` (iRobot), `roomba_robot/` launch files, `*.sh` ROS network scripts.
- `arduino/` - Arduino sketches (serial control, Robot_Base_Code4).

## Web interface (`web/`)
- `web/app.py` - FastAPI app and `STATIC_ENDPOINTS`; `web/codebase.py` - repo scanner; `web/project.py` - this repo's
  text, citations, About, `/api/examples` and `/api/laserscan`; `web/laserscan.py` - the scan from plotLaserScan.py.
- `web/static/` - `main.js`, `home.js`, `code.js`, `highlight.js`, `how.js`, `demo.js`, `kinematics_engine.js`
  (+ `.test.mjs`), `core.js`, `static_api.js`, styles. `run_web.py` (port 8259), `scripts/build_static.py`,
  `scripts/screenshots.py`, `scripts/make_kinematics_fixture.py` -> `tests/fixtures/kinematics_cases.json`.

## Tests
- `tests/test_web.py`, `tests/test_robot_kinematics.py`, `tests/test_piRobot_dead_reckoning.py` (with
  `tests/ros_stubs.py`), `tests/test_serial_sketch.py` (with `tests/arduino_harness`, `scripts/serial_harness.py`).
