"""Project-specific content for the robot_code showcase: excludes, third-party files, tiles, the "How it works"
sections (claims cited to a file + a text that must occur on the cited line; resolved and checked by the tests),
About, and the demo endpoints (robot_kinematics.py outputs and the recorded laser scan).
"""
import sys
from pathlib import Path

from fastapi import FastAPI

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import robot_kinematics as rk  # noqa: E402  (the module the ROS nodes import)

from .laserscan import scan  # noqa: E402

TITLE = "ROS Robot Code"
REPO = "robot_code"
PORT = 8259
KICKER = "ROS · Python · Raspberry Pi · Arduino"
DESCRIPTION = ("ROS nodes and Arduino code for two home-built mobile robots (an Arduino-driven base and a Raspberry Pi "
               "robot): /cmd_vel to motors, wheel encoders, compass heading, dead reckoning and lidar.")
DEMO = {"title": "Kinematics demo",
        "blurb": "Drive with linear and angular velocity and see the wheel speeds, serial strings and motor commands the "
                 "nodes produce, a compass heading from raw magnetometer values, and the lidar scan stored in the repo."}

EXCLUDE = ("web/*", "scripts/*", "tests/*", "run_web.py", "requirements-web.txt", ".github/*", "docs/*",
           "SESSION_LOG.md", "INTERFACE.md", ".gitignore", "*~", "*.swp", ".DS_Store")
VENDORED = (("Create2_TetheredDrive.py", "Create2_TetheredDrive.py (iRobot)"),
            ("piRobot/odom_ekf.py", "odom_ekf.py (Patrick Goebel)"),
            ("piRobot/odom_out_and_back.py", "odom_out_and_back.py (Patrick Goebel)"),
            ("piRobot/timed_out_and_back.py", "timed_out_and_back.py (Patrick Goebel)"))
NOT_CODE = ("Markdown", "Text")


def tiles(shown):
    nodes = [f for f in shown if f["lang"] == "Python" and "import rospy" in f["text"]]
    launch = [f for f in shown if f["path"].endswith(".launch")]
    ino = [f for f in shown if f["path"].endswith(".ino")]
    return [[len(nodes), "ROS node scripts (rospy)"], [len(launch), "launch files"], [len(ino), "Arduino .ino files"]]


N3 = "piRobot/twist_piRobotNode3.py"
SVG_GRAPH = """<svg viewBox="0 0 780 260" role="img" aria-label="ROS graph of the Raspberry Pi robot: wheelEncoderNode
publishes lwheel and rwheel, bearingNode publishes robot_bearing, twist_piRobotNode3 subscribes to cmd_vel and these and
drives the RRB3 motor board and publishes odometry; an XV-11 lidar node publishes the laser scan">
<defs><marker id="arw" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
<path class="arh" d="M0 0L10 5L0 10z"/></marker></defs>
<rect class="bx3" x="10" y="20" width="170" height="44" rx="6"/><text class="t" x="95" y="47" text-anchor="middle">teleop / planner</text>
<rect class="bx3" x="10" y="96" width="170" height="44" rx="6"/><text class="t" x="95" y="123" text-anchor="middle">wheelEncoderNode</text>
<rect class="bx3" x="10" y="172" width="170" height="44" rx="6"/><text class="t" x="95" y="199" text-anchor="middle">bearingNode</text>
<path class="ar" d="M180 42H300"/><text class="tm" x="240" y="36" text-anchor="middle">/cmd_vel</text>
<path class="ar" d="M180 118H300"/><text class="tm" x="240" y="112" text-anchor="middle">/lwheel /rwheel</text>
<path class="ar" d="M180 194H300"/><text class="tm" x="240" y="188" text-anchor="middle">/robot_bearing (°)</text>
<rect class="bx" x="303" y="20" width="200" height="196" rx="6"/><text class="t" x="403" y="50" text-anchor="middle">twist_piRobotNode3</text>
<text class="tm" x="403" y="80" text-anchor="middle">wheel_speeds → RRB3</text><text class="tm" x="403" y="100" text-anchor="middle">set_motors(power, dir)</text>
<text class="tm" x="403" y="130" text-anchor="middle">dead reckoning X, Y</text><text class="tm" x="403" y="150" text-anchor="middle">tf odom → base_link</text>
<path class="ar" d="M503 60H600"/><rect class="bx2" x="603" y="38" width="165" height="44" rx="6"/><text class="t" x="685" y="65" text-anchor="middle">RRB3 motors</text>
<path class="ar" d="M503 150H600"/><rect class="bx2" x="603" y="128" width="165" height="44" rx="6"/><text class="t" x="685" y="155" text-anchor="middle">/piRobot, X, Y</text>
<rect class="bx3" x="603" y="200" width="165" height="44" rx="6"/><text class="t" x="685" y="227" text-anchor="middle">xv11_laser → /scan</text>
</svg>"""

HOW = [
    {"id": "graph", "title": "Two robots, one ROS graph", "svg": SVG_GRAPH,
     "caption": "The Raspberry Pi robot as wired up by piRobot/piRobotLaunch.launch and twist_piRobotNode3.py.",
     "body": """<p>The repo drives two robots from ROS. The <b>Arduino robot</b> runs <code>twist_serial.py</code> on the
computer: it subscribes to <code>/turtle1/cmd_vel</code> (so the turtlesim teleop keys drive it) and writes
<code>$L…Z</code>/<code>$R…Z</code> strings at 19200 baud to the Arduino sketch in <code>arduino/</code>. The
<b>Raspberry Pi robot</b> uses a RasPiRobot board (RRB3) for its motors; its launch file starts the motor node, a wheel
encoder node, the XV-11 lidar driver, a compass node and a USB camera. <code>twist_piRobotNode3.py</code> combines
<code>/cmd_vel</code>, the wheel speeds and the compass bearing, drives the motors and publishes odometry and a tf
transform. There is also a launch file for an iRobot Roomba through the turtlebot driver.</p>""",
     "cites": [
         {"file": "twist_serial.py", "find": "ser = serial.Serial('/dev/ttyACM0', 19200)"},
         {"file": "twist_serial.py", "find": 'rospy.Subscriber("/turtle1/cmd_vel", Twist, callback)'},
         {"file": "piRobot/piRobotLaunch.launch", "find": 'name="wheelEncoder"'},
         {"file": "piRobot/piRobotLaunch.launch", "find": 'name="xv11_laser"'},
         {"file": N3, "find": 'rospy.Subscriber("/robot_bearing", Float32, bearing_callback)'},
         {"file": N3, "find": 'OdometryPublisher = rospy.Publisher("piRobot", Odometry,queue_size=10)'},
         {"file": "roomba_robot/roombaRobotLaunch2.launch", "find": "robot_type: roomba"},
     ]},
    {"id": "drive", "title": "From /cmd_vel to the wheels",
     "body": """<p>Both robots use the differential-drive split: left = v + (axle/2)·ω and right = v − (axle/2)·ω, with
'+z' turning left. The Arduino robot (axle 0.36 m) negates the forward speed because its motors run backwards for
positive values, multiplies by 100 and formats the result with <code>%f</code>, e.g. <code>$L-25.000000Z</code>;
the sketch's <code>atoi</code> keeps the integer part, clamps to ±255 and stops a motor for |value| ≤ 5. The Pi robot
(axle 0.18 m) multiplies by 3, turns a negative value into direction 1 (reverse) and caps the power at 1 before
<code>RRB3.set_motors(left, left_dir, right, right_dir)</code>. This maths now lives in
<code>robot_kinematics.py</code>, which the nodes import and the demo page uses.</p>""",
     "cites": [
         {"file": "robot_kinematics.py", "find": "left = linear_x + (axle / 2.0) * angular_z"},
         {"file": "robot_kinematics.py", "find": "SERIAL_AXLE = 0.36"},
         {"file": "robot_kinematics.py", "find": "left, right = wheel_speeds(-1 * linear_x, angular_z, axle)"},
         {"file": "robot_kinematics.py", "find": 'return "$L%fZ" % (left * 100), "$R%fZ" % (right * 100)'},
         {"file": "robot_kinematics.py", "find": "return min(l_motor, 1), l_dir, min(r_motor, 1), r_dir"},
         {"file": "twist_piRobot.py", "find": "piRobot.set_motors(LMotor,LDirection,RMotor,RDirection)"},
         {"file": "arduino/Arduino_serial_control_WORKS/Arduino_serial_control_WORKS.ino", "find": "int l_val = atoi(temp);"},
     ]},
    {"id": "encoders", "title": "Wheel encoders on the Pi's GPIO",
     "body": """<p>The encoders are read on GPIO pins 21 (left) and 20 (right). <code>wheelCount()</code> polls both pins for
0.1 s and counts every change of level; four such samples are averaged. With a 3.5 cm wheel radius and 20 slots
(two edges each) one count is 2π·3.5/40 cm, so speed = average count × (2π·3.5 / 40) / 0.1 cm/s;
<code>wheelEncoderNode.py</code> divides by 100 and publishes m/s on <code>/lwheel</code> and <code>/rwheel</code> five
times a second.</p>""",
     "cites": [
         {"file": "piRobot/wheelEncoderNode.py", "find": "leftpinNumber = 21"},
         {"file": "piRobot/wheelEncoderNode.py", "find": "if (leftFlag == leftinputValue):"},
         {"file": "piRobot/wheelEncoderNode.py", "find": "conversionFactor = (wheelCircumference / (wheelEncoderPins*2))/ sampleLength"},
         {"file": "piRobot/wheelEncoderNode.py", "find": "leftWheelSpeed = leftaverageSample * conversionFactor/100"},
         {"file": "piRobot/wheelEncoderNode.py", "find": "Left_Encoder = rospy.Publisher('lwheel',Float32,queue_size = 10)"},
     ]},
    {"id": "compass", "title": "Compass heading over I²C",
     "body": """<p><code>bearing.py</code> and <code>piRobot/bearingNode.py</code> talk to a magnetometer at I²C address
0x1e: they configure 8-sample averaging at 15 Hz, gain 1090 LSb/gauss and continuous mode, then read X (registers 3-4),
Z (5-6) and Y (7-8) as signed 16-bit values, scale them by 0.92 and take the heading as atan2(y, x) in degrees, folded
into 0-360. No calibration offsets are applied (they are commented out).</p>""",
     "cites": [
         {"file": "bearing.py", "find": "address = 0x1e"},
         {"file": "bearing.py", "find": "write_byte(0, 0b01110000) # Set to 8 samples @ 15Hz"},
         {"file": "bearing.py", "find": "scale = 0.92"},
         {"file": "bearing.py", "find": "#x_offset = -10"},
         {"file": "robot_kinematics.py", "find": "bearing = math.atan2(y_raw * scale, x_raw * scale)"},
         {"file": "piRobot/bearingNode.py", "find": "return format(math.degrees(bearing), '.2f')"},
     ]},
    {"id": "dead-reckoning", "title": "Dead reckoning (and a fix)",
     "body": """<p><code>twist_piRobotNode3.py</code> assumes the robot only drives straight or turns on the spot. Each
command sets <code>DIRECTION</code> (+1 forward, -1 reverse, 0 stopped or turning); five times a second the node moves
X, Y by speed × time since the last command along the compass bearing, and publishes them with the forward speed
(mean of the wheel speeds) and turn rate (difference / axle).</p>
<p>Two bugs were fixed: the direction test read the RRB3 direction flags the wrong way round (both wheels forward gave
0, both reverse gave +1, and the -1 branch could never run), and the bearing, which arrives in degrees, went straight
into <code>cos</code>/<code>sin</code>; it is now converted to radians. Open question for George: a compass bearing
runs clockwise from north while ROS headings run anticlockwise from x, so the X/Y axes may still be mirrored.</p>""",
     "cites": [
         {"file": N3, "find": "###ROBOT only to be travelling in straight lines"},
         {"file": N3, "find": "elif LDirection == 0 and RDirection == 0:"},
         {"file": N3, "find": "theta = math.radians(BEARING)"},
         {"file": N3, "find": "delta_x = distance * math.cos(bearing)"},
         {"file": N3, "find": "omega = float((rightWheel - leftWheel)/axle)"},
     ]},
    {"id": "tools", "title": "Lidar plot, keyboard driving and a room model",
     "body": """<p><code>plotLaserScan.py</code> holds one recorded XV-11 scan (360 ranges, 1° apart) and plots it as
x = r·cos θ, y = r·sin θ; the demo page draws the same points. <code>outAndBack.py</code> is a Tk keyboard driver for
the Pi robot (i/m/j/l/k for forward, reverse, left, right, stop; f drives a set distance; c/x rotate 90°), and
<code>robotController.py</code> models a room as 1 cm tiles with blocked cells for a simulated robot. The shell
scripts set <code>ROS_MASTER_URI</code> and friends to the master host before starting nodes.</p>""",
     "cites": [
         {"file": "plotLaserScan.py", "find": "angle_increment = 0.0174532923847"},
         {"file": "plotLaserScan.py", "find": "xpoints = np.multiply(ranges, np.cos(radians))"},
         {"file": "outAndBack.py", "find": "robot.set_motors(0.8,0,0.8,0) #(left speed, left direction"},
         {"file": "robotController.py", "find": "#room squares 1cmx1cm"},
         {"file": "ROS_terminal_startup.sh", "find": 'export ROS_MASTER_URI="http://$var:11311"'},
     ]},
]

ABOUT = {
    "port": PORT,
    "what": """<p>ROS (Python 2 era, 2015-2017) and Arduino code for George's home-built robots: a Makeblock/Arduino base
driven over serial, a Raspberry Pi robot on a RasPiRobot v3 board with wheel encoders, a compass, an XV-11 lidar and a
camera, and an iRobot Roomba/Create 2. The Arduino sketches are also in the
<a href="https://github.com/gddickinson/Arduino">Arduino</a> repo; the networking scripts in
<a href="https://github.com/gddickinson/shell_scripts">shell_scripts</a>.</p>""",
    "needs": ["ROS 1 (the scripts use rospy, tf, geometry_msgs, nav_msgs; written for Python 2).",
              "Raspberry Pi robot: RasPiRobot board v3 (<code>rrb3</code> library), wheel encoders on GPIO 20/21, an "
              "I²C magnetometer at 0x1e, XV-11 lidar (<code>xv_11_laser_driver</code>), USB camera.",
              "Arduino robot: the sketch in <code>arduino/</code> on a Makeblock board, connected on /dev/ttyACM0."],
    "run": ["Copy the node scripts (with <code>robot_kinematics.py</code> next to them) into a catkin package "
            "(the launch files use <code>beginner_tutorials</code>), then e.g. "
            "<code>roslaunch beginner_tutorials piRobotLaunch.launch</code>.",
            "Arduino robot: <code>rosrun turtlesim turtle_teleop_key</code> and <code>python twist_serial.py</code>."],
    "credits": ["George Dickinson.", "Create2_TetheredDrive.py: iRobot Corporation (BSD-style licence in its header).",
                "piRobot/odom_ekf.py, odom_out_and_back.py, timed_out_and_back.py: Patrick Goebel, Pi Robot Project "
                "(GNU GPL v2 or later, per their headers)."],
    "licence": "MIT (LICENSE) for George's code; the third-party files above keep their own licences (THIRD_PARTY.md).",
}

EXTRA_ENDPOINTS = ["/examples", "/laserscan"]


def register(app: FastAPI):
    @app.get("/api/examples")
    def examples():
        """A few twists through robot_kinematics.py (the module the nodes import), shown as 'computed by Python'."""
        rows = []
        for x, z in [(0.25, 0.0), (-0.25, 0.0), (0.0, 1.0), (0.2, 0.5), (0.5, -2.0)]:
            rows.append({"x": x, "z": z, "serial": list(rk.serial_commands(x, z)),
                         "rrb3": [round(v, 6) for v in rk.rrb3_command(x, z)]})
        return rows

    @app.get("/api/laserscan")
    def laserscan():
        return scan()
