"""Pure robot maths shared by the ROS nodes (no ROS, serial or GPIO imports; works on Python 2 and 3).

Factored out of twist_serial.py, twist_piRobot.py and bearing.py so it can be tested and reused (the web interface
imports it); those scripts now import these functions, with the same numbers as before.

- wheel_speeds: differential-drive split of a /cmd_vel twist into left/right wheel speeds.
- serial_commands: twist -> the "$L<n>Z" / "$R<n>Z" strings twist_serial.py writes to the Arduino.
- rrb3_command: twist -> (left power, left direction, right power, right direction) for the RasPiRobot board (RRB3).
- twos_complement / compass_bearing: HMC5883L-style magnetometer registers -> heading in degrees (bearing.py).
"""
import math

SERIAL_AXLE = 0.36   # m, twist_serial.py (Arduino robot)
PI_AXLE = 0.18       # m, twist_piRobot.py (Raspberry Pi robot)


def wheel_speeds(linear_x, angular_z, axle):
    """Left and right wheel speeds for a forward speed and a turn rate ('+z' = left, '-z' = right)."""
    left = linear_x + (axle / 2.0) * angular_z
    right = linear_x - (axle / 2.0) * angular_z
    return left, right


def serial_commands(linear_x, angular_z, axle=SERIAL_AXLE):
    """The two serial strings twist_serial.py sends: wheel speeds x100, with the forward speed negated (the Arduino
    robot's motors run backwards for positive values)."""
    left, right = wheel_speeds(-1 * linear_x, angular_z, axle)
    return "$L%fZ" % (left * 100), "$R%fZ" % (right * 100)


def rrb3_command(linear_x, angular_z, axle=PI_AXLE, gain=3):
    """Motor powers (0-1) and directions (0 forward, 1 reverse) for RRB3.set_motors, as in twist_piRobot.py."""
    left, right = wheel_speeds(linear_x, angular_z, axle)
    l_motor, r_motor = left * gain, right * gain
    l_dir = r_dir = 0
    if l_motor < 0:
        l_motor, l_dir = l_motor * -1, 1
    if r_motor < 0:
        r_motor, r_dir = r_motor * -1, 1
    return min(l_motor, 1), l_dir, min(r_motor, 1), r_dir


def twos_complement(high, low):
    """Signed 16-bit value from a high and a low register byte (read_word_2c in bearing.py)."""
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    return val


def compass_bearing(x_raw, y_raw, scale=0.92):
    """Heading in degrees, 0 <= h < 360, from the raw X and Y magnetometer readings (atan2(y, x), as in bearing.py)."""
    bearing = math.atan2(y_raw * scale, x_raw * scale)
    if bearing < 0:
        bearing += 2 * math.pi
    return math.degrees(bearing)
