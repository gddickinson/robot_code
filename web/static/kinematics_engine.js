// JS port of robot_kinematics.py (the pure maths the ROS nodes import) plus the Arduino end of the serial link.
// kinematics_engine.test.mjs checks every function against values computed by the Python module
// (tests/fixtures/kinematics_cases.json from scripts/make_kinematics_fixture.py).

export const SERIAL_AXLE = 0.36, PI_AXLE = 0.18;

export function wheelSpeeds(linearX, angularZ, axle) {
  return [linearX + (axle / 2) * angularZ, linearX - (axle / 2) * angularZ];
}

// Python's "%f": 6 decimals, exact ties rounded half to even, and -0.0 printed as "-0.000000".
export function pyF(x) {
  const neg = x < 0 || Object.is(x, -0), a = Math.abs(x);
  let s = a.toFixed(6);
  if (Number.isInteger(a * 2e6) && !Number.isInteger(a * 1e6)) {      // exact tie at the 7th decimal
    let n = Math.floor(a * 1e6);
    if (n % 2) n += 1;
    s = (n / 1e6).toFixed(6);
  }
  return (neg ? '-' : '') + s;
}

export function serialCommands(linearX, angularZ, axle = SERIAL_AXLE) {
  const [l, r] = wheelSpeeds(-1 * linearX, angularZ, axle);
  return [`$L${pyF(l * 100)}Z`, `$R${pyF(r * 100)}Z`];
}

export function rrb3Command(linearX, angularZ, axle = PI_AXLE, gain = 3) {
  const [l, r] = wheelSpeeds(linearX, angularZ, axle);
  let lm = l * gain, rm = r * gain, ld = 0, rd = 0;
  if (lm < 0) { lm = lm * -1; ld = 1; }
  if (rm < 0) { rm = rm * -1; rd = 1; }
  return [Math.min(lm, 1), ld, Math.min(rm, 1), rd];
}

export function twosComplement(high, low) {
  const v = (high << 8) + low;
  return v >= 0x8000 ? -((65535 - v) + 1) : v;
}

export function compassBearing(xRaw, yRaw, scale = 0.92) {
  let b = Math.atan2(yRaw * scale, xRaw * scale);
  if (b < 0) b += 2 * Math.PI;
  return b * 180 / Math.PI;
}

// What arduino/Arduino_serial_control_WORKS does with one "$L<n>Z" string: atoi of the text after the letter
// (stops at the '.'), clamp to +-255, deadband 5 (|v| <= 5 -> stop()). Checked against the compiled sketch in
// tests/test_serial_sketch.py.
export function arduinoMotor(command) {
  const m = /^\$[LR]\s*([+-]?\d*)/.exec(command);
  let v = m && m[1] && m[1] !== '+' && m[1] !== '-' ? parseInt(m[1], 10) : 0;
  v = Math.max(-255, Math.min(255, v));
  return { value: v, call: v > 5 || v < -5 ? `run(${v})` : 'stop()' };
}

// plotLaserScan.py: x = r cos(i * angle_increment), y = r sin(i * angle_increment) for i = 0..359.
export function scanPoints(ranges, angleIncrement) {
  return ranges.map((r, i) => [r * Math.cos(angleIncrement * i), r * Math.sin(angleIncrement * i)]);
}
