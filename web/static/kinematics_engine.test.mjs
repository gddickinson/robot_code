// Node test: the JS port agrees with robot_kinematics.py (tests/fixtures/kinematics_cases.json) and with the compiled
// Arduino sketch's handling of the serial strings (values from tests/test_serial_sketch.py).
// Run: node web/static/kinematics_engine.test.mjs
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { scanPoints, wheelSpeeds, serialCommands, rrb3Command, twosComplement, compassBearing, pyF, arduinoMotor, SERIAL_AXLE } from './kinematics_engine.js';

const fx = JSON.parse(readFileSync(new URL('../../tests/fixtures/kinematics_cases.json', import.meta.url)));
const close = (a, b, msg) => assert.ok(Math.abs(a - b) <= 1e-12 * Math.max(1, Math.abs(b)), `${msg}: ${a} vs ${b}`);
for (const t of fx.twists) {
  const w = wheelSpeeds(-t.x, t.z, SERIAL_AXLE);
  close(w[0], t.wheels_serial[0], 'left'); close(w[1], t.wheels_serial[1], 'right');
  assert.deepEqual(serialCommands(t.x, t.z), t.serial, `serial ${t.x},${t.z}`);
  rrb3Command(t.x, t.z).forEach((v, i) => close(v, t.rrb3[i], `rrb3 ${t.x},${t.z}[${i}]`));
}
for (const t of fx.ties) assert.equal(pyF(t.v), t.f, `%f of ${t.v}`);
for (const w of fx.words) assert.equal(twosComplement(w.high, w.low), w.value);
for (const c of fx.compass) close(compassBearing(c.x, c.y), c.bearing, `compass ${c.x},${c.y}`);
scanPoints(fx.scan.ranges, fx.scan.angle_increment).forEach(([x, y], i) => { close(x, fx.scan.x[i], 'scan x'); close(y, fx.scan.y[i], 'scan y'); });
assert.deepEqual(arduinoMotor('$L25.000000Z'), { value: 25, call: 'run(25)' });     // as the compiled sketch does
assert.deepEqual(arduinoMotor('$R-3.000000Z'), { value: -3, call: 'stop()' });
assert.deepEqual(arduinoMotor('$L-400.000000Z'), { value: -255, call: 'run(-255)' });
console.log(`kinematics engine ok (${fx.twists.length} twists, ${fx.compass.length} headings match robot_kinematics.py)`);
