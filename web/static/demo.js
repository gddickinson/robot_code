// Kinematics demo: robot_kinematics.py ported to JS (kinematics_engine.js, checked against the Python in node tests):
// a twist -> wheel speeds -> serial strings (Arduino robot) or RRB3 motor commands (Pi robot); raw magnetometer values
// -> compass heading; and the laser scan recorded in plotLaserScan.py.
import { route, app, api, esc, list } from './core.js';
import { serialCommands, rrb3Command, wheelSpeeds, compassBearing, arduinoMotor, scanPoints, SERIAL_AXLE, PI_AXLE } from './kinematics_engine.js';

const f3 = v => (Math.abs(v) < 5e-13 ? 0 : v).toFixed(3);

function bars(l, r, max, labels) {
  const bar = (x, v, lab) => {
    const h = Math.round(Math.min(1, Math.abs(v) / max) * 60);
    return `<rect x="${x - 14}" y="40" width="28" height="130" rx="5" class="whl"/>
      ${h ? `<rect x="${x - 6}" y="${v >= 0 ? 105 - h : 105}" width="12" height="${h}" class="${v >= 0 ? 'fw' : 'bw'}"/>` : ''}
      <text class="tm" x="${x}" y="190" text-anchor="middle">${esc(lab)}</text>`;
  };
  return `<svg viewBox="0 0 260 200" role="img" aria-label="Wheel values: left ${f3(l)}, right ${f3(r)}">
    <rect x="70" y="30" width="120" height="150" rx="10" class="body"/><text class="tm" x="130" y="20" text-anchor="middle">bars: + up, − down</text>
    ${bar(48, l, labels[0])}${bar(212, r, labels[1])}</svg>`;
}

function dial(deg) {
  const a = (deg - 90) * Math.PI / 180, x = 80 + 60 * Math.cos(a), y = 80 + 60 * Math.sin(a);
  return `<svg viewBox="0 0 160 160" role="img" aria-label="Heading ${deg.toFixed(1)} degrees, drawn clockwise from the top">
    <circle cx="80" cy="80" r="70" class="whl"/><text class="tm" x="80" y="22" text-anchor="middle">0°</text>
    <text class="tm" x="146" y="84" text-anchor="middle">90°</text><text class="tm" x="80" y="150" text-anchor="middle">180°</text>
    <text class="tm" x="14" y="84" text-anchor="middle">270°</text><line x1="80" y1="80" x2="${x.toFixed(1)}" y2="${y.toFixed(1)}" class="needle"/></svg>`;
}

route('demo', async () => {
  const st = { robot: 'serial', x: 0.2, z: 0.5, mx: 120, my: 40, hideShort: false };
  app().innerHTML = `<p class="kicker">Interactive demo</p><h1>Kinematics demo</h1>
  <p class="lead">The maths in <code>robot_kinematics.py</code>, which the ROS nodes import, ported to JavaScript and
  checked against the Python in the tests. Nothing here moves a simulated robot: it shows the numbers the nodes compute.</p>
  <h2>A /cmd_vel twist to the motors</h2>
  <div class="demo-grid">
    <div class="panel">
      <div class="seg" role="group" aria-label="Robot"><button id="r-ser" class="on" type="button">Arduino robot (twist_serial.py)</button><button id="r-pi" type="button">Pi robot (twist_piRobot.py)</button></div>
      <div id="sl"></div>
      <div id="out" class="out"></div>
    </div>
    <div class="panel"><div id="pic" class="pic"></div></div>
  </div>
  <h3>Computed by the Python module (robot_kinematics.py)</h3><div id="py" class="scroll"></div>
  <h2>Compass heading from raw magnetometer values</h2>
  <div class="demo-grid"><div class="panel"><div id="csl"></div><div id="cout" class="out"></div></div>
    <div class="panel"><div id="dial" class="pic"></div></div></div>
  <h2>The recorded lidar scan</h2>
  <div class="demo-grid"><div class="panel"><p class="small">The 360 ranges stored in <code>plotLaserScan.py</code> (one XV-11
    scan), plotted as the script does: x = r·cos θ, y = r·sin θ, with the robot at the centre.</p><div id="sinfo" class="small"></div>
    <label><input type="checkbox" id="hide"> hide readings under 0.06 m (display filter, not in the script)</label></div>
    <div class="stage"><canvas id="scan" width="560" height="560" aria-label="Lidar scan points around the robot"></canvas></div></div>`;
  const $ = id => document.getElementById(id);
  const mk = (host, id, label, min, max, step, key, unit) => {
    const w = document.createElement('div'); w.className = 'ctl';
    w.innerHTML = `<label for="${id}">${esc(label)} <b>${st[key]}${unit}</b></label><input type="range" id="${id}" min="${min}" max="${max}" step="${step}" value="${st[key]}">`;
    host.appendChild(w);
    w.querySelector('input').addEventListener('input', e => { st[key] = +e.target.value; w.querySelector('b').textContent = st[key] + unit; draw(); });
  };
  mk($('sl'), 'lx', 'linear.x (forward speed)', -1, 1, 0.05, 'x', ' m/s');
  mk($('sl'), 'az', 'angular.z (turn rate, + = left)', -3, 3, 0.1, 'z', ' rad/s');
  mk($('csl'), 'mx', 'X reading (signed 16-bit)', -600, 600, 1, 'mx', '');
  mk($('csl'), 'my', 'Y reading (signed 16-bit)', -600, 600, 1, 'my', '');

  function draw() {
    if (st.robot === 'serial') {
      const [l, r] = wheelSpeeds(-st.x, st.z, SERIAL_AXLE), [cl, cr] = serialCommands(st.x, st.z);
      const ml = arduinoMotor(cl), mr = arduinoMotor(cr);
      $('out').innerHTML = `<p>wheel speeds (forward negated, axle ${SERIAL_AXLE} m): left ${f3(l)}, right ${f3(r)}</p>
        <p>serial: <code>${esc(cl)}</code> <code>${esc(cr)}</code></p>
        <p>Arduino sketch: left motor1.${ml.call}, right motor2.${mr.call}</p>`;
      $('pic').innerHTML = bars(ml.value, mr.value, 255, [`M1 ${ml.value}`, `M2 ${mr.value}`]);
    } else {
      const [l, r] = wheelSpeeds(st.x, st.z, PI_AXLE), [lp, ld, rp, rd] = rrb3Command(st.x, st.z);
      $('out').innerHTML = `<p>wheel speeds (axle ${PI_AXLE} m): left ${f3(l)}, right ${f3(r)}</p>
        <p><code>set_motors(${f3(lp)}, ${ld}, ${f3(rp)}, ${rd})</code> (power 0-1, direction 1 = reverse)</p>`;
      $('pic').innerHTML = bars(ld ? -lp : lp, rd ? -rp : rp, 1, [`left ${ld ? '−' : ''}${f3(lp)}`, `right ${rd ? '−' : ''}${f3(rp)}`]);
    }
    const b = compassBearing(st.mx, st.my);
    $('cout').innerHTML = `<p>heading = atan2(${st.my}·0.92, ${st.mx}·0.92) = <b>${b.toFixed(2)}°</b></p>`;
    $('dial').innerHTML = dial(b);
  }
  const pick = robot => { st.robot = robot; $('r-ser').classList.toggle('on', robot === 'serial'); $('r-pi').classList.toggle('on', robot === 'pi'); draw(); };
  $('r-ser').addEventListener('click', () => pick('serial'));
  $('r-pi').addEventListener('click', () => pick('pi'));
  draw();

  const [ex, s] = await Promise.all([api('/examples'), api('/laserscan')]);
  $('py').innerHTML = `<table><thead><tr><th>linear.x</th><th>angular.z</th><th>serial strings</th><th>RRB3 set_motors</th></tr></thead><tbody>
    ${list(ex, e => `<tr><td>${e.x}</td><td>${e.z}</td><td><code>${esc(e.serial.join(' '))}</code></td><td><code>${esc(e.rrb3.join(', '))}</code></td></tr>`)}</tbody></table>`;
  const short = s.ranges.filter(r => r < 0.06).length;
  $('sinfo').innerHTML = `<p>${s.n} readings, ${Math.min(...s.ranges).toFixed(3)}-${Math.max(...s.ranges).toFixed(3)} m; ${short} are under 0.06 m
    (0.053 or 0.033 m), plotted as recorded unless hidden.</p>`;
  const pts = scanPoints(s.ranges, s.angle_increment);
  const drawScan = () => {
    const c = $('scan'), g = c.getContext('2d'), k = 50;
    g.fillStyle = '#000'; g.fillRect(0, 0, 560, 560);
    g.strokeStyle = '#212c4c';
    for (let m = 1; m <= 5; m++) { g.beginPath(); g.arc(280, 280, m * k, 0, 2 * Math.PI); g.stroke(); }
    g.fillStyle = '#94a3c6'; g.font = '11px monospace';
    for (let m = 1; m <= 5; m++) g.fillText(`${m} m`, 284 + m * k, 276);
    g.fillStyle = '#7cff6b';
    pts.forEach(([x, y], i) => { if (st.hideShort && s.ranges[i] < 0.06) return; g.fillRect(280 + x * k - 1.5, 280 - y * k - 1.5, 3, 3); });
    g.fillStyle = '#ff5fa2'; g.beginPath(); g.arc(280, 280, 5, 0, 2 * Math.PI); g.fill();
  };
  $('hide').addEventListener('change', e => { st.hideShort = e.target.checked; drawScan(); });
  drawScan();
});
