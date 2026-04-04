#!/usr/bin/env python3
"""
Capra Rove — track drive test. Zero dependencies.
Open http://localhost:9000 in your browser and use WASD.
"""

import json, socket, struct, threading, time, urllib.request
import http.server, webbrowser

HOST      = "192.168.168.37"
HTTP_PORT = 8080
SPEED      = 2.0
TURN_SPEED = 1.5

LEFT_NODES  = {32, 33}
RIGHT_NODES = {31, 34}
ALL_NODES   = LEFT_NODES | RIGHT_NODES

# Per-track leader/follower.
# Leader: velocity control.  Follower: torque control, copies leader's iq_setpoint.
# Adjust if motors are physically mounted in opposite orientations on a track.
LEADER_FOLLOWER = {
    32: 33,   # left track:  32 leads → 33 follows
    34: 31,   # right track: 34 leads → 31 follows
}
FOLLOWER_LEADER = {v: k for k, v in LEADER_FOLLOWER.items()}
TORQUE_CONSTANT = 0.236  # Nm/A — from axis0.config.motor.torque_constant, tune if needed

# ── Discovery ─────────────────────────────────────────────────────────────────

def discover_cmd_ports():
    url = f"http://{HOST}:{HTTP_PORT}/discover"
    with urllib.request.urlopen(url, timeout=3) as r:
        sensors = json.loads(r.read())["sensors"]
    ports = {}
    for s in sensors:
        sid = s["id"]
        if sid.startswith("odrive_"):
            nid = int(sid.split("_")[1])
            if nid in ALL_NODES:
                ports[nid] = s["command_port"]
    return ports

# ── Node data fetcher ─────────────────────────────────────────────────────────

class NodeDataFetcher:
    """Background thread that polls /odrive_N/data for all nodes."""
    def __init__(self, node_ids):
        self._data = {nid: {} for nid in node_ids}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not self._stop.is_set():
            for nid in list(self._data):
                try:
                    url = f"http://{HOST}:{HTTP_PORT}/odrive_{nid}/data"
                    with urllib.request.urlopen(url, timeout=1) as r:
                        data = json.loads(r.read())
                    with self._lock:
                        self._data[nid] = data
                except Exception:
                    pass
            time.sleep(0.2)

    def get_all(self):
        with self._lock:
            return dict(self._data)

    def stop(self):
        self._stop.set()

# ── UDP stream ────────────────────────────────────────────────────────────────

class DriveStream:
    def __init__(self, ports, fetcher):
        self.sock      = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ports     = ports
        self.fetcher   = fetcher
        self.seqs      = {n: 0 for n in ports}
        self.lv = self.rv = 0.0
        self.estopped  = False
        self._lock     = threading.Lock()
        self._stop     = threading.Event()

        # Flag to reset per-node transition trackers after estop/clear_errors.
        # The loop owns the trackers as locals; this flag crosses the thread boundary.
        self._reset_trackers = False

        # Last velocity/torque command sent to each node — exposed for the UI.
        self._last_cmd = {n: 0.0 for n in ports}

        threading.Thread(target=self._loop, daemon=True).start()

    def set(self, lv, rv):
        with self._lock:
            self.lv, self.rv = lv, rv

    def estop(self):
        """Zero velocities and mark as estopped — loop will stop sending commands."""
        with self._lock:
            self.lv = self.rv = 0.0
            self.estopped = True
            self._reset_trackers = True  # force re-send of mode/state after resume

    def clear_errors(self):
        """Allow normal drive commands again after errors are cleared."""
        with self._lock:
            self.estopped = False
            self._reset_trackers = True  # re-send axis_state=8 and controller_mode

    def get_cmds(self):
        """Return a snapshot of last commands sent per node — for the UI."""
        with self._lock:
            return dict(self._last_cmd)

    def _send(self, nid, port, cmd):
        pay = json.dumps(cmd).encode()
        hdr = struct.pack("<BBH", 0x01, 0x10, self.seqs[nid])
        try:
            self.sock.sendto(hdr + pay, (HOST, port))
        except OSError:
            pass
        self.seqs[nid] = (self.seqs[nid] + 1) & 0xFFFF

    def _loop(self):
        # Per-node transition trackers live here — only this thread reads/writes them.
        # reset_trackers flag crosses the thread boundary via self._lock.
        last_axis_state = {n: None for n in self.ports}
        last_ctrl_mode  = {n: None for n in self.ports}

        while not self._stop.is_set():
            with self._lock:
                lv, rv   = self.lv, self.rv
                estopped = self.estopped
                if self._reset_trackers:
                    last_axis_state = {n: None for n in self.ports}
                    last_ctrl_mode  = {n: None for n in self.ports}
                    self._reset_trackers = False

            # While estopped, send nothing — drives are in error state.
            # The Rust watchdog fires SET_INPUT_POS(0) which does NOT clear ODrive
            # errors, so the ESTOP_REQUESTED state is preserved.
            if estopped:
                time.sleep(0.020)
                continue

            # Always stay in ClosedLoopControl (8) — NEVER send Idle from this loop.
            # Sending axis_state=1 on every keyup caused drives to Idle→ClosedLoop
            # on every key press, creating the "Idle spike" stutter.
            # Only send on first cycle (or after estop/clear_errors resets the tracker).
            state = 8

            # Snapshot node data once per cycle for torque following.
            node_data = self.fetcher.get_all()

            new_cmds = {}
            for nid, port in self.ports.items():
                cmd = {}

                # Axis state: only emit on first cycle or after tracker was reset.
                if last_axis_state[nid] != state:
                    cmd["axis_state"] = state
                    last_axis_state[nid] = state

                moving = bool(lv or rv)

                if nid in FOLLOWER_LEADER:
                    ctrl_key = (1, 1)  # TORQUE_CONTROL, PASSTHROUGH
                    if last_ctrl_mode[nid] != ctrl_key:
                        cmd["control_mode"] = 1
                        cmd["input_mode"]   = 1
                        last_ctrl_mode[nid] = ctrl_key

                    leader = FOLLOWER_LEADER[nid]
                    iq = node_data.get(leader, {}).get("iq_setpoint", 0.0) or 0.0
                    torque = round(iq * TORQUE_CONSTANT, 4) if moving else 0.0
                    cmd["input_torque"] = torque
                    new_cmds[nid] = torque

                else:
                    ctrl_key = (2, 1)  # VELOCITY_CONTROL, PASSTHROUGH
                    if last_ctrl_mode[nid] != ctrl_key:
                        cmd["control_mode"] = 2
                        cmd["input_mode"]   = 1
                        last_ctrl_mode[nid] = ctrl_key

                    vel = round(lv if nid in LEFT_NODES else -rv, 4)
                    cmd["input_vel"] = vel
                    new_cmds[nid] = vel

                self._send(nid, port, cmd)

            # Publish command snapshot for the UI — one lock acquire per cycle.
            with self._lock:
                self._last_cmd.update(new_cmds)

            time.sleep(0.020)  # 20 ms — well below the 250 ms server watchdog

    def stop(self):
        with self._lock:
            self.lv = self.rv = 0.0
            self.estopped = False  # allow final zero-vel frames to go through
        time.sleep(0.15)
        self._stop.set()

# ── HTML ──────────────────────────────────────────────────────────────────────

HTML = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Capra Rove</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body { background:#0d0d0d; color:#eee; font-family:monospace; padding:24px; }
h2 { color:#0f0; margin-bottom:18px; font-size:1.3em; letter-spacing:1px; }

.controls { display:flex; align-items:center; gap:32px; margin-bottom:24px;
            background:#161616; border:1px solid #2a2a2a; border-radius:8px; padding:18px 24px; }
.vel-display { font-size:1.2em; color:#ff0; }
.key-hint { color:#555; font-size:0.9em; line-height:1.8; }
.kbd { display:grid; grid-template-columns:repeat(3,36px); gap:4px; }
.k { background:#222; border:1px solid #444; border-radius:4px; width:36px; height:36px;
     display:flex; align-items:center; justify-content:center; color:#fff; font-size:0.95em; }
.k.active { background:#0a4a0a; border-color:#0f0; color:#0f0; }
.k.blank  { background:transparent; border-color:transparent; }
.estop { background:#8b0000; border:2px solid #ff2222; border-radius:6px; padding:10px 28px;
         color:#fff; font-family:monospace; font-size:1.1em; font-weight:bold;
         cursor:pointer; letter-spacing:2px; transition:background 0.1s; }
.estop:hover { background:#b00000; }
.estop:active, .estop.fired { background:#ff2222; color:#000; }
.clrerr { background:#1a1a00; border:2px solid #aa8800; border-radius:6px; padding:10px 20px;
          color:#cc0; font-family:monospace; font-size:1.0em; font-weight:bold;
          cursor:pointer; letter-spacing:1px; transition:background 0.1s; }
.clrerr:hover { background:#2a2a00; }
.clrerr:active, .clrerr.fired { background:#aa8800; color:#000; }

.cards { display:grid; grid-template-columns:repeat(auto-fill,minmax(340px,1fr)); gap:16px; }
.card { background:#161616; border:1px solid #2a2a2a; border-radius:8px; padding:16px; }
.card.error { border-color:#a00; }
.card-title { font-size:1em; font-weight:bold; color:#0f0; margin-bottom:3px; }
.card-sub   { font-size:0.78em; color:#555; margin-bottom:10px; }
.badge { display:inline-block; padding:2px 8px; border-radius:3px; font-size:0.8em; margin-left:6px; }
.badge.idle   { background:#1a1a00; color:#aa0; }
.badge.run    { background:#001a00; color:#0f0; }
.badge.err    { background:#1a0000; color:#f44; }
.badge.leader { background:#00151f; color:#0af; }
.badge.follow { background:#0f1520; color:#68a; }

.section { margin-top:8px; border-top:1px solid #1e1e1e; padding-top:6px; }
.section-label { font-size:0.7em; color:#333; letter-spacing:1px; text-transform:uppercase;
                 margin-bottom:4px; }
.rows { display:grid; grid-template-columns:1fr 1fr; gap:3px 16px; }
.row { display:flex; justify-content:space-between; padding:2px 0;
       border-bottom:1px solid #1a1a1a; font-size:0.83em; }
.row .lbl { color:#555; }
.row .val { color:#ccc; }
.row .val.hi  { color:#ff0; }
.row .val.err { color:#f44; }
.row .val.warn { color:#fa0; }
.row .val.ok   { color:#0c0; }

/* Temperature colour coding */
.temp-ok   { color:#0c0; }
.temp-warn { color:#fa0; }
.temp-hot  { color:#f44; }
.temp-na   { color:#444; }
</style>
</head>
<body>
<h2>&#9655; Capra Rove — Track Test</h2>

<div class="controls">
  <div class="kbd">
    <div class="k blank"></div><div class="k" id="kw">W</div><div class="k blank"></div>
    <div class="k" id="ka">A</div><div class="k" id="ks">S</div><div class="k" id="kd">D</div>
  </div>
  <div>
    <div class="vel-display" id="vel">L: +0.00 &nbsp; R: +0.00</div>
    <div class="key-hint" style="margin-top:8px">
      SPACE stop &nbsp;|&nbsp; Q quit &nbsp;|&nbsp; E estop &nbsp;|&nbsp; C clear errors
    </div>
  </div>
  <div style="display:flex;flex-direction:column;gap:10px;align-items:center">
    <button class="estop"  id="estopBtn"  onclick="fireEstop()">&#9888; ESTOP</button>
    <button class="clrerr" id="clrErrBtn" onclick="fireClearErrors()">&#10003; CLEAR ERRORS</button>
  </div>
</div>

<div class="cards" id="cards"></div>

<script>
const pressed = new Set();
const SPEED = """ + str(SPEED) + """, TURN = """ + str(TURN_SPEED) + """;
const TORQUE_CONSTANT = """ + str(TORQUE_CONSTANT) + """;

document.addEventListener('keydown', e => {
  const k = e.key.toLowerCase();
  if (['w','a','s','d',' ','q','e','c'].includes(k)) e.preventDefault();
  if (k === 'e') { fireEstop(); return; }
  if (k === 'c') { fireClearErrors(); return; }
  pressed.add(k);
  updateKeys();
});
document.addEventListener('keyup', e => { pressed.delete(e.key.toLowerCase()); updateKeys(); });

function fireClearErrors() {
  fetch('/clearerrors');
  const btn = document.getElementById('clrErrBtn');
  btn.classList.add('fired');
  setTimeout(() => btn.classList.remove('fired'), 400);
}
function fireEstop() {
  pressed.clear(); updateKeys();
  fetch('/cmd', {method:'POST', body:JSON.stringify({lv:0,rv:0}),
                 headers:{'Content-Type':'application/json'}});
  fetch('/estop');
  const btn = document.getElementById('estopBtn');
  btn.classList.add('fired');
  setTimeout(() => btn.classList.remove('fired'), 400);
}
function updateKeys() {
  ['w','a','s','d'].forEach(k =>
    document.getElementById('k'+k).classList.toggle('active', pressed.has(k)));
}
function compute() {
  const w=pressed.has('w'), s=pressed.has('s'), a=pressed.has('a'), d=pressed.has('d');
  if (pressed.has(' '))  return [0, 0];
  if (w && a) return [0,      SPEED];
  if (w && d) return [SPEED,  0];
  if (s && a) return [0,     -SPEED];
  if (s && d) return [-SPEED, 0];
  if (w)      return [SPEED,  SPEED];
  if (s)      return [-SPEED,-SPEED];
  if (a)      return [-TURN,  TURN];
  if (d)      return [ TURN, -TURN];
  return [0, 0];
}

setInterval(() => {
  if (pressed.has('q')) { fetch('/quit'); return; }
  const [lv, rv] = compute();
  fetch('/cmd', {method:'POST', body:JSON.stringify({lv,rv}),
                 headers:{'Content-Type':'application/json'}});
  const fmt = v => (v>=0?'+':'')+v.toFixed(2);
  document.getElementById('vel').innerHTML = `L: ${fmt(lv)} &nbsp; R: ${fmt(rv)}`;
}, 50);

// ── Data display ──────────────────────────────────────────────────────────────

const AXIS_STATES = {
  0:'Undefined',1:'Idle',2:'Startup',3:'FullCal',4:'MotorCal',
  6:'EncSearch',7:'EncCal',8:'ClosedLoop',9:'LockinSpin',
  10:'DirFind',11:'Homing',12:'HallPol',13:'HallPhase'
};
const AXIS_ERRORS = {
  0x00000001:'INVALID_STATE',        0x00000002:'DC_BUS_UNDER_VOLTAGE',
  0x00000004:'DC_BUS_OVER_VOLTAGE',  0x00000008:'CURRENT_MEAS_TIMEOUT',
  0x00000010:'BRAKE_RES_DISARMED',   0x00000020:'MOTOR_DISARMED',
  0x00000040:'MOTOR_FAILED',         0x00000080:'SENSORLESS_FAILED',
  0x00000100:'ENCODER_FAILED',       0x00000200:'CONTROLLER_FAILED',
  0x00000800:'WATCHDOG_EXPIRED',     0x00004000:'ESTOP_REQUESTED',
  0x00010000:'OVER_TEMP',            0x00020000:'UNKNOWN_POSITION',
};
const MOTOR_ERRORS = {
  0x00000001:'PHASE_R_OOR',    0x00000002:'PHASE_L_OOR',   0x00000004:'ADC_FAILED',
  0x00000008:'DRV_FAULT',      0x00000010:'CTRL_DEADLINE',  0x00001000:'CURRENT_LIMIT',
  0x00010000:'MOTOR_OVER_TEMP',0x00020000:'FET_OVER_TEMP',  0x00400000:'WATCHDOG_EXPIRED',
};
const ENCODER_ERRORS = {
  0x00000001:'UNSTABLE_GAIN', 0x00000002:'CPR_POLEPAIRS', 0x00000004:'NO_RESPONSE',
  0x00000010:'ILLEGAL_HALL',  0x00000020:'INDEX_NOT_FOUND',
};
const CONTROLLER_ERRORS = {
  0x00000001:'OVERSPEED', 0x00000002:'INVALID_INPUT_MODE', 0x00000004:'UNSTABLE_GAIN',
  0x00000020:'INVALID_ESTIMATE', 0x00000080:'SPINOUT_DETECTED',
};

function decodeFlags(val, table) {
  if (!val) return null;
  const names = Object.entries(table).filter(([b]) => val & Number(b)).map(([,n]) => n);
  return names.length ? names : [`0x${val.toString(16).toUpperCase()}`];
}
function fmtN(v, dec=3) {
  if (v === undefined || v === null) return '—';
  if (typeof v === 'number') return v.toFixed(dec);
  return String(v);
}
function tempClass(t) {
  if (t === null || t === undefined) return 'temp-na';
  if (t >= 70) return 'temp-hot';
  if (t >= 50) return 'temp-warn';
  return 'temp-ok';
}
function row(label, val, unit='', cls='') {
  return `<div class="row"><span class="lbl">${label}</span>` +
         `<span class="val ${cls}">${val}${unit ? ` <span style="color:#333">${unit}</span>` : ''}</span></div>`;
}
function errCell(flags) {
  if (!flags) return `<span class="val ok">OK</span>`;
  return `<span class="val err" title="${flags.join(', ')}">${flags[0]}${flags.length>1?` +${flags.length-1}`:''}</span>`;
}

function renderCard(nid, d) {
  const hasErr  = d.axis_error !== 0;
  const stLabel = AXIS_STATES[d.axis_state] ?? `State ${d.axis_state}`;
  const side    = ([32,33].includes(nid)) ? 'Left' : 'Right';
  const role    = d._role ?? 'solo';
  const roleBadge = role === 'leader'   ? `<span class="badge leader">&#9650; Leader &rarr; ${d._follow}</span>` :
                    role === 'follower' ? `<span class="badge follow">&#9660; Follower of ${d._follow}</span>` : '';

  const axisErrNames = decodeFlags(d.axis_error, AXIS_ERRORS);
  const stateBadge = hasErr
    ? `<span class="badge err">&#9888; ${axisErrNames.join(', ')}</span>`
    : d.axis_state === 8
      ? `<span class="badge run">&#9679; ${stLabel}</span>`
      : `<span class="badge idle">${stLabel}</span>`;

  // Computed values
  // Power: prefer Get_Powers electrical_power if available, else bus_V*bus_I fallback.
  const elecPwr  = (d.electrical_power != null && d.electrical_power !== 0)
                   ? d.electrical_power.toFixed(1) : null;
  const mechPwr  = (d.mechanical_power != null && d.mechanical_power !== 0)
                   ? d.mechanical_power.toFixed(1) : null;
  const busCalcPwr = (d.bus_voltage != null && d.bus_current != null)
                   ? Math.abs(d.bus_voltage * d.bus_current).toFixed(1) : '—';

  // Torque: prefer Get_Torques, fall back to iq_measured * TORQUE_CONSTANT.
  const torqueEst = (d.torque_estimate != null && d.torque_estimate !== 0)
                   ? d.torque_estimate.toFixed(3)
                   : (d.iq_measured != null ? (d.iq_measured * TORQUE_CONSTANT).toFixed(3) : '—');
  const torqueTgt = d.torque_target != null ? d.torque_target.toFixed(3) : '—';

  const velCmd   = d._vel_cmd != null ? d._vel_cmd.toFixed(3) : '—';

  const fetT     = d.fet_temp   != null ? d.fet_temp.toFixed(1)   : null;
  const motT     = d.motor_temp != null ? d.motor_temp.toFixed(1) : null;
  const fetStr   = fetT != null ? fetT + ' °C' : '—';
  const motStr   = motT != null ? motT + ' °C' : '—';

  const axisErrForErrors = d.active_errors ?? d.axis_error ?? 0;
  const motErrs = decodeFlags(d.axis_error,      MOTOR_ERRORS);
  const ctlErrs = decodeFlags(axisErrForErrors,  CONTROLLER_ERRORS);

  return `<div class="card ${hasErr?'error':''}">
    <div class="card-title">odrive_${nid}</div>
    <div class="card-sub">${side} &nbsp;|&nbsp; node ${nid} ${stateBadge} ${roleBadge}</div>

    <div class="section">
      <div class="section-label">Motion</div>
      <div class="rows">
        ${row('pos',          fmtN(d.pos_estimate, 3), 'rev')}
        ${row('vel',          fmtN(d.vel_estimate, 3), 'rev/s')}
        ${row('cmd',          velCmd,                  role==='follower'?'Nm':'rev/s')}
        ${row('torque tgt',   torqueTgt,               'Nm')}
        ${row('torque est',   torqueEst,               'Nm')}
      </div>
    </div>

    <div class="section">
      <div class="section-label">Current</div>
      <div class="rows">
        ${row('iq set',  fmtN(d.iq_setpoint, 3), 'A')}
        ${row('iq meas', fmtN(d.iq_measured, 3), 'A')}
      </div>
    </div>

    <div class="section">
      <div class="section-label">Power</div>
      <div class="rows">
        ${row('bus V',    fmtN(d.bus_voltage, 2), 'V')}
        ${row('bus I',    fmtN(d.bus_current, 3), 'A')}
        ${row('elec W',   elecPwr ?? busCalcPwr,   'W')}
        ${row('mech W',   mechPwr ?? '—',          'W')}
      </div>
    </div>

    <div class="section">
      <div class="section-label">Thermal</div>
      <div class="rows">
        ${row('FET',   fetStr, '', tempClass(d.fet_temp))}
        ${row('motor', motStr, '', tempClass(d.motor_temp))}
      </div>
    </div>

    <div class="section">
      <div class="section-label">Errors</div>
      <div class="rows">
        <div class="row"><span class="lbl">axis err</span>${errCell(decodeFlags(d.axis_error, AXIS_ERRORS))}</div>
        <div class="row"><span class="lbl">active</span>${errCell(ctlErrs)}</div>
        <div class="row"><span class="lbl">disarm</span>
          <span class="val ${d.disarm_reason?'err':''}">${d.disarm_reason?'0x'+d.disarm_reason.toString(16).toUpperCase():'—'}</span>
        </div>
      </div>
    </div>
  </div>`;
}

async function fetchData() {
  try {
    const r = await fetch('/nodedata');
    const data = await r.json();
    const sorted = Object.entries(data).sort(([a],[b]) => Number(a)-Number(b));
    document.getElementById('cards').innerHTML =
      sorted.map(([nid, d]) => renderCard(Number(nid), d)).join('');
  } catch(e) {}
}
setInterval(fetchData, 250);
fetchData();
</script>
</body>
</html>"""

# ── HTTP handler ──────────────────────────────────────────────────────────────

def make_handler(stream, fetcher):
    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a): pass

        def do_GET(self):
            if self.path == "/quit":
                stream.stop(); fetcher.stop()
                self.send_response(200); self.end_headers()
                threading.Thread(target=self.server.shutdown, daemon=True).start()
            elif self.path == "/estop":
                stream.estop()
                for nid in ALL_NODES:
                    try:
                        url = f"http://{HOST}:{HTTP_PORT}/odrive_{nid}/estop"
                        req = urllib.request.Request(url, data=b"", method="POST")
                        urllib.request.urlopen(req, timeout=2)
                    except Exception:
                        pass
                self.send_response(200); self.end_headers()
            elif self.path == "/clearerrors":
                stream.clear_errors()
                for nid in ALL_NODES:
                    try:
                        body = json.dumps({"clear_errors": True}).encode()
                        url = f"http://{HOST}:{HTTP_PORT}/odrive_{nid}/command"
                        req = urllib.request.Request(url, data=body, method="POST",
                                                     headers={"Content-Type": "application/json"})
                        urllib.request.urlopen(req, timeout=2)
                    except Exception:
                        pass
                self.send_response(200); self.end_headers()
            elif self.path == "/nodedata":
                data = fetcher.get_all()
                cmds = stream.get_cmds()
                for nid, d in data.items():
                    d["_vel_cmd"] = cmds.get(nid, 0.0)
                    if nid in LEADER_FOLLOWER:
                        d["_role"] = "leader"
                        d["_follow"] = LEADER_FOLLOWER[nid]
                    elif nid in FOLLOWER_LEADER:
                        d["_role"] = "follower"
                        d["_follow"] = FOLLOWER_LEADER[nid]
                    else:
                        d["_role"] = "solo"
                        d["_follow"] = None
                body = json.dumps(data).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", len(body))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(HTML.encode())

        def do_POST(self):
            n    = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(n))
            stream.set(body["lv"], body["rv"])
            self.send_response(200); self.end_headers()

    return Handler

# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print(f"Discovering ODrives on {HOST}:{HTTP_PORT}...")
    try:
        ports = discover_cmd_ports()
    except Exception as e:
        print(f"Discovery failed: {e}"); raise SystemExit(1)

    if not ports:
        print("No ODrive nodes found."); raise SystemExit(1)

    for nid, port in sorted(ports.items()):
        side = "Left" if nid in LEFT_NODES else "Right (flipped)"
        print(f"  odrive_{nid} ({side}) → cmd:{port}")

    fetcher = NodeDataFetcher(list(ports.keys()))
    stream  = DriveStream(ports, fetcher)
    server  = http.server.HTTPServer(("127.0.0.1", 9000), make_handler(stream, fetcher))

    print("\nOpening http://localhost:9000")
    webbrowser.open("http://localhost:9000")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stream.stop(); fetcher.stop()
        print("Stopped.")
