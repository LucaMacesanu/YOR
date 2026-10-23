"""SameHand grip-force experiment: gamepad trigger closes the fingers, D-pad sets tau_max live,
a browser page plots the servo current, and every 200 Hz finger-state sample is saved to CSV.

Start the hand and yor.py first, and nothing else that drives the right gripper
(whole_body_teleop.py / vr_teleop.py would fight over it; joystick.py would also read the pad):

    ~/SameHand-UMI/device/software/run.sh start calib bridge
    python robot/yor.py
    python robot/test_gripper_force.py [--tau-max 0.4] [--squeeze]
    # then open http://<pi-ip>:8092 on the laptop

Gamepad (Logitech F710 in X mode / Xbox layout):
    A               torque on (holds the fingers where they are)
    B               limp
    right trigger   close: 0 = open .. full pull = closed (touching, or full press with squeeze)
    D-pad up/down   tau_max +/- 0.05 (0.05..1.0 of stall)
    X               squeeze on/off
    Y               marker (written to the CSV, drawn on the plot), e.g. "object in hand"
    Back            quit (fingers go limp)

Current (A) is the torque measurement: HLS servos report it directly. `effort` is drive duty,
not torque. The servos cut torque above ~3.2-3.5 A for ~2 s (status bit 3, overcurrent) and the
grip silently lets go: the page shows that band and flags any nonzero status.

CSV: teleop_data/gripper_force/<timestamp>.csv, one row per finger-state sample (~200 Hz).
"""
import argparse
import asyncio
import csv
import json
import os
import queue
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")  # no display needed for pygame's joystick
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

import pygame  # noqa: E402
import zenoh  # noqa: E402
from aiohttp import web  # noqa: E402
from commlink import RPCClient  # noqa: E402

from robot.arm.samehand_gripper import pb  # noqa: E402  (SameHand glove protobufs)

PAD = {"a": 0, "b": 1, "x": 2, "y": 3, "back": 6, "rt_axis": 5}
TAU_STEP = 0.05
TAU_RANGE = (0.05, 1.0)
PAD_HZ = 50.0
GRIP_MIN_CHANGE = 0.01
TRIP_A = (3.2, 3.5)  # overcurrent trip band (SameHand integration.md section 7)
JOINTS = ("j1", "j2")
FIELDS = ["t", "t_us", "trigger", "pos_cmd", "tau_max", "squeeze", "enabled", "torque_on", "marker"] + [
    f"{j}_{k}" for j in JOINTS for k in
    ("position", "velocity", "current", "effort", "temperature", "voltage", "status")]


class Experiment:
    def __init__(self, args):
        self.yor = RPCClient("localhost", args.rpc_port)
        self.lock = threading.Lock()
        # Command side, owned by the pad loop; read by the Zenoh callback for each row.
        self.trigger = 0.0
        self.pos_cmd = None
        self.tau_max = args.tau_max
        self.squeeze = args.squeeze
        self.enabled = False
        self.marker = 0  # increments on Y; the row where it changes is the mark
        self.rows = queue.Queue()
        self.plot = deque(maxlen=int(200 * 30))  # last 30 s for the page
        self.plot_seq = 0
        self.stop = threading.Event()

        out_dir = REPO / "teleop_data" / "gripper_force"
        out_dir.mkdir(parents=True, exist_ok=True)
        self.path = out_dir / f"{datetime.now():%Y-%m-%d--%H-%M-%S}.csv"

        self.z = zenoh.open(zenoh.Config())
        self.sub = self.z.declare_subscriber("samehand/fingers/glove/state", self._on_state)

    # ------------------------------------------------------------------ robot side
    def apply_settings(self):
        self.tau_max = self.yor.set_right_gripper_tau_max(tau_max=self.tau_max)
        self.squeeze = self.yor.set_right_gripper_squeeze(squeeze=self.squeeze)
        if self.tau_max is None:
            raise SystemExit("yor.py has no SameHand gripper (--no-samehand or --no-arms?)")

    def _on_state(self, sample):
        m = pb.State()
        m.ParseFromString(bytes(sample.payload))
        if len(m.actuators) < len(JOINTS):
            return
        with self.lock:
            row = {"t": time.time(), "t_us": m.t_us, "trigger": self.trigger, "pos_cmd": self.pos_cmd,
                   "tau_max": self.tau_max, "squeeze": int(self.squeeze), "enabled": int(self.enabled),
                   "torque_on": int(m.torque_on), "marker": self.marker}
        for j, a in zip(JOINTS, m.actuators):
            for k in ("position", "velocity", "current", "effort", "temperature", "voltage"):
                row[f"{j}_{k}"] = round(float(getattr(a, k)), 4)
            row[f"{j}_status"] = int(a.status)
        self.rows.put(row)

    def writer(self):
        with open(self.path, "w", newline="") as f:
            w = csv.DictWriter(f, FIELDS)
            w.writeheader()
            n = 0
            while not (self.stop.is_set() and self.rows.empty()):
                try:
                    row = self.rows.get(timeout=0.1)
                except queue.Empty:
                    continue
                w.writerow(row)
                n += 1
                if n % 50 == 0:
                    f.flush()
                with self.lock:
                    self.plot.append(row)
                    self.plot_seq += 1

    # ------------------------------------------------------------------ gamepad
    def pad_loop(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() < 1:
            raise SystemExit("No gamepad found")
        js = pygame.joystick.Joystick(0)
        js.init()
        print(f"Gamepad: {js.get_name()}. A torque on, B limp, RT close, D-pad up/down tau_max, "
              f"X squeeze, Y marker, Back quit.")
        prev = {k: 0 for k in ("a", "b", "x", "y", "back")}
        prev_hat = (0, 0)
        sent = None
        period = 1.0 / PAD_HZ
        while not self.stop.is_set():
            t0 = time.monotonic()
            pygame.event.pump()
            btn = {k: js.get_button(PAD[k]) for k in prev}
            pressed = {k: btn[k] and not prev[k] for k in prev}
            prev = btn
            hat = js.get_hat(0) if js.get_numhats() else (0, 0)
            trig = min(1.0, max(0.0, (js.get_axis(PAD["rt_axis"]) + 1.0) / 2.0))

            if pressed["back"]:
                break
            if pressed["a"] and trig > 0.05:
                print("Release the trigger before torque on")
            elif pressed["a"]:
                ok = self.yor.enable_right_gripper()
                with self.lock:
                    self.enabled = bool(ok)
                sent = None
                print("Torque ON" if ok else "Torque on FAILED (bridge running?)")
            if pressed["b"]:
                self.yor.disable_right_gripper()
                with self.lock:
                    self.enabled = False
                print("Limp")
            if hat[1] != prev_hat[1] and hat[1] != 0:
                want = round(min(TAU_RANGE[1], max(TAU_RANGE[0], self.tau_max + hat[1] * TAU_STEP)), 2)
                got = self.yor.set_right_gripper_tau_max(tau_max=want)
                with self.lock:
                    self.tau_max = got
                print(f"tau_max {got:.2f}")
            prev_hat = hat
            if pressed["x"]:
                got = self.yor.set_right_gripper_squeeze(squeeze=not self.squeeze)
                with self.lock:
                    self.squeeze = got
                sent = None
                print(f"squeeze {'ON' if got else 'OFF'}")
            if pressed["y"]:
                with self.lock:
                    self.marker += 1
                print(f"marker {self.marker}")

            pos = 1.0 - trig  # CanGripper convention: 1 open, 0 closed
            if self.enabled and (sent is None or abs(pos - sent) >= GRIP_MIN_CHANGE):
                self.yor.set_right_gripper(gripper_pos=pos)
                sent = pos
            with self.lock:
                self.trigger = trig
                self.pos_cmd = sent if self.enabled else None
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    # ------------------------------------------------------------------ web plot
    async def ws(self, request):
        sock = web.WebSocketResponse()
        await sock.prepare(request)
        with self.lock:
            backlog = list(self.plot)
            seq = self.plot_seq
        await sock.send_str(json.dumps({"rows": backlog, "file": str(self.path.relative_to(REPO)),
                                        "trip": TRIP_A}))

        async def drain():  # reading is what notices the browser's close frame
            async for _ in sock:
                pass
        reader = asyncio.ensure_future(drain())
        while not sock.closed and not reader.done() and not self.stop.is_set():
            await asyncio.sleep(0.05)
            with self.lock:
                n = min(self.plot_seq - seq, len(self.plot))
                new = list(self.plot)[-n:] if n > 0 else []
                seq = self.plot_seq
            if new:
                try:
                    await sock.send_str(json.dumps({"rows": new}))
                except ConnectionError:  # browser went away mid-send
                    break
        reader.cancel()
        return sock

    def web_thread(self, port):
        async def index(_):
            return web.Response(text=PAGE, content_type="text/html")

        app = web.Application()
        app.add_routes([web.get("/", index), web.get("/ws", self.ws)])
        loop = asyncio.new_event_loop()
        runner = web.AppRunner(app)
        loop.run_until_complete(runner.setup())
        loop.run_until_complete(web.TCPSite(runner, "0.0.0.0", port).start())
        ips = subprocess.run(["hostname", "-I"], capture_output=True, text=True).stdout.split() or ["<pi-ip>"]
        print("Live plot: " + "  ".join(f"http://{ip}:{port}" for ip in ips if ":" not in ip))
        while not self.stop.is_set():
            loop.run_until_complete(asyncio.sleep(0.2))
        loop.run_until_complete(runner.cleanup())

    def close(self):
        try:
            self.yor.disable_right_gripper()
        except Exception as e:  # noqa: BLE001
            print(f"WARNING: could not make the fingers limp: {e}")
        self.stop.set()
        self.sub.undeclare()
        self.z.close()


PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><title>Grip force</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#fff;--fg:#1d1d1f;--mut:#6e6e73;--grid:#e5e5ea;--j1:#2a6fdb;--j2:#d9480f;--tau:#2b8a3e;
--trig:#868e96;--trip:rgba(224,49,49,.13);--bad:#e03131}
@media (prefers-color-scheme:dark){:root{--bg:#161618;--fg:#f2f2f7;--mut:#98989d;--grid:#2c2c2e;
--j1:#6ea8fe;--j2:#ff8a4c;--tau:#69db7c;--trig:#adb5bd;--trip:rgba(255,107,107,.18);--bad:#ff6b6b}}
body{margin:0;padding:16px;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif}
h1{font-size:17px;margin:0 0 8px}.row{display:flex;flex-wrap:wrap;gap:8px 20px;margin:0 0 10px}
.k{color:var(--mut)}.v{font-variant-numeric:tabular-nums;font-weight:600}.bad{color:var(--bad)}
canvas{width:100%;height:300px;display:block;margin-bottom:12px}
.lg span{margin-right:14px}.sw{display:inline-block;width:10px;height:3px;vertical-align:middle;margin-right:4px}
</style></head><body>
<h1>SameHand grip force</h1>
<div class="row" id="st"></div>
<div class="lg"><span><i class="sw" style="background:var(--j1)"></i>j1 current</span>
<span><i class="sw" style="background:var(--j2)"></i>j2 current</span>
<span><i class="sw" style="background:var(--trip);height:10px"></i>overcurrent trip band</span></div>
<canvas id="c1"></canvas>
<div class="lg"><span><i class="sw" style="background:var(--tau)"></i>tau_max</span>
<span><i class="sw" style="background:var(--trig)"></i>trigger</span>
<span><i class="sw" style="background:var(--j1);opacity:.5"></i>|effort| j1</span>
<span><i class="sw" style="background:var(--j2);opacity:.5"></i>|effort| j2</span></div>
<canvas id="c2"></canvas>
<div class="k" id="file"></div>
<script>
const WIN=20, rows=[]; let trip=[3.2,3.5];
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const ws=new WebSocket(`ws://${location.host}/ws`);
ws.onmessage=e=>{const d=JSON.parse(e.data); if(d.trip) trip=d.trip;
 if(d.file) document.getElementById('file').textContent='Saving to '+d.file;
 for(const r of d.rows) rows.push(r); const t0=rows.length?rows[rows.length-1].t-30:0;
 while(rows.length&&rows[0].t<t0) rows.shift();};
ws.onclose=()=>document.getElementById('file').textContent+='  (disconnected)';
function plot(id,ymax,series,band){
 const c=document.getElementById(id),dpr=devicePixelRatio||1,W=c.clientWidth,H=c.clientHeight;
 c.width=W*dpr;c.height=H*dpr;const g=c.getContext('2d');g.scale(dpr,dpr);
 const L=40,R=8,T=8,B=20,w=W-L-R,h=H-T-B; if(!rows.length) return;
 const tn=rows[rows.length-1].t,x=t=>L+w*(1-(tn-t)/WIN),y=v=>T+h*(1-v/ymax);
 g.font='11px system-ui';g.fillStyle=css('--mut');g.strokeStyle=css('--grid');g.lineWidth=1;
 for(let i=0;i<=4;i++){const v=ymax*i/4;g.beginPath();g.moveTo(L,y(v));g.lineTo(L+w,y(v));g.stroke();
  g.fillText(v.toFixed(ymax>2?1:2),4,y(v)+4);}
 for(let s=0;s<=WIN;s+=5){g.fillText(`-${WIN-s}s`,x(tn-WIN+s)-12,H-4);}
 if(band){g.fillStyle=css('--trip');g.fillRect(L,y(band[1]),w,y(band[0])-y(band[1]));}
 let lastM=null;g.strokeStyle=css('--mut');g.setLineDash([3,3]);
 for(const r of rows){if(r.t<tn-WIN)continue;if(lastM!==null&&r.marker!==lastM){g.beginPath();
  g.moveTo(x(r.t),T);g.lineTo(x(r.t),T+h);g.stroke();}lastM=r.marker;}
 g.setLineDash([]);
 for(const [col,f,a] of series){g.strokeStyle=css(col);g.globalAlpha=a||1;g.lineWidth=1.6;g.beginPath();
  let on=false;for(const r of rows){if(r.t<tn-WIN)continue;const v=f(r);
  if(v==null){on=false;continue;}const X=x(r.t),Y=y(Math.min(v,ymax));on?g.lineTo(X,Y):g.moveTo(X,Y);on=true;}
  g.stroke();g.globalAlpha=1;}
 g.fillStyle=css('--bad');for(const r of rows){if(r.t<tn-WIN)continue;
  if(r.j1_status||r.j2_status) g.fillRect(x(r.t)-1,T,2,6);}
}
function tick(){
 const r=rows[rows.length-1];
 if(r){const peak=Math.max(...rows.filter(q=>q.t>r.t-WIN).map(q=>Math.max(Math.abs(q.j1_current),Math.abs(q.j2_current))));
  const f=r.j1_status||r.j2_status;
  const item=(k,v,c)=>`<span><span class="k">${k}</span> <span class="v ${c||''}">${v}</span></span>`;
  document.getElementById('st').innerHTML=[item('tau_max',r.tau_max.toFixed(2)),
  item('squeeze',r.squeeze?'on':'off'),item('torque',r.torque_on?'on':'off'),
  item('trigger',r.trigger.toFixed(2)),
  item('current',`${Math.abs(r.j1_current).toFixed(2)} / ${Math.abs(r.j2_current).toFixed(2)} A`),
  item(`peak ${WIN}s`,peak.toFixed(2)+' A',peak>=trip[0]?'bad':''),
  item('temp',`${r.j1_temperature} / ${r.j2_temperature} °C`),item('supply',r.j1_voltage.toFixed(1)+' V'),
  item('status',f?`${r.j1_status} / ${r.j2_status} TRIPPED`:'ok',f?'bad':'')].join('');}
 const cmax=Math.max(4,...rows.map(q=>Math.max(Math.abs(q.j1_current),Math.abs(q.j2_current))));
 plot('c1',Math.ceil(cmax),[['--j1',q=>Math.abs(q.j1_current)],['--j2',q=>Math.abs(q.j2_current)]],trip);
 plot('c2',1,[['--tau',q=>q.tau_max],['--trig',q=>q.trigger],['--j1',q=>Math.abs(q.j1_effort),.5],
  ['--j2',q=>Math.abs(q.j2_effort),.5]]);
 requestAnimationFrame(tick);}
requestAnimationFrame(tick);
</script></body></html>"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tau-max", type=float, default=0.4, help="starting tau_max (0..1 of stall)")
    ap.add_argument("--squeeze", action="store_true", help="start with squeeze on")
    ap.add_argument("--port", type=int, default=8092, help="live plot web port")
    ap.add_argument("--rpc-port", type=int, default=5557)
    args = ap.parse_args()

    exp = Experiment(args)
    exp.apply_settings()
    print(f"tau_max {exp.tau_max:.2f}, squeeze {'on' if exp.squeeze else 'off'}; saving to {exp.path}")
    threads = [threading.Thread(target=exp.writer, daemon=True),
               threading.Thread(target=exp.web_thread, args=(args.port,), daemon=True)]
    for t in threads:
        t.start()
    try:
        exp.pad_loop()
    except KeyboardInterrupt:
        pass
    finally:
        exp.close()
        threads[0].join(timeout=2.0)
        print(f"Saved {exp.path}")


if __name__ == "__main__":
    main()
