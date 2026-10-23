# VR teleop of the MuJoCo sim

Drive a simulated YOR from a Quest headset with the same page, controller logic
and bridge as the real robot. Runs on one machine (Windows, Linux or macOS); no
robot, Pi or Orin needed.

```
Quest headset (WebXR page)
 ├─ wss://<pc>:8443/ws  poses up, HUD down -> vr-teleop/server/yor_bridge.py ─┐ commlink RPC :5557
 └─ wss://<pc>:8444/ws  ZED video down     <- vr-teleop server.py --video-only │
                                                 ▲ ZMQ tcp://127.0.0.1:5560  ▼
                                              robot/yor_mujoco.py (MuJoCo sim)
```

- `robot/yor_mujoco.py` stands in for `robot/yor.py`: it serves the same RPC
  calls on the same port, so the bridge runs unchanged (the calls are listed in
  `vr-teleop/docs/YOR_RPC_INTERFACE.md`). It also renders the head ZED pair
  and publishes it locally.
- The video server streams those frames to the headset in stereo, the way the
  Orin streams the real ZED (`"zmq"` source in `vr-teleop/server/cameras.py`,
  config `vr-teleop/camera_config.sim.json`).
- `robot/sim_vr_teleop.py` starts all three and stops them together.

## Setup (once)

1. **Code.** The `sim` branch of this repo, with the `vr-teleop` submodule:
   ```bash
   git clone -b sim --recursive <YOR repo url>
   # or, in an existing clone:
   git checkout sim && git submodule update --init vr-teleop
   ```
2. **Python 3.10+ environment** with only the sim's dependencies (the hardware
   drivers are not needed):
   ```bash
   python -m venv .venv-sim
   # Windows: .venv-sim\Scripts\activate    Linux/macOS: source .venv-sim/bin/activate
   pip install "mujoco>=3.2.7" mink "qpsolvers[daqp]" "commlink>=0.2.1" "loop-rate-limiters>=1.1.0" \
               pyzmq numpy opencv-python aiohttp cryptography
   ```
3. **Laptops with two GPUs (Windows):** the model has ~850k triangles, which
   renders at ~3 fps on Intel integrated graphics and 200+ fps on an NVIDIA
   card. Settings -> Display -> Graphics settings -> add the `python.exe` the
   venv was created from (the venv's own `python.exe` is a launcher for it;
   `python -c "import sys; print(sys._base_executable)"` prints it) -> Options ->
   High performance.
4. **Firewall:** the headset must reach this machine on TCP 8443 and 8444.
   Windows asks the first time; allow Python on private networks, or (admin
   PowerShell):
   ```powershell
   New-NetFirewallRule -DisplayName "VR Teleop sim" -Direction Inbound -Protocol TCP -LocalPort 8443,8444 -Action Allow
   ```
   Linux with ufw: `sudo ufw allow 8443,8444/tcp`.

## Run

```bash
python robot/sim_vr_teleop.py            # add --headless for no viewer window
python robot/sim_vr_teleop.py --gripper agx   # same robot with the stock AgileX grippers
```

It detects this machine's LAN IP (override with `--ip <ip>`), makes a
self-signed TLS cert for it if needed, starts the sim, video server and bridge,
and prints:

```
[sim_vr_teleop] Ready. On the headset (same network as this machine):
  1. https://<ip>:8444/  -> accept the certificate (video)
  2. https://<ip>:8443/  -> accept the certificate -> Enter VR
```

On the headset (Quest browser, same Wi-Fi as the PC), do both steps in order.
Step 1 is needed once per certificate: without it the video socket fails
silently and the HUD shows NO VIDEO. Ctrl-C in the terminal stops everything.

The three parts can also be run by hand, each in its own terminal:

```bash
python robot/yor_mujoco.py                                   # sim + ZED publisher
cd vr-teleop
python server/gen_cert.py <ip>                               # once per IP
python server/server.py --video-only --port 8444 --camera-config camera_config.sim.json
python server/yor_bridge.py --rpc-host localhost --video <ip>:8444
```

Don't use `robot/whole_body_teleop.py` for the sim: it applies the real neck's
calibration file and points the video at the Orin.

## Controls

Teleop starts **off**.

| Input | Action |
|---|---|
| A + X (one per hand), tap | teleop on / off |
| A + X, hold 1.5 s | home both arms + neck, teleop off |
| grip (hold) | move that arm with your hand; release and it holds |
| right trigger | close the right gripper |
| B + Y | base mode on / off; left stick drives, right stick turns |
| A + Y / X + B (hold) | lift down / up |
| head | neck pan / tilt; left stick click re-centres |
| right stick up / down, click | adjust / reset the stereo IPD (not in base mode) |

The MuJoCo viewer on the PC shows the robot from outside; the bridge's terminal
logs button presses and what it sends.

## How the sim matches the robot

- **Arms:** the robot's IK model (`nero-welded-base-and-lift.mjcf`) and
  `SingleArmIK`, the same home pose, re-sync to measured joints before each
  solve, and hold after 10 failed solves, like `ArmNode`. EE poses are
  base-frame and lift-independent, as on the robot.
- **Base:** velocity in the robot frame, held until the next command (like
  `BaseController`), acceleration-limited, driven through the model's 4 swerve
  modules. The model's drive actuators reach about 80% of the commanded wheel
  speed, so the sim base is a bit slower than commanded.
- **Lift:** up / down / stop at a fixed 0.05 m/s (the real lift speed is not
  measured). The sim's travel is 0..1.2 m; the HUD bar assumes 0..0.5 m.
- **Neck:** robot convention (degrees, + pan = head right, + tilt = head down),
  0 = straight ahead and level. No calibration file.
- **Grippers:** 1 = open, 0 = closed. Default: the custom grippers on both
  arms (finger travel is a placeholder). `--gripper agx` uses
  `scene-full-agx.mjcf`, the stock AgileX parallel jaw on both arms (70 mm
  stroke, the Piper gripper model), seated where the custom gripper sits; its
  exact mount offset and jaw orientation on the real Nero are not measured. That
  model is generated from `yor-full.mjcf` by
  `robot/yor-description/make_agx_variant.py`; re-run it after regenerating
  `yor-full.mjcf`.
- **Video:** both eyes rendered at 960x540 from the model's ZED cameras (120 mm
  baseline, 70 deg vertical FOV) with matching `stereo_info`, ~22 fps on Windows.
- **Not simulated:** navigation calls (`follow_path`, `move_to`, ...), wrist
  cameras, the episode recorder. The world is just a floor.

## Checks

```bash
python robot/test_yor_mujoco.py          # drives the sim directly, headless
python robot/test_yor_mujoco.py --gripper agx   # same, AgileX-gripper model
python robot/test_yor_mujoco.py --rpc    # same checks against a running yor_mujoco.py
python vr-teleop/tools/video_probe.py <ip>:8444   # video fps / frame size, like the headset sees it
```

## Troubleshooting

- **Headset can't open the page:** firewall (see setup), or the router isolates
  Wi-Fi clients. Check from another machine: `curl -k https://<ip>:8443/`.
- **NO VIDEO in the HUD:** open `https://<ip>:8444/` on the headset and accept
  the cert. Check `https://<ip>:8444/health`: `head_stereo` should show fps near 20-30.
- **Video at a few fps:** rendering on integrated graphics (setup step 3).
- **Cert warning every time / wrong IP:** the cert is made for one IP; the
  launcher regenerates it when the IP changes (`--ip` to force one).
- **`ROBOT LINK STALLED`:** the sim isn't running or crashed; see its output.
