"""
Web viewer for the arms' end-effector frame: where VR teleop rotates the
gripper about today (the MJCF `*_arm_ee` site) and where a forward-shifted
control point would sit.

    python robot/viz_tcp.py            # mirrors the live arms if yor.py is running
    python robot/viz_tcp.py --no-live  # model only; pose the arm with the sliders
    # open http://<pi-ip>:8080 in a browser

Shown per arm:
  - the current EE site: small axes (red x, green y, blue z) + yellow dot
  - the candidate control point: magenta ball + axes, joined by a line
The panel sets the candidate's direction (an axis of the EE site frame) and
distance, and prints the offset vector to put in the teleop config.
"""
import argparse
import socket
import threading
import time
from pathlib import Path

import mujoco
import numpy as np
import viser

MJCF = Path(__file__).parent / "yor-description/nero-welded-base-and-lift.mjcf"
SIDES = ("left", "right")
AXES = {"+y": (0, 1, 0), "-y": (0, -1, 0), "+z": (0, 0, 1), "-z": (0, 0, -1), "+x": (1, 0, 0), "-x": (-1, 0, 0)}


def mat_to_wxyz(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).reshape(9))
    return q


def lan_ip():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except OSError:
        return "localhost"


class LiveJoints:
    """Polls yor.py for joint angles; None while it is unreachable."""

    def __init__(self, host, port):
        self.host, self.port = host, port
        self.q = {s: None for s in SIDES}
        self.ok = False
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        from commlink import RPCClient

        rpc = None
        while True:
            try:
                if rpc is None:
                    rpc = RPCClient(host=self.host, port=self.port)
                for s in SIDES:
                    self.q[s] = np.asarray(getattr(rpc, f"get_{s}_joint_positions")(), dtype=float)
                self.ok = True
                time.sleep(0.1)
            except Exception:  # noqa: BLE001 -- yor.py down or arm missing: retry
                self.ok, rpc = False, None
                time.sleep(2.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-live", action="store_true", help="don't read joint angles from yor.py")
    ap.add_argument("--rpc-host", default="localhost")
    ap.add_argument("--rpc-port", type=int, default=5557)
    ap.add_argument("--port", type=int, default=8080, help="web viewer port")
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(MJCF.as_posix())
    data = mujoco.MjData(model)
    qadr = {s: [model.jnt_qposadr[model.joint(f"{s}_arm_joint{i}").id] for i in range(1, 8)] for s in SIDES}
    jrange = {s: [model.jnt_range[model.joint(f"{s}_arm_joint{i}").id] for i in range(1, 8)] for s in SIDES}
    site = {s: model.site(f"{s}_arm_ee").id for s in SIDES}

    server = viser.ViserServer(host="0.0.0.0", port=args.port)
    server.scene.set_up_direction("+z")

    # Visual meshes only (collision geoms are group 3).
    geoms = []
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or model.geom_group[g] == 3:
            continue
        mid = model.geom_dataid[g]
        v0, nv = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        f0, nf = model.mesh_faceadr[mid], model.mesh_facenum[mid]
        rgba = model.mat_rgba[model.geom_matid[g]] if model.geom_matid[g] >= 0 else model.geom_rgba[g]
        h = server.scene.add_mesh_simple(
            f"/robot/geom{g}", model.mesh_vert[v0:v0 + nv].copy(), model.mesh_face[f0:f0 + nf].copy(),
            color=tuple(int(255 * c) for c in rgba[:3]), opacity=0.55,
        )
        geoms.append((g, h))

    markers = {}
    for s in SIDES:
        markers[s] = {
            "ee": server.scene.add_frame(f"/{s}/ee", axes_length=0.06, axes_radius=0.003, origin_radius=0.008),
            "tcp": server.scene.add_frame(f"/{s}/tcp", axes_length=0.05, axes_radius=0.003, origin_radius=0.001),
            "ball": server.scene.add_icosphere(f"/{s}/tcp_ball", radius=0.015, color=(255, 0, 200)),
        }

    gui = server.gui
    status = gui.add_markdown("starting...")
    show = gui.add_dropdown("Arms", ("both", "right", "left"), initial_value="right")
    axis = gui.add_dropdown("Forward axis (EE site frame)", tuple(AXES), initial_value="+y")
    dist = gui.add_slider("Shift forward (m)", min=0.0, max=0.40, step=0.005, initial_value=0.25)
    readout = gui.add_markdown("")
    live_cb = gui.add_checkbox("Mirror live robot", initial_value=not args.no_live)
    with gui.add_folder("Manual pose (when not mirroring)"):
        pose_side = gui.add_dropdown("Arm", SIDES, initial_value="right")
        sliders = [gui.add_slider(f"joint{i + 1}", min=float(jrange["right"][i][0]), max=float(jrange["right"][i][1]),
                                  step=0.01, initial_value=0.0) for i in range(7)]
    manual_q = {s: np.zeros(7) for s in SIDES}

    @pose_side.on_update
    def _(_):
        for i, sl in enumerate(sliders):
            sl.value = float(manual_q[pose_side.value][i])

    for i, sl in enumerate(sliders):
        @sl.on_update
        def _(_, i=i):
            manual_q[pose_side.value][i] = sliders[i].value

    live = LiveJoints(args.rpc_host, args.rpc_port) if not args.no_live else None
    print(flush=True); print(f"[viz_tcp] open http://{lan_ip()}:{args.port}")

    while True:
        mirroring = live_cb.value and live is not None and live.ok
        for s in SIDES:
            q = live.q[s] if mirroring and live.q[s] is not None else manual_q[s]
            data.qpos[qadr[s]] = q
        mujoco.mj_kinematics(model, data)

        for g, h in geoms:
            h.position = data.geom_xpos[g].copy()
            h.wxyz = mat_to_wxyz(data.geom_xmat[g])

        off = dist.value * np.array(AXES[axis.value], dtype=float)
        for s in SIDES:
            vis = show.value in ("both", s)
            p, R = data.site_xpos[site[s]].copy(), data.site_xmat[site[s]].reshape(3, 3)
            wxyz = mat_to_wxyz(R)
            tcp = p + R @ off
            m = markers[s]
            m["ee"].position, m["ee"].wxyz = p, wxyz
            m["tcp"].position, m["tcp"].wxyz = tcp, wxyz
            m["ball"].position = tcp
            for h in m.values():
                h.visible = vis
            server.scene.add_line_segments(f"/{s}/link", np.array([[p, tcp]]), colors=(255, 0, 200),
                                           thickness=0.004, visible=vis)

        if live is None:
            src = "model only (--no-live)"
        elif not live_cb.value:
            src = "manual pose"
        else:
            src = "**live robot**" if live.ok else f"waiting for yor.py at {args.rpc_host}:{args.rpc_port} (manual pose)"
        status.content = f"Pose: {src}"
        readout.content = (f"Yellow dot = current pivot (`*_arm_ee` site).  \nMagenta = candidate pivot.  \n"
                           f"Offset in EE frame: `({off[0]:+.3f}, {off[1]:+.3f}, {off[2]:+.3f})` m")
        time.sleep(0.1)


if __name__ == "__main__":
    main()
