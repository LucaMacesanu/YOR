"""
VR teleop of the MuJoCo sim in one command. Starts, on this machine:
  1. robot/yor_mujoco.py            the sim, serving yor.py's RPC on :5557 and
                                    publishing its head ZED view on tcp://127.0.0.1:5560
  2. vr-teleop server.py --video-only   streams that view to the headset on :8444
  3. vr-teleop yor_bridge.py         the headset page + whole-body teleop on :8443

    python robot/sim_vr_teleop.py [--ip <this machine's LAN IP>] [--headless]
    # headset: open https://<ip>:8444/ (accept the cert), then https://<ip>:8443/ -> Enter VR

Ctrl-C stops all three. Setup and controls: docs/SIM_VR_TELEOP.md.
"""
import argparse
import socket
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
RPC_PORT = 5557
VIDEO_PORT = 8444
PAGE_PORT = 8443


def lan_ip():
    """IP of the interface that routes off this machine (no packet is sent)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect(("10.255.255.255", 1))
            return s.getsockname()[0]
        except OSError:
            return "127.0.0.1"


def cert_ips(crt):
    from cryptography import x509

    cert = x509.load_pem_x509_certificate(crt.read_bytes())
    try:
        san = cert.extensions.get_extension_for_class(x509.SubjectAlternativeName).value
    except x509.ExtensionNotFound:
        return set()
    return {str(ip) for ip in san.get_values_for_type(x509.IPAddress)}


def wait_for_port(port, proc, name, timeout_s=60.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if proc.poll() is not None:
            raise SystemExit(f"{name} exited during startup (code {proc.returncode})")
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.5):
                return
        except OSError:
            time.sleep(0.5)
    raise SystemExit(f"{name} did not open port {port} within {timeout_s:.0f} s")


def stop(proc):
    if proc.poll() is not None:
        return
    if sys.platform == "win32":
        # venv python.exe on Windows is a launcher with a child process: kill the tree
        subprocess.run(["taskkill", "/T", "/F", "/PID", str(proc.pid)], capture_output=True)
    else:
        proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ip", default=None, help="LAN IP the headset uses to reach this machine (default: detected)")
    parser.add_argument("--headless", action="store_true", help="no MuJoCo viewer window")
    parser.add_argument("--gripper", choices=("custom", "agx"), default="custom",
                        help="custom: the new grippers; agx: the stock AgileX grippers")
    parser.add_argument("--vr-teleop", default=str(REPO / "vr-teleop"), help="vr-teleop checkout (default: the submodule)")
    args = parser.parse_args()

    ip = args.ip or lan_ip()
    vr = Path(args.vr_teleop).resolve()
    if not (vr / "server" / "yor_bridge.py").exists():
        raise SystemExit(f"No vr-teleop at {vr}. Run: git submodule update --init vr-teleop")

    crt = vr / "certs" / "server.crt"
    if not crt.exists() or ip not in cert_ips(crt):
        print(f"[sim_vr_teleop] Making a TLS cert for {ip}")
        subprocess.run([sys.executable, str(vr / "server" / "gen_cert.py"), ip], check=True)

    py = [sys.executable, "-u"]
    procs = []
    try:
        sim = subprocess.Popen(py + [str(REPO / "robot" / "yor_mujoco.py"), "--gripper", args.gripper]
                               + (["--headless"] if args.headless else []),
                               cwd=REPO)
        procs.append(sim)
        wait_for_port(RPC_PORT, sim, "yor_mujoco.py")
        video = subprocess.Popen(py + ["server/server.py", "--video-only", "--port", str(VIDEO_PORT),
                                       "--camera-config", "camera_config.sim.json"], cwd=vr)
        procs.append(video)
        wait_for_port(VIDEO_PORT, video, "video server")
        bridge = subprocess.Popen(py + ["server/yor_bridge.py", "--rpc-host", "localhost", "--port", str(PAGE_PORT),
                                        "--video", f"{ip}:{VIDEO_PORT}"], cwd=vr)
        procs.append(bridge)
        wait_for_port(PAGE_PORT, bridge, "bridge")

        print(f"\n[sim_vr_teleop] Ready. On the headset (same network as this machine):\n"
              f"  1. https://{ip}:{VIDEO_PORT}/  -> accept the certificate (video)\n"
              f"  2. https://{ip}:{PAGE_PORT}/  -> accept the certificate -> Enter VR\n"
              f"  Teleop starts OFF: tap A + X. Ctrl-C here stops everything.\n", flush=True)
        while all(p.poll() is None for p in procs):
            time.sleep(0.5)
        print("[sim_vr_teleop] A process exited -- stopping the rest.")
    except KeyboardInterrupt:
        pass
    finally:
        for p in reversed(procs):
            stop(p)


if __name__ == "__main__":
    main()
