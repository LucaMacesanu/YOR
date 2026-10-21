# YOR Robot Repository

This repository contains the codebase for the YOR robot, supporting both high-fidelity MuJoCo simulation and physical robot control via CAN bus.

## 1. Installation & Setup

### 1.1 Prerequisites
- **OS:** Linux (Ubuntu 22.04+ recommended) for physical robot control. macOS/Windows supported for Simulation only.
- **Python:** Version 3.10 or higher.
- **Hardware (Physical only):**
  - CANable or compatible USB-to-CAN adapter.
  - AgileX Nero arm(s).
  - SparkFlex/SparkMax motor controllers.

### 1.2 Cloning the Repository
This repository uses submodules. Clone recursively:
```bash
git clone -b nero --recursive https://github.com/YOR-robot/YOR.git
cd YOR
```
If you already cloned without submodules:
```bash
git submodule update --init --recursive
```

### 1.3 Environment Setup (Recommended: Conda)

For **Linux systems with physical robot support**, we recommend using conda to manage dependencies, as it provides better control over C++ library versions.

**1. Create Conda Environment:**
```bash
conda create -n yor-nero python=3.10 -y
conda activate yor-nero
```

**2. Install System Dependencies:**
```bash
conda install -y -c conda-forge pinocchio spdlog catch2 boost pybind11 gxx cxx-compiler
```

**3. Install Build Tools:**
```bash
pip install scikit-build-core cmake ruckig
```

**4. Install Hardware Drivers (Linux Only):**

Install `sparkcan_py`:
```bash
cd sparkcan_py
pip install .
cd ..
```

Install `nerolib` into the active environment:
```bash
cd nerolib
bash install.sh
cd ..
```
*(This will automatically install dependencies into your active `yor-nero` environment.)*

**5. Install Main Robot Package:**
```bash
pip install -e .
```

### 1.4 Environment Setup (Alternative: `uv`)

For **simulation-only** or **macOS/Windows** setups, `uv` provides a fast, lightweight alternative.

**1. Install `uv` (if not installed):**
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

**2. Sync Environment:**
```bash
uv sync
```
This will automatically create a virtual environment (`.venv`), install the correct Python version, and sync all dependencies from `uv.lock`.

**3. Activate Environment:**
```bash
source .venv/bin/activate
```

> [!NOTE]
> The `uv` method installs only simulation dependencies. Physical robot drivers (`sparkcan_py` and `nerolib`) must be installed separately following section 1.3 steps 4-5.

### 1.5 Verify Installation

Test that all packages import correctly:
```bash
python -c "import robot; print('Robot package installed successfully')"
```

For physical robot setup, also verify:
```bash
python -c "import nerolib; import sparkcan_py; print('Hardware drivers installed successfully')"
```

### 1.6 Navigation & Mapping Setup (Optional)

For dynamic mapping and navigation (e.g., using `robot/zed_pub_node.py` or `robot/nav/`), additional dependencies are required.

**1. Install PyTorch (Linux Conda Recommended):**
```bash
conda install -y pytorch torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia
```
*(Adjust the CUDA version to match your system based on [pytorch.org](https://pytorch.org)).*

**2. Install Navigation Python Packages:**
You can install all necessary python dependencies using the `nav` optional group:
```bash
pip install -e ".[nav]"
```

**3. Install ZED SDK (for physical cameras):**
Download and install the [ZED SDK for Linux](https://www.stereolabs.com/developers/release) (matching your CUDA version), then install its Python wrapper:
```bash
wget -O ZED_SDK_Linux.run https://download.stereolabs.com/zedsdk/4.1/cu118/ubuntu22
chmod +x ZED_SDK_Linux.run
./ZED_SDK_Linux.run -- silent skip_tools
python -m pyzed.sl.get_python_api
```

---

## 2. Running Simulation (Works on macOS/Linux/Windows)

The simulation uses MuJoCo and does not require hardware drivers.

**1. Launch Simulation:**
```bash
python robot/yor_mujoco.py
```
This will launch a passive MuJoCo viewer.

**2. Control the Robot:**
In a separate terminal (with the same environment activated), run a teleop script:
```bash
python robot/teleop/telestick.py
```

`yor_mujoco.py` serves the same RPC calls as `yor.py` on port 5557, so the robot's
teleop clients work against it. Checks: `python robot/test_yor_mujoco.py`.

**VR teleop of the sim (Quest headset):** `python robot/sim_vr_teleop.py` starts the
sim, its stereo ZED video and the VR bridge in one go. Setup, controls and
troubleshooting: [docs/SIM_VR_TELEOP.md](docs/SIM_VR_TELEOP.md).

---

## 3. Running Physical Robot (Linux Only)

**1. Automated Setup (Recommended):**

Run the following script to set up the CAN interface, launch the robot driver, and start the joystick controller in a tmux session:
```bash
./create_windows.sh
```

**2. Manual Setup:**

If you prefer to run components individually:

- **Hardware Setup:**
  Connect CAN adapter/motors and configure interfaces:
  ```bash
  ./extra/setup.sh
  ```

- **Launch Robot Driver:**
  ```bash
  python robot/yor.py
  ```

- **Control the Robot:**
  In a separate terminal:
  ```bash
  python robot/teleop/joystick.py
  ```

---

## 4. Teleoperation Options

Teleoperation scripts communicate with either Simulation or Physical Robot via `commlink` (ZeroMQ). Run these in a separate terminal while the robot/sim is running.

**Keyboard / Test Controls:**
```bash
python robot/teleop/telestick.py
```

**Joystick (Gamepad):**
```bash
python robot/teleop/joystick.py
```

**Oculus / VR Teleop:**
Requires Oculus controller connected via Bluetooth or link.
```bash
# Single arm + base
python robot/teleop/oculus_teleop.py

# Whole body (Base + Arm)
python robot/teleop/oculus_wb_teleop.py

# Bimanual
python robot/teleop/oculus_bimanual_teleop.py
```

---

## 5. Policy Deployment

This section describes how to run learned policy inference on a remote GPU server (e.g., Maxwell) and stream actions to the physical robot over an SSH tunnel.

### Architecture

```
Maxwell (GPU server)                    Pi (robot)
─────────────────────                   ──────────────────────
inference server                        joystick_policy_teleop.py
  publishes actions ──[SSH -L 5570]──▶  subscribes on localhost:5570
  subscribes to obs ◀──[SSH -R 5568]──  publishes joint states on :5568
```

| Port | Direction | Purpose |
|------|-----------|---------|
| 5557 | local | Robot RPC server (ZMQ) |
| 5568 | robot → Maxwell | Joint state observations (ZMQ pub) |
| 5570 | Maxwell → robot | Policy actions (ZMQ pub) |

### Step 1 — Open SSH tunnel (run on the Pi)

```bash
ssh -R 5568:localhost:5568 -L 5570:localhost:5570 lim2045@maxwell.poly.edu
```

- `-R 5568:localhost:5568` — forwards Maxwell:5568 to Pi:5568, so the inference server can subscribe to the robot's joint states
- `-L 5570:localhost:5570` — forwards Pi:localhost:5570 to Maxwell:5570, so the robot can subscribe to actions from the inference server

Keep this terminal open for the duration of the session.

### Step 2 — Start the robot driver (Pi)

```bash
python robot/yor.py
```

### Step 3 — Start the inference server (Maxwell)

Start your inference server on Maxwell. It should:
- Subscribe to joint state observations on port 5568 (topic `teleop/episode`)
- Publish actions on port 5570 (topic `inference/right_delta_joints`)

Action payload format:
```python
{
    "timestamp": float,                  # Unix time
    "right_delta_joints": np.ndarray,    # shape (8,): [Δj0..Δj6, gripper_absolute]
    "infer_ms": float,                   # inference latency in ms
}
```

The first 7 elements are **delta** joint angles (added to current position); the 8th is an **absolute** gripper target (0.0 open → 1.0 closed).

### Step 4 — Start joystick policy teleop (Pi)

```bash
python robot/teleop/joystick_policy_teleop.py
```

Optional flag:
```bash
python robot/teleop/joystick_policy_teleop.py --policy-port 5570
```

**Controls (Xbox/Logitech layout):**

| Button | Action |
|--------|--------|
| Start | Enable control loop |
| Back | Disable control loop |
| A | Toggle policy rollout ON / OFF |
| X | Home left arm |
| B | Home right arm |

### Expected output

On the Pi you should see:
```
[Policy] Connected to inference server on localhost:5570
[JoystickPolicy] Ready.
```

Once you press **Start** then **A** to enable policy:
```
[Action] age=0.012s  ΔR=[0.001, -0.003, ...]  Rg=0.00
```

If `age` grows steadily, the inference server has stopped sending. If no `[Action]` lines appear, check that the SSH tunnel is active and the inference server is running on Maxwell.

### Troubleshooting

**`[Policy] Connection failed ... retrying`** — the SSH tunnel is not active or the inference server is not running on Maxwell.

**`Address already in use` on port 5568 or 5570** — a previous session is still running. Find and kill it:
```bash
lsof -i :5568 -i :5570
kill <pid>
```

**Actions arrive but arm does not move** — ensure the robot driver (`yor.py`) is running and the control loop is enabled (press **Start** on the joystick).

---

## 6. Troubleshooting

**"ModuleNotFoundError: No module named 'sparkcan_py'"**
- You are likely on macOS/Windows or skipped the hardware driver installation. This is expected if you are only running simulation. Physical robot control (`robot/yor.py`) will not work without this.

**"ImportError: ... linux/can.h not found"**
- You are trying to install hardware drivers on a non-Linux OS. Use the simulation (`robot/yor_mujoco.py`) instead.

**"CMake Error: Could NOT find Python3 (missing: Python3_NumPy_INCLUDE_DIRS)"**
- When installing `nerolib`, use `bash install.sh` from the `nerolib/` directory
- Ensure your conda environment is activated before running the install script

**"undefined reference to GLIBCXX_3.4.31"**
- Install the conda C++ compiler: `conda install -y -c conda-forge gxx cxx-compiler`
- This ensures the C++ standard library version matches the boost libraries

**"ZMQ / RPC Connection Failed"**
- Ensure `robot/yor.py` or `robot/yor_mujoco.py` is running in another terminal
- Check if port 5557 is blocked or already in use

**Import errors after installation**
- Verify your environment is activated: `conda activate yor-nero` or `source .venv/bin/activate`
- Try reinstalling in editable mode: `pip install -e .`
