# BeamNG.tech + Openpilot Bridge

This is a proof-of-concept integration of [BeamNG.tech](https://beamng.tech) with [Openpilot](https://github.com/commaai/openpilot) for automotive security research and telematics data collection. It is not a standalone repo — these files drop into openpilot's `tools/sim/` directory as a replacement for the MetaDrive bridge.

The goal is to give researchers a realistic vehicle simulation environment with access to rich telematics data (100+ features: RPM, wheel speeds, IMU, electrics, etc.) without needing physical hardware.

This is example/proof-of-concept code. It works, but there are rough edges — see [Limitations](#limitations).

---

## Architecture

```
┌─────────────────────────────┐        ┌──────────────────────────────┐
│        Windows Host         │        │         WSL / Linux          │
│                             │        │                              │
│  BeamNG.tech (v0.33.3.0)   │◄──────►│  Openpilot                   │
│                             │        │    └─ run_bridge.py          │
│  - Vehicle physics          │        │    └─ BeamNGBridge           │
│  - Camera (dash_cam)        │  TCP   │         ├─ camera process    │
│  - Electrics sensor         │        │         ├─ control process   │
│  - GPS                      │        │         └─ sensors process   │
└─────────────────────────────┘        └──────────────────────────────┘
```

BeamNG runs on Windows. Openpilot runs in WSL (Ubuntu). They communicate over TCP via [beamngpy](https://github.com/BeamNG/BeamNGpy). Camera frames are streamed over network sockets (no shared memory across OS boundary — this is the main bottleneck, see [Limitations](#limitations)).

Three separate processes handle camera, controls, and sensors concurrently using a lock-free `DoubleBuffer` to avoid blocking between the fast openpilot control loop and the slower beamngpy polling.

---

## Prerequisites

### Windows side
- BeamNG.tech v0.33.3.0 (tested; newer versions likely work with minor changes)
- BeamNG.tech research license (free for academic use)

### WSL / Linux side
- Openpilot (follow their setup: https://github.com/commaai/openpilot)
- beamngpy: `pip install beamngpy`
- pyopencl (for camera YUV conversion)

---

## Setup

### 1. Drop files into openpilot

Copy the contents of this repo into openpilot's `tools/sim/` directory:

```
openpilot/tools/sim/
├── bridge/
│   ├── beamng_v3/          ← main bridge (use this)
│   │   ├── beamng_bridge.py
│   │   ├── beamng_world.py
│   │   ├── beamng_process.py
│   │   └── buffer.py
│   ├── beamng_v2/          ← older version, kept for reference
│   └── beamng.v1/          ← oldest version, kept for reference
└── run_bridge.py           ← already points to beamng_v3
```

### 2. Start BeamNG.tech on Windows

BeamNG must be launched with the `-tcom` flag to enable the tech communication interface. Replace the IP with your **Windows host IP** (the one reachable from WSL — check with `ip route` in WSL and look for the default gateway):

```bat
"C:\Program Files (x86)\BeamNG.tech.v0.38.3.0\Bin64\BeamNG.tech.x64.exe" -console -tcom -tcom-listen-ip 192.168.1.108
```

Load a scenario in BeamNG and make sure a vehicle named `thePlayer` exists in the scene. The bridge connects to whatever vehicle is currently active — it does not spawn one itself.

### 3. Configure the bridge

In `bridge/beamng_v3/beamng_world.py`, update the connection settings:

```python
self.bng = BeamNGpy(
    host='169.254.212.158',   # ← change to your Windows host IP (reachable from WSL)
    port=64256,
    home="/mnt/c/Program Files/BeamNG.tech.v0.33.3.0",  # ← path to BeamNG install (as seen from WSL)
    quit_on_close=True
)
```

**Finding your WSL-to-Windows IP:**
```bash
ip route | grep default   # the gateway IP is your Windows host
```

Or from PowerShell on Windows:
```powershell
ipconfig   # look for WSL adapter
```

### 4. Start Openpilot

```bash
./tools/sim/launch_openpilot.sh
```

### 5. Start the bridge

```bash
cd tools/sim
./run_bridge.py
```

---

## Controls

| Key  | Action                |
|------|-----------------------|
| `1`  | Cruise Resume / Accel |
| `2`  | Cruise Set / Decel    |
| `3`  | Cruise Cancel         |
| `r`  | Reset simulation      |
| `i`  | Toggle ignition       |
| `q`  | Quit                  |
| `wasd` | Manual control      |

To engage openpilot: press `2`, then use `1`/`2` to adjust cruise speed. Press `s` to disengage (simulates brake).

---

## Collected Data

When running, the bridge reads the following from BeamNG via the `Electrics` sensor and vehicle state:

- **Vehicle state**: velocity (x/y/z), position (x/y), steering angle
- **Electrics**: RPM, throttle, brake, clutch, gear, wheel speeds, and 40+ additional signals
- **Camera**: RGB dashcam feed (streamed to openpilot's vision pipeline as NV12)
- **GPS**: position

The camera resolution and other parameters are set in `beamng_world.py`:

```python
self.dash_cam = Camera(
    name="dash_cam",
    vehicle=self.vehicle,
    bng=self.bng,
    pos=(0, 0, 1.5),        # camera position relative to vehicle
    update_priority=1,
    is_streaming=True,
    is_render_colours=True,
    is_using_shared_memory=False,   # must be False for WSL
    resolution=self.resolution      # set by W, H in lib/common.py
)
```

---

## Code Structure

```
bridge/
├── common.py               # SimulatorBridge base class, main control loop
├── beamng_v3/
│   ├── beamng_bridge.py    # BeamNGBridge: spawns the world
│   ├── beamng_world.py     # BeamNGWorld: connects to BeamNG, manages processes
│   ├── beamng_process.py   # Three worker functions: camera, controls, sensors
│   └── buffer.py           # DoubleBuffer: lock-free shared state between processes
lib/
├── camerad.py              # Simulates openpilot's camerad: RGB→NV12, VisionIPC
├── simulated_car.py        # Simulates openpilot's car interface
├── simulated_sensors.py    # Feeds sensor data into openpilot's messaging
├── common.py               # SimulatorState, World base class, W/H constants
├── keyboard_ctrl.py        # Keyboard input thread
└── manual_ctrl.py          # Joystick input thread
run_bridge.py               # Entry point
launch_openpilot.sh         # Starts openpilot services
rgb_to_nv12.cl              # OpenCL kernel for color space conversion
```

### DoubleBuffer

The bridge uses a custom `DoubleBuffer` (`bridge/beamng_v3/buffer.py`) to share state between processes without blocking. Openpilot's control loop runs at 100Hz; beamngpy sensor polling is much slower. The DoubleBuffer lets the fast loop always read the latest available value without waiting on the slow loop:

```python
# writer (sensors process)
sensors_q.put((velocity_x, velocity_y, velocity_z, position_x, position_y, steering, bearing))

# reader (control loop) — always gets latest, never blocks
(vx, vy, vz, px, py, steering, bearing) = sensors_q.get()
```

### Steering angle

BeamNG's steering is normalized `[-1, 1]`. Openpilot works in degrees. The conversion factor is currently hardcoded at 700:

```python
# applying controls
self.control_q.put((-steer_angle / 700, throttle_out, brake_out))

# reading back
state.steering_angle = -steering * 700
```

This ratio varies by vehicle. You'll need to tune it per vehicle model.

---

## Limitations

- **Camera bottleneck**: BeamNG supports shared memory for camera streaming, but only within the same OS. Since BeamNG runs on Windows and openpilot in WSL, frames go over network sockets, which is slow. This is the main performance constraint. If you run both BeamNG and openpilot natively on Linux, you can enable shared memory by setting `is_using_shared_memory=True` in the camera setup — this should eliminate the bottleneck. BeamNG's Linux support was flaky on older versions but appears more stable now; untested with this bridge.
- **Hardcoded IP and path**: The BeamNG host IP and install path are hardcoded in `beamng_world.py`. Make sure to update these for your setup.
- **Hardcoded steering ratio**: The 700 factor works for some vehicles but needs tuning for others.
- **No scenario spawning**: The bridge connects to whatever is already running in BeamNG. You need to load a map and have a vehicle named `thePlayer` active before starting the bridge.
- **Not real-time guaranteed**: beamngpy polling introduces variable latency. Fine for data collection; not suitable for timing-sensitive experiments.

---

## Versioning

Three versions of the bridge are included:

- `beamng.v1` — initial prototype, direct beamngpy calls in the main loop
- `beamng_v2` — refactored to match openpilot's MetaDrive bridge structure
- `beamng_v3` — current version, adds multiprocess architecture and DoubleBuffer

Use `beamng_v3`. The others are kept for reference.

---

## Related Work

- [Independent BeamNG+Openpilot bridge](https://forum.beamng.tech/t/demo-of-beamng-bridge-for-openpilot/451) — a separate implementation by a different author, also bridging BeamNG with openpilot. Uses [SimHub](https://www.simhubdash.com/) for vehicle data extraction instead of beamngpy. [Video demo](https://www.youtube.com/watch?v=I-7zSl75KWM&t=2s).
- [VehicleSec '25 paper](https://www.usenix.org/conference/vehiclesec25) — describes this framework and telematics data collection methodology
- [PIVOT Project](https://pivot-auto.org/) — datasets collected using this framework will be published here
- New version (WIP): replaces beamngpy with a CAN bus layer using custom Lua modules for BeamNG — eliminates the camera bottleneck and enables proper automotive security research (CAN injection, replay attacks, etc.)

---

## License

This code is released for research purposes. BeamNG.tech requires a separate research license from [BeamNG GmbH](https://beamng.tech). Openpilot is licensed under MIT by [comma.ai](https://github.com/commaai/openpilot).
