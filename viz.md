# sinew-viz — live ANNY body from sinew trackers

Drives an **ANNY** parametric body mesh (via **SOMA-X** retargeting) from the
native sinew driver and shows it in the **native polyscope viewer** (`anny_demo`,
Vulkan GPU skinning). No restricted/non-commercial parametric body topology is
used; SOMA-X retargets onto the ANNY identity (Apache-2.0 / CC0), so the shipped
body is ANNY.

## Pipeline (no Python runtime)

```text
dongle (USB) ─▶ sinew_tui (C, 9-axis fused) ─OSC /sinew/tracker─▶
   anny_demo (C++):  15 sinew joints ─▶ G calibrate ─▶ FK ─▶ Vulkan LBS (lbs.spv) ─▶ polyscope mesh
   SteamVR: /vr/* in on 39541, 15 virtual trackers out to driver_sinew on 39542 @ 62.5 Hz
```

The **runtime is a single native binary** — no Python, PyTorch, viser, or ONNX. The
SOMA/ANNY rig + skinning weights and the phenotype model are baked **once** into the
native assets `soma_rig.h` (`bake/make_rig.py`) and `soma_pheno.bin` (`bake/make_pheno.py`) in
the heavy **`export`** env; `anny_demo` compiles/loads them and runs the per-frame LBS
on the GPU via the Lean→Slang `lbs.spv` kernel. `anny_demo` does its own calibration,
foot-lock root motion, and the full SteamVR bridge in C++.

- **OSC contract**: `/sinew/tracker ,sffffd` = (joint_label, qw,qx,qy,qz, t).
  Quaternion order **w,x,y,z**. (`/sinew/accel` exists but is **ignored** —
  it's gravity-noisy; see root-motion note below.)
- **Rotation representation**: **3×3 rotation matrices end-to-end** — no
  quaternions or Euler/axis-angle internally. The incoming quat converts to
  a matrix on ingest, and `SOMALayer(..., pose2rot=False)` takes a `(1,77,3,3)`
  matrix pose. (Quats appear only at the dongle-input and viser-gizmo boundaries.)
- **Joint map** (`SINEW_TO_SOMA`): `LeftUpperLeg→LeftLeg`, `LeftLowerLeg→LeftShin`,
  `LeftUpperArm→LeftArm`, `LeftLowerArm→LeftForeArm`; the rest are 1:1.
- **SOMA/ANNY API** (pinned by `bake/introspect.py`): pose `(1,77,3,3)` rot-matrices
  with `pose2rot=False` (78 public joints minus Root), ANNY identity `(1,11)` =
  `[gender,age,muscle,weight,height,proportions,cupsize,firmness,african,asian,caucasian]`,
  `scale_params={}` (SOMA maps it to anny local_changes — must be `{}` not `None`),
  faces 36108 tris, output dict `["vertices"(18056,3), "joints"(77,3)]`.
  Rest pose: `t_pose_world`/`t_pose_local` (78,4,4); hierarchy
  `public_joint_parent_ids`. Shape priors: `shape_pca`, `shape_mean`,
  `shape_eigenvalues`.
- **`enable_procedural_transforms=False`, `apply_correctives=False`** (both
  required together). The procedural rig returns a *degenerate public-joint rest*
  for the legs — thigh/shin/foot land at shoulder height (~y 1.44 m) with ~4 cm
  bones — so FK flings the legs up to the head. The legacy 78-joint rig hangs the
  legs correctly (thigh→shin→foot straight down) and needs no GitHub sidecars.
- **Units: metres everywhere.** `t_pose_world` ships in cm; it is used only for
  its scale-free 3×3 rotations, and the one place its positions are printed
  scales by `CM_TO_M`. All runtime geometry (mesh, `joints`, root motion) is m.

## Setup

Pure CMake — a C/C++ toolchain plus the windowing libs, no Python:

- **Linux:** a C++17 compiler, `cmake` (≥3.19), and `apt install xorg-dev libgl1-mesa-dev` (the GL/X11
  dev libs polyscope needs). `ninja` is optional (`-G Ninja` for faster builds).
- **Windows:** Visual Studio (MSVC) or MinGW, plus CMake.

The baked rig (`soma_rig.h`) is committed and the large runtime assets (`soma_pheno.bin`, calibrator
weights) come from releases, so the build needs **no Python**. Re-baking the rig after a SOMA/ANNY
update is the only Python step (`bake/`, run out-of-band — it is not part of the build).

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release   # add -G Ninja if ninja is installed
cmake --build build --parallel                   # sinew_tui + anny_demo + driver_sinew (one tree)
cmake --build build --target package             # CPack release tarball
```

## Run

```bash
build/sinew_tui              # the TUI driver (auto-detects the serial port; COM5 on Windows)
build/viz_native/anny_demo   # the native ANNY viewer
```
Override the serial port with `$SINEW_PORT` (`$env:SINEW_PORT` on Windows).

> ⚠️ On Windows the driver finds the dongle by USB **VID:PID**
> (`VID_248A&PID_8002 REBORNRX`) and uses whatever COM it enumerated as. If that
> match fails it tries COM5, then scans `SERIALCOMM` and takes the first openable
> port (often COM1, the motherboard serial — opens fine, no dongle). Override with
> `--port <COM>` (or `$env:SINEW_PORT` via `run.py`).

**Manual (native, dongle in WSL):**

1. Free the dongle (close the vendor capture app), `usbipd attach --wsl --busid <dongle>` → `/dev/ttyACM*`, `sudo chmod 666 /dev/ttyACM1`.
2. Driver (needs a pty — wrap in `script`):
   `script -qfc "./build-wsl/sinew_tui 39539 39540 --port /dev/ttyACM1 --ip 127.0.0.1" /dev/null`
3. Viewer: run the native `anny_demo` (it binds OSC 39539 itself).

## Viewer panel (polyscope, left-docked)

- **Phenotype** (collapsing header): live sliders (gender/age/muscle/weight/height/proportions/…)
  reshape the body on the GPU — an advantage over the baked viser body.
- **T-pose calibrate (3s)**: 3 s countdown, then captures the per-joint sensor→bone offset **and**
  a hips-referenced global alignment `G` (sensor-world → model-world), so motion deltas are
  conjugated into the model frame (`bone_world = G·R·off`) and limbs rotate about anatomical axes
  instead of swinging up. **clear calibration** holds the neutral rest pose.
- **Pre-calibrated stream**: passthrough mode — the bone world orientation is streamed directly; no
  live calibration needed.
- **show ANNY mesh** / **show tracker axes**: toggle the mesh and the per-tracker RGB axis gizmos.
- **root motion (foot lock)**: see below.
- **SteamVR**: **HMD root** (re-root the body to the headset), **VR IK** (reach unsensored arms to
  the controllers), **drive SteamVR trackers** (feed 15 virtual trackers to `driver_sinew`).
  Defaults come from `SINEW_HMD_ROOT` / `SINEW_VR_IK` / `SINEW_DRIVE_VR`.

## Root motion / 6DOF

IMUs give **rotation only** — full body motion (root translation) must be
*solved*:

- **Horizontal**: lock the planted (lower) foot's XZ; root translates as you step.
- **Vertical**: ground the lower foot; jumps add a ballistic height integrated
  from the Hips vertical accel (gravity removed via EMA, contact decay).

The accel jump drifts. The robust path is **bone-length + foot-contact FK**: with
known leg bone lengths (from the SOMA rest pose) and a planted foot at known
ground height, FK pins the hip/root exactly. The body model's constant bone
lengths and joint hierarchy are constraints that also regularize the pose (no limb
stretching). Sensor translation/limb-scale need no solving: a rigid IMU reports
the same segment orientation wherever it sits, and FK adopts the model's constant
bone lengths — only mounting **rotation** matters.

## Yaw drift / 9-axis

The tracker's onboard quaternion is **6-axis** (gyro+accel): good pitch/roll, but
yaw **free-runs**, so limbs drift seconds after calibration — and each tracker's
quat lives in its own world frame, so no single `G` or calibration holds them.
The fix is **9-axis** fusion (magnetometer + accel), implemented driver-side; the
method, the solver comparison, and the shipped complementary filter are specified
in `spec/Sinew/Fusion.lean`. The driver emits
the corrected quat on `/sinew/tracker`, so the viz consumes it transparently. See
`docs/host-commands.md` § "9-axis fusion" for the operational summary.

## Files

- `viz_native/anny_demo.cpp` — the live app (OSC → FK → Vulkan LBS → polyscope; calibration,
  foot-lock root motion, tracker/device gizmos, and the SteamVR bridge). The only runtime.
- `CMakeLists.txt` — the whole build (root includes `viz_native` + `steamvr`); CPack packages the
  release tarball. Pure CMake, no Python.
- `bake/make_rig.py` / `bake/make_pheno.py` / `bake/introspect.py` — the only Python: regenerate the
  baked assets `soma_rig.h` / `soma_pheno.bin` after a SOMA/ANNY update. Run out-of-band (e.g. a venv
  with torch + py-soma-x); the build never invokes them — `soma_rig.h` is committed.
