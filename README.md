# trossen_adamo

The official [Adamo](https://adamohq.com) integration for Trossen Robotics'
WidowX AI, Pro, and Glide follower/leader teleop. Built by Adamo so Trossen hosts can deploy
Adamo teleop, recording, and replay against `libtrossen_arm` arms out of the
box. Seven binaries built from one CMake project:

| Binary               | Talks to                                                                                             |
| -------------------- | ---------------------------------------------------------------------------------------------------- |
| `trossen_leader`     | leader arm via `libtrossen_arm`, Adamo pubsub — Glide or wxai_v0 (`--model`)                        |
| `trossen_follower`   | follower arm via `libtrossen_arm`, Adamo pubsub — wxai_v0 or Pro (`--model`), RealSense or ZED video (`--camera-backend`) |
| `bimanual_leader`    | two leader arms via `libtrossen_arm`, Adamo pubsub — per-arm Glide or wxai_v0 (`--left-model`/`--right-model`) |
| `bimanual_follower`  | two follower arms via `libtrossen_arm`, Adamo pubsub — per-arm wxai_v0 or Pro, up to 4 RealSense or 3 ZED cameras |
| `vr_headset`         | Meta Quest VR headset via UDP (trossen_vr), Adamo pubsub                                             |
| `vr_follower`        | follower arm via `libtrossen_arm`, Adamo pubsub (VR control)                                         |
| `vr_bimanual`        | both follower arms via `libtrossen_arm`, Adamo pubsub (VR control)                                   |

For container builds (multi-arch Linux images), see [`docker/README.md`](docker/README.md).

## Architecture

### Leader-Follower Teleop

Two binaries on two hosts, talking over Adamo's pubsub bus:

- `trossen_leader` runs the leader arm in `external_effort` mode at
  `--rate-hz`, publishes joint state, and applies (negatively-scaled) force
  feedback from the follower.
- `trossen_follower` runs the follower arm in `position` mode, EMA-smooths
  incoming leader poses, publishes its own external efforts back, and
  (optionally) streams a color track to Adamo — RealSense or Stereolabs ZED,
  selected via `--camera-backend` (default `realsense`).

`trossen_leader` accepts `--model glide_right|glide_left|wxai_v0`
(default `glide_right`). A Glide leader talking to a wxai_v0/Pro follower
needs a joint-frame correction (Glide's joints 3/4 are mirrored relative to
wxai_v0's convention; joint 5 has a fixed grip-alignment offset of π/4) —
applied automatically before the pose goes on the wire whenever `--model` is a
Glide variant. Gripper force feedback differs per model: Glide uses a
normalised cubic fit; wxai_v0 applies the scaled effort directly.

`trossen_follower` accepts `--model wxai_v0|pro` (default `wxai_v0`) to select
the correct end-effector configuration for wxai_v0 or Pro follower arms.

#### Button-gated teleop (Glide leader only, single-arm)

Pass `--button-gated` to `trossen_follower` to control teleop from the Glide
leader's buttons instead of starting automatically (default off — without the
flag, nothing here changes):

- **SEL_1**: start/stop teleop. Stopping moves the follower to its home pose;
  starting ramps to the leader's current pose instead of snapping.
- **SEL_2**: if the follower has faulted (e.g. a joint/velocity limit), clears
  the error and resumes — no restart needed.

Requires a Glide leader (only Glide has buttons).

If the Glide leader's own driver faults, `trossen_leader` handles it without
crashing: it tells the follower to stop and go home immediately, waits 2s,
then attempts one clear. If that clear fails, or the leader faults again
before SEL_1 was pressed to resume teleop, the leader exits — otherwise it
waits for SEL_1 to resume normally. A fault after teleop has genuinely
resumed is treated as new and goes through the same recovery again.

**Button LEDs** guide the operator through the states above (Glide's LEDs are
monochrome — on/off/breathe, no color — so "needs attention" is a pulse, not
red):
- **Stopped**: SEL_1 breathes — press it to start.
- **Teleop active**: all four LEDs solid.
- **Error** (follower or leader fault): SEL_2 breathes — press it to recover.

### Bimanual Teleop

Two binaries on two hosts, each driving **two** arms (right + left) — the
same joint-space architecture as the single-arm pair above:

- `bimanual_leader` drives two leader arms. Each arm is configured
  independently via `--left-model glide_left|wxai_v0` and
  `--right-model glide_right|wxai_v0` (defaults: `glide_left`/`glide_right`).
  The Glide frame correction is applied per-arm: right's joint-5 offset is
  `+π/4`, left's is `−π/4`. Gripper feedback is model-aware per arm (cubic
  fit for Glide, direct scale for wxai_v0).
- `bimanual_follower` drives two follower arms, each configured independently
  via `--left-model wxai_v0|pro` and `--right-model wxai_v0|pro` (default
  `wxai_v0`). It EMA-smooths each arm's incoming pose independently and
  streams up to **4 RealSense** or **3 ZED** cameras (`--camera-backend`,
  `--num-cameras`, `--camera-track-N`, `--camera-serial-N` flags, N=0..3).

Left/right state and effort topics are namespaced with `_left`/`_right`
suffixes under the same robot name; both arms share a single ready-handshake
pair.

### VR Teleoperation

VR teleoperation enables remote control of Trossen arms using Meta Quest VR
controllers via Adamo pubsub. Two modes are supported:

**Single-arm mode** (`vr_headset` + `vr_follower`):
- `vr_headset` receives VR controller data via UDP (port 9000) from a Unity
  app on Meta Quest, publishes VR frames (poses + buttons + triggers) to Adamo.
- `vr_follower` subscribes to VR frames, controls one follower arm in cartesian
  mode. Grip/hand trigger to engage, release to pause. Button B to exit. Right
  index trigger controls gripper.

**Bimanual mode** (`vr_headset` + `vr_bimanual`):
- `vr_headset` broadcasts controller data for both hands to Adamo.
- `vr_bimanual` controls two follower arms simultaneously (right arm at
  192.168.1.4, left arm at 192.168.1.5). Each controller independently controls
  its corresponding arm. Button B (right) or Y (left) to exit.

On startup, all binaries use a ready-handshake: each side publishes a
wall-clock timestamp on its `*_ready` topic and blocks until it sees a fresh
sample from the peer. Either side exiting (Ctrl-C, fault, timeout) drives the
arms back to `home` then `sleep` via an RAII guard.

The integration primitives — latest-only subscriber, decoupled latest-only
publisher, ready handshake, arm-park guard, wire codec — live header-only
in `common/include/trossen_adamo/` and can be reused for other Trossen +
Adamo work.

## Build

```sh
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="/path/to/adamo-sdk;/path/to/zed" \
  -DTROSSEN_ARM_DIR=/path/to/trossen_arm/build/install
cmake --build build --parallel
```

Binaries land at
`build/{leader,follower,bimanual_leader,bimanual_follower,vr_headset,vr_follower,vr_bimanual}/`.

Useful flags:

- `-DADAMO_TROSSEN_BUILD_FOLLOWER=OFF` / `-DADAMO_TROSSEN_BUILD_LEADER=OFF` — skip single-arm teleop binaries.
- `-DADAMO_TROSSEN_BUILD_BIMANUAL_LEADER=OFF` / `-DADAMO_TROSSEN_BUILD_BIMANUAL_FOLLOWER=OFF` — skip bimanual binaries.
- `-DADAMO_TROSSEN_BUILD_VR_HEADSET=OFF` / `-DADAMO_TROSSEN_BUILD_VR_FOLLOWER=OFF` / `-DADAMO_TROSSEN_BUILD_VR_BIMANUAL=OFF` — skip VR binaries.
- `-DADAMO_TROSSEN_ENABLE_REALSENSE=OFF` / `-DADAMO_TROSSEN_ENABLE_ZED=OFF` — disable a camera backend so `trossen_follower`/`bimanual_follower` don't need that SDK at all (both default `ON`; each is independent, so a ZED-only build never needs `librealsense2` and a RealSense-only build never needs the ZED SDK/CUDA).
- `-DCMAKE_PREFIX_PATH=/abs/path/extracted-sdk` — point CMake at an Adamo SDK install (tarballs at <https://install.adamohq.com/sdk/>).
- `-DTROSSEN_ARM_GIT_TAG=<ref>` — pin the upstream `libtrossen_arm` ref.
- `-Drealsense2_DIR=/abs/path/lib/cmake/realsense2` — for non-system librealsense.

`librealsense2` is required for `trossen_follower`/`bimanual_follower` only when
`ADAMO_TROSSEN_ENABLE_REALSENSE` is `ON` (the default; `brew install librealsense`
on macOS, `apt install librealsense2-dev` on Linux). The Stereolabs **ZED SDK
v4** + CUDA are required only when `ADAMO_TROSSEN_ENABLE_ZED` is `ON` (also the
default; installed at `/usr/local/zed` on Jetson/L4T hosts — add that prefix to
`CMAKE_PREFIX_PATH` if not found automatically). VR binaries require
`trossen_vr` (install to `/usr/local` via `sudo make install`).

## Run

```sh
export ADAMO_API_KEY=ak_...
```

Defaults (override via env var or CLI flag — CLI > env > default):

| Setting     | Default       | Env                          | Flag            |
| ----------- | ------------- | ---------------------------- | --------------- |
| Robot name  | `wxai`        | `ADAMO_ROBOT_NAME`           | `--robot`       |
| Leader IP   | `192.168.1.2` | `ADAMO_TROSSEN_LEADER_IP`    | `--leader-ip`   |
| Follower IP | `192.168.1.3` | `ADAMO_TROSSEN_FOLLOWER_IP`  | `--follower-ip` |

**Order:** start the follower first (it brings the camera up before the
handshake), then the leader. Either side blocks until both have published a
fresh `*_ready` heartbeat.

**Follower** (wxai_v0 arm, RealSense camera):

```sh
build/follower/trossen_follower \
    --teleoperation-time 86400 \
    --rate-hz 100 \
    --clear-error \
    --protocol quic \
    --follower-ip 192.168.1.3 \
    --model wxai_v0 \
    --camera-backend realsense \
    --camera-width 640 --camera-height 480 --camera-fps 30 \
    --camera-bitrate-kbps 4000
```

Key follower options:
- `--model wxai_v0|pro` — arm model (default `wxai_v0`).
- `--no-camera` — skip the camera streamer (bench-test arms only).
- `--camera-backend realsense|zed` — which camera to stream (default `realsense`).
- `--camera-track NAME` — published track name (default `main`).
- `--camera-serial <SN>` — pin to a specific device if multiple are plugged in.
- `--camera-width`/`--camera-height` — RealSense only (default 640×480).
- `--camera-resolution HD2K|HD1200|HD1080|HD720|SVGA|VGA` — ZED only (default `HD1200`).

**Leader**:

```sh
build/leader/trossen_leader \
    --teleoperation-time 86400 \
    --rate-hz 100 \
    --velocity-limit 3.0 \
    --clear-error \
    --protocol quic \
    --leader-ip 192.168.1.2 \
    --model glide_right
```

- `--model glide_right|glide_left|wxai_v0` — arm model (default `glide_right`).

`Ctrl-C` on either side unwinds both cleanly (move-home → sleep). `--help` on
either binary prints the full flag list.

### Bimanual Classical Teleop

Defaults (override via env var or CLI flag — CLI > env > default):

| Setting            | Default       | Env                                  | Flag                  |
| ------------------ | ------------- | ------------------------------------ | --------------------- |
| Robot name         | `wxai`        | `ADAMO_ROBOT_NAME`                   | `--robot`             |
| Right leader IP    | `192.168.1.2` | `ADAMO_TROSSEN_RIGHT_LEADER_IP`      | `--right-leader-ip`   |
| Left leader IP     | `192.168.1.4` | `ADAMO_TROSSEN_LEFT_LEADER_IP`       | `--left-leader-ip`    |
| Right follower IP  | `192.168.1.3` | `ADAMO_TROSSEN_RIGHT_FOLLOWER_IP`    | `--right-follower-ip` |
| Left follower IP   | `192.168.1.5` | `ADAMO_TROSSEN_LEFT_FOLLOWER_IP`     | `--left-follower-ip`  |

Same start order: start `bimanual_follower` first, then `bimanual_leader`.

**Bimanual Follower** (ZED cameras — pin a serial per camera so multiple
streamers don't race for the same physical device):

```sh
build/bimanual_follower/bimanual_follower \
    --teleoperation-time 86400 \
    --rate-hz 100 \
    --clear-error \
    --protocol quic \
    --left-follower-ip 192.168.1.5 \
    --right-follower-ip 192.168.1.3 \
    --left-model wxai_v0 \
    --right-model wxai_v0 \
    --camera-backend zed \
    --num-cameras 3 \
    --camera-track-0 cam0 --camera-serial-0 <SN0> \
    --camera-track-1 cam1 --camera-serial-1 <SN1> \
    --camera-track-2 cam2 --camera-serial-2 <SN2> \
    --camera-resolution HD1200 --camera-fps 30 \
    --camera-bitrate-kbps 4000
```

Key bimanual follower options:
- `--left-model wxai_v0|pro` / `--right-model wxai_v0|pro` — per-arm model (default `wxai_v0`).
- `--camera-backend realsense|zed` — camera backend (default `realsense`).
- `--num-cameras N` — 1–4 for RealSense, 1–3 for ZED (default `3`).
- `--camera-track-N NAME` / `--camera-serial-N SERIAL` — per-camera track name and serial (N = 0, 1, 2, 3).
- `--camera-width`/`--camera-height` — RealSense only (default 640×480).
- `--camera-resolution HD2K|HD1200|HD1080|HD720|SVGA|VGA` — ZED only (default `HD1200`).
- `--no-camera` — disable all camera streamers.

**Bimanual Leader** (Glide arms; right's joint-5 offset is `+π/4`, left's
is `−π/4` — applied automatically per side):

```sh
build/bimanual_leader/bimanual_leader \
    --teleoperation-time 86400 \
    --rate-hz 100 \
    --velocity-limit 3.0 \
    --clear-error \
    --protocol quic \
    --left-leader-ip 192.168.1.4 \
    --right-leader-ip 192.168.1.2 \
    --left-model glide_left \
    --right-model glide_right
```

- `--left-model glide_left|wxai_v0` / `--right-model glide_right|wxai_v0` — per-arm model (defaults: `glide_left`/`glide_right`).

`Ctrl-C` on either side unwinds both arms on that host cleanly (move-home →
sleep). `--help` on either binary prints the full flag list.

Each side's Glide leader self-recovers from its own driver faults the same
way as the single-arm case above (independently — a fault on one side
doesn't affect the other), and each side's `--button-gated` `bimanual_follower`
behaves the same as the single-arm follower's SEL_1/SEL_2. Each side's
button LEDs are driven independently too — same Stopped/Active/Error
patterns as above.

### VR Teleoperation

**VR Headset** (receives Meta Quest controller data, publishes to Adamo):

```sh
build/vr_headset/vr_headset \
    --teleoperation-time 86400 \
    --rate-hz 100 \
    --protocol quic
```

**Single-arm VR Follower** (right controller controls one arm):

```sh
build/vr_follower/vr_follower \
    --teleoperation-time 86400 \
    --follower-ip 192.168.1.4 \
    --rate-hz 100 \
    --clear-error \
    --protocol quic
```

**Bimanual VR Follower** (both controllers control both arms):

```sh
build/vr_bimanual/vr_bimanual \
    --teleoperation-time 86400 \
    --right-arm-ip 192.168.1.4 \
    --left-arm-ip 192.168.1.5 \
    --rate-hz 100 \
    --clear-error \
    --protocol quic
```

**VR Controls:**
- **Grip/hand trigger**: Engage arm (hold to control, release to pause)
- **Index trigger**: Control gripper (0.0–1.0 → 0–40 mm opening)
- **Button B** (right) or **Y** (left): Exit teleoperation

**VR Defaults:**
- Robot name: `vr_teleop` (override via `--robot` or env `ADAMO_ROBOT_NAME`)
- Single-arm follower IP: `192.168.1.3` (override via `--follower-ip` or env `ADAMO_TROSSEN_FOLLOWER_IP`)
- Bimanual right arm IP: `192.168.1.4` (override via `--right-arm-ip` or env `ADAMO_TROSSEN_RIGHT_ARM_IP`)
- Bimanual left arm IP: `192.168.1.5` (override via `--left-arm-ip` or env `ADAMO_TROSSEN_LEFT_ARM_IP`)

## Supported platforms

Matches upstream `libtrossen_arm`: Ubuntu 20.04/22.04/24.04 on x86_64 + arm64,
and macOS 14/15 on arm64. macOS x86_64 is rejected at CMake configure time.

## Support

Built and maintained by [Adamo](https://adamohq.com) for Trossen Robotics.

- **Integration questions** (this code, build/run issues, requests for the
  bridge): file an issue in this repo.
- **Adamo platform questions** (operator UI, recording/replay, SDK
  semantics, accounts, API keys): see [docs.adamohq.com](https://docs.adamohq.com).

## License

[MIT](LICENSE). The Adamo C SDK (`libadamo`) is redistributed under its own
terms; see <https://install.adamohq.com>. `libtrossen_arm` is fetched from
[TrossenRobotics/trossen_arm](https://github.com/TrossenRobotics/trossen_arm).


