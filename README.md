# trossen_adamo

The official [Adamo](https://adamohq.com) integration for Trossen Robotics'
WidowX AI follower/leader teleop. Built by Adamo so Trossen hosts can deploy
Adamo teleop, recording, and replay against `libtrossen_arm` arms out of the
box. Four binaries built from one CMake project:

| Binary             | Talks to                                                         |
| ------------------ | ---------------------------------------------------------------- |
| `trossen_leader`   | leader arm via `libtrossen_arm`, Adamo pubsub                    |
| `trossen_follower` | follower arm via `libtrossen_arm`, Adamo pubsub, RealSense video |
| `vr_headset`       | Meta Quest VR headset via UDP (trossen_vr), Adamo pubsub         |
| `vr_follower`      | follower arm via `libtrossen_arm`, Adamo pubsub (VR control)     |

For container builds (multi-arch Linux images), see [`docker/README.md`](docker/README.md).

## Architecture

### Arm-to-Arm Teleop

Two binaries on two hosts, talking over Adamo's pubsub bus:

- `trossen_leader` runs the leader arm in `external_effort` mode at
  `--rate-hz`, publishes joint state, and applies (negatively-scaled) force
  feedback from the follower.
- `trossen_follower` runs the follower arm in `position` mode, EMA-smooths
  incoming leader poses, publishes its own external efforts back, and
  (optionally) streams a RealSense color track to Adamo.

### VR Teleop

Two binaries for Meta Quest remote control:

- `vr_headset` receives VR controller data via UDP (port 9000) from a Unity
  app on Meta Quest, publishes VR frames (poses + buttons) to Adamo.
- `vr_follower` subscribes to VR frames, controls the follower arm in
  cartesian mode. Press A to engage/pause, B to exit, right trigger controls
  gripper.

On startup, both teleop modes use a ready-handshake: each side publishes a
wall-clock timestamp on its `*_ready` topic and blocks until it sees a fresh
sample from the peer. Either side exiting (Ctrl-C, fault, timeout) drives the
arm back to `home` then `sleep` via an RAII guard.

The integration primitives — latest-only,vr_headset,vr_follower}/st-only
publisher, ready handshake, arm-park guard, wire codec — live header-only
in `common/include/trossen_adamo/` and can be reused for other Trossen +
Adamo work.

## Build

```sh
cmake -S . -B build
cmake --build build --parallel
```

Binaries land at `build/{leader,follower}/trossen_{leader,follower}`.

Useful flags:

- `-DADAMO_TROSSEN_BUILD_FOLLOWER=OFF` / `-DADAMO_TROSSEN_BUILD_LEADER=OFF` / `-DADAMO_TROSSEN_BUILD_VR_HEADSET=OFF` / `-DADAMO_TROSSEN_BUILD_VR_FOLLOWER=OFF` — skip binaries.
- `-DCMAKE_PREFIX_PATH=/abs/path/extracted-sdk` — point CMake at an Adamo SDK install (tarballs at <https://install.adamohq.com/sdk/v0.1.34/>).
- `-DTROSSEN_ARM_GIT_TAG=<ref>` — pin the upstream `libtrossen_arm` ref (default `v1.10.0`).
- `-Drealsense2_DIR=/abs/path/lib/cmake/realsense2` — for non-system librealsense.

`librealsense2` is required only for the follower (`brew install librealsense`
on macOS, `apt install librealsense2-dev` on Linux). VR binaries require
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

**Follower** (needs `sudo -E` on macOS so librealsense can claim the USB device):

```sh
sudo -E build/follower/trossen_follower \
    --teleoperation-time 86400 \
    --rate-hz 100 \
    --clear-error \
    --protocol quic \
    --camera-width 640 --camera-height 480 --camera-fps 30 \
    --camera-bitrate-kbps 4000
```

- `--no-camera` — skip the RealSense streamer (bench-test arms only).
- `--camera-track NAME` — override the published track name (default `main`).
  Adamo's operator UI groups tracks by these names (`main`/`front`/`rear`/
  `head`/`overlay`).
- `--camera-serial <SN>` — pin to a specific RealSense if multiple are plugged in.

**Leader**:

```sh
build/leader/trossen_leader \
    --teleoperation-time 86400 \
    --rate-hz 100 \
    --velocity-limit 3.0 \
    --clear-error \
    --protocol quic
```

`Ctrl-C` on either side unwinds both cle

### VR Teleoperation

**VR Headset** (receives Meta Quest controller data, publishes to Adamo):

```sh
build/vr_headset/vr_headset \
    --teleoperation-time 86400 \
    --rate-hz 100 \
    --protocol quic
```

**VR Follower** (subscribes to VR frames, controls robot arm):

```sh
build/vr_follower/vr_follower \
    --teleoperation-time 86400 \
    --follower-ip 192.168.1.4 \
    --rate-hz 100 \
    --clear-error \
    --protocol quic
```

Defaults: robot name `vr_headset`, follower IP `192.168.1.3` (override via
`--robot` / `--follower-ip` or env `ADAMO_ROBOT_NAME` /
`ADAMO_TROSSEN_FOLLOWER_IP`). Start follower first, then headset. Press **A**
to engage teleop, **B** to exit.anly (move-home → sleep). `--help` on
either binary prints the full flag list.

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
