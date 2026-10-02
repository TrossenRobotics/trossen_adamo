## Installation

Install the build tools:

```sh
sudo apt install -y build-essential cmake git curl
```

Then install these dependencies before your first build:

- [Adamo SDK](https://docs.adamohq.com/quickstart/#install-the-sdk) — the **C** SDK, video build
- [libtrossen_arm](https://docs.trossenrobotics.com/trossen_arm/main/getting_started/software_setup.html#c) — latest version, installs to `/usr/local`
- [librealsense2](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) — needed to build `trossen_follower` and `vr_follower`.
- [trossen_vr](https://github.com/TrossenRobotics/trossen_vr) — needed by the VR binaries. Only the C++ library is required.

## Build

```sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/opt/adamo-sdk-linux-x86_64-video-latest
cmake --build build --parallel
```

Set `CMAKE_PREFIX_PATH` to wherever you extracted the Adamo SDK.

To skip a dependency, turn off the binary that needs it:
`-DADAMO_TROSSEN_BUILD_FOLLOWER=OFF`, `-DADAMO_TROSSEN_BUILD_VR_HEADSET=OFF`,
`-DADAMO_TROSSEN_BUILD_VR_FOLLOWER=OFF`. `vr_bimanual` is off by default, so
add `-DADAMO_TROSSEN_BUILD_VR_BIMANUAL=ON` to build it.

## Run

Set your Adamo API key in every terminal:

```sh
export ADAMO_API_KEY=ak_...
```

Replace `<...-IP>` below with your arms' IP addresses. Run `--help` on any
binary for the full list of options.

### Leader / follower teleop

Start the follower first, then the leader.

```sh
build/follower/trossen_follower --teleoperation-time 86400 --clear-error --follower-ip <FOLLOWER-IP>
```

```sh
build/leader/trossen_leader --teleoperation-time 86400 --clear-error --leader-ip <LEADER-IP>
```

The follower streams one RealSense camera by default. Add `--no-camera` to run without it.

### VR teleop

Start the headset bridge, then the follower.

```sh
build/vr_headset/vr_headset --teleoperation-time 86400
```

Single arm:

```sh
build/vr_follower/vr_follower --teleoperation-time 86400 --clear-error --follower-ip <FOLLOWER-IP>
```

Single arm with two cameras:

```sh
build/vr_follower/vr_follower --teleoperation-time 86400 --clear-error --follower-ip <FOLLOWER-IP> \
    --num-cameras 2 --camera-serial-0 <MAIN-SERIAL> --camera-serial-1 <WRIST-SERIAL>
```

Two arms:

```sh
build/vr_bimanual/vr_bimanual --teleoperation-time 86400 --clear-error \
    --right-arm-ip <RIGHT-IP> --left-arm-ip <LEFT-IP>
```

VR controls: hold the grip trigger to move the arm, release to pause. The index
trigger opens and closes the gripper. Press B (right) or Y (left) to exit.

## Troubleshooting

- `Unable to connect to ... zenoh.adamohq.com:443. Timeout!` usually means an
  outdated Adamo SDK. Reinstall the latest and make sure only one copy is installed.
- `Failed to load libcuda.so` on a computer without an NVIDIA GPU: use the CPU encoder.
  ```sh
  sudo apt install -y gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad
  export ADAMO_VIDEO_BACKEND=gstreamer ADAMO_VIDEO_ENCODER=x264enc
  ```
