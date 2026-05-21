# Containerised builds

Two images, each a multi-arch (`linux/amd64` + `linux/arm64`) manifest list:

| Image              | Binary             | Bundled deps                                   |
| ------------------ | ------------------ | ---------------------------------------------- |
| `trossen-leader`   | `trossen_leader`   | libadamo (Adamo C SDK), libtrossen_arm         |
| `trossen-follower` | `trossen_follower` | + librealsense2 (built from source)            |

The same images run on a Linux host directly and on macOS via Docker Desktop
(which selects `linux/arm64` on Apple Silicon, `linux/amd64` on Intel).

## Pull & run (when images are published)

```sh
export ADAMO_API_KEY=ak_...
docker compose -f docker/docker-compose.yml up follower   # on the follower host
docker compose -f docker/docker-compose.yml up leader     # on the leader host
```

`docker-compose.yml` defaults match the parent README:

- robot name `wxai`, leader arm `192.168.1.2`, follower arm `192.168.1.3`
- 100 Hz teleop, QUIC protocol, RealSense at 640×480 @ 30 fps
- Override any of those via env vars or by appending flags to `command`.

## Build from source

From the repo root (the build context = this directory's parent):

```sh
docker build -f docker/Dockerfile.leader   -t trossen-leader:dev   .
docker build -f docker/Dockerfile.follower -t trossen-follower:dev .
```

Build args (all optional, sensible defaults baked in):

| Arg                     | Default   | Notes                                        |
| ----------------------- | --------- | -------------------------------------------- |
| `ADAMO_SDK_VERSION`     | `0.1.34`  | Pulled from `install.adamohq.com/sdk/v<X>/`. |
| `TROSSEN_ARM_GIT_TAG`   | `main`    | Upstream `TrossenRobotics/trossen_arm` ref.  |
| `LIBREALSENSE_GIT_TAG`  | `v2.55.1` | follower only.                               |
| `UBUNTU_VERSION`        | `22.04`   | Base image tag.                              |

Multi-arch via buildx:

```sh
docker buildx build --platform linux/amd64,linux/arm64 \
    -f docker/Dockerfile.leader \
    -t ghcr.io/lukeschmitt-tr/trossen-leader:v0.1.34 --push .
```

The follower's `librealsense2` build adds ~5–10 min on first run; subsequent
builds reuse the cached `realsense` stage.

## Hardware passthrough notes (follower)

The compose file uses `privileged: true` and host networking — the
shortest path that works on a freshly-set-up Linux host. To tighten:

1. Install librealsense's udev rules on the **host** (not the container):

   ```sh
   sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/
   sudo udevadm control --reload && sudo udevadm trigger
   ```

2. Drop `privileged: true` from `docker-compose.yml`. Keep the
   `/dev/bus/usb` device mount and host networking.
3. Confirm `lsusb` on the host sees the RealSense before bringing the
   container up — if the host can't see it, the container can't either.

Time sync between the leader and follower hosts matters for low-latency
teleop. Run `chrony` (or `systemd-timesyncd`) on both ends and confirm
they're tracking the same source.
