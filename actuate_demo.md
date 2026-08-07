# Adamo Teleop Demo — Actuate Conference Guide

Quick reference for building and running the Adamo bimanual teleop demo (Glide leader arms controlling Pro follower arms, with ZED camera streaming) at the Actuate conference.

This demo runs on **two computers**:

- **Follower computer** — connected to the two Pro follower arms and the 3 ZED cameras.
- **Leader computer** — connected to the two Glide leader arms.

---

## 1. Set the Adamo API Key

Run this in the terminal on **both** computers before building or running anything:

```bash
export ADAMO_API_KEY=ak_...
```

Get the real key from your adamo account

---

## 2. Pull the Latest Code

On **both** computers, before building, make sure you're on the right branch and up to date:

```bash
git checkout dev/bimanuel_pro_glide_zed_teleop
git pull
```

---

## 3. Build

### Follower Computer

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DTROSSEN_ARM_DIR=/usr/local \
  -DADAMO_TROSSEN_ENABLE_REALSENSE=OFF \
  -DADAMO_TROSSEN_BUILD_VR_HEADSET=OFF \
  -DADAMO_TROSSEN_BUILD_VR_FOLLOWER=OFF \
  -DADAMO_TROSSEN_BUILD_VR_BIMANUAL=OFF

cmake --build build --parallel
```

This keeps ZED camera support enabled (needed for this demo) and skips the RealSense camera backend and the VR binaries, none of which this demo uses.

### Leader Computer

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DTROSSEN_ARM_DIR=/usr/local \
  -DADAMO_TROSSEN_ENABLE_REALSENSE=OFF \
  -DADAMO_TROSSEN_ENABLE_ZED=OFF \
  -DADAMO_TROSSEN_BUILD_VR_HEADSET=OFF \
  -DADAMO_TROSSEN_BUILD_VR_FOLLOWER=OFF \
  -DADAMO_TROSSEN_BUILD_VR_BIMANUAL=OFF \
  -DADAMO_TROSSEN_BUILD_BIMANUAL_FOLLOWER=OFF \
  -DADAMO_TROSSEN_BUILD_FOLLOWER=OFF

cmake --build build --parallel
```

This computer only drives the leader arms, so it skips both camera SDKs entirely and skips building the follower binaries.

---

## 4. Check the Arm IPs and Camera Serials

These are set directly as command flags below. double-check they match the hardware actually connected before starting the demo.

**Arm IPs:**

- **Follower computer:** `--left-follower-ip 192.168.1.4`, `--right-follower-ip 192.168.1.5`
- **Leader computer:** `--left-leader-ip 192.168.1.3`, `--right-leader-ip 192.168.1.2`

**Camera serials (Follower computer only):** run this to list the ZED cameras currently plugged in and their serial numbers:

```bash
ZED_Explorer --all
```

Match each serial number to the correct `--camera-serial-N` flag in the follower run command below (step 6) — `cam_main`/`-0`, `cam_left`/`-1`, `cam_right`/`-2`. If a camera has been swapped, update its serial number here.

---

## 5. Two Flags Worth Understanding

### `--command-time` (smoothness tuning)

Controls how smooth vs. laggy the follower arm's motion looks, and the right value depends on the network between the two computers.

- Our demo command uses `--command-time 0.2`, which is a good starting point.
- If the arm looks jittery/jerky, raise this number a bit. If it feels laggy/delayed, lower it slightly.

### `--button-gated` (control teleop from the Glide leader's own buttons)

With this flag, teleop doesn't start automatically — it's controlled by the **SEL_1** and **SEL_2** buttons on each Glide leader arm:

- **SEL_1**: start/stop teleop for that arm. Stopping sends the follower arm to its home pose; starting ramps smoothly back to the leader's current pose (no sudden jump).
- **SEL_2**: if that arm's follower has faulted (hit a joint or speed limit), press this to clear the fault and resume — no restart needed.

Each arm (left/right) is independent — its buttons only affect its own side.

The buttons' LEDs tell you the state:
- **Stopped:** SEL_1 breathes (pulses) — press it to start.
- **Teleop active:** all four LEDs solid.
- **Error:** SEL_2 breathes — press it to recover.

---

## 6. Start the Demo

**Start order matters: start the follower first (it needs to bring the cameras up), then the leader.** Each side waits for the other automatically, so it's fine if there's a short gap between starting the two.

### Follower Computer

```bash
build/bimanual_follower/bimanual_follower \
    --robot trossen \
    --teleoperation-time 86400 \
    --rate-hz 100 \
    --connect-timeout 60 \
    --ready-timeout 60 \
    --clear-error \
    --left-follower-ip 192.168.1.4 \
    --right-follower-ip 192.168.1.5 \
    --left-model pro \
    --right-model pro \
    --button-gated \
    --command-time 0.2 \
    --camera-backend zed \
    --num-cameras 3 \
    --camera-track-0 cam_main --camera-serial-0 97900849 \
    --camera-track-1 cam_left --camera-serial-1 97525506 \
    --camera-track-2 cam_right --camera-serial-2 51287468 \
    --camera-resolution HD1200 \
    --camera-fps 30 \
    --camera-bitrate-kbps 4000
```

### Leader Computer

```bash
build/bimanual_leader/bimanual_leader \
    --robot trossen \
    --teleoperation-time 86400 \
    --force-feedback-gain 0.1 \
    --rate-hz 100 \
    --connect-timeout 60 \
    --ready-timeout 60 \
    --clear-error \
    --left-leader-ip 192.168.1.3 \
    --right-leader-ip 192.168.1.2 \
    --left-model glide_left \
    --right-model glide_right
```

`--teleoperation-time 86400` just means the session stays open for 24 hours, so it won't time out mid-conference.

---

## 7. Stopping the Demo

Press `Ctrl-C` on either computer. Both sides shut down cleanly on their own — arms move home, then go to sleep. No need to force-quit.

---

## Quick Troubleshooting

- **Follower/leader won't connect to each other:** confirm the follower was started *before* the leader, and both computers are on the same network.
- **Arm motion looks jittery or laggy:** adjust `--command-time` on the follower side (see step 5).
- **Camera track doesn't match the right camera:** re-run `ZED_Explorer --all` and confirm the serial numbers match step 4.
- **Follower arm stopped and won't respond:** if `--button-gated` is on, press **SEL_2** on that Glide leader arm to clear the fault, then **SEL_1** to resume.
