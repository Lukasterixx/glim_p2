# GLIM fork — Jetson (dog) test container

Build **this GLIM fork** on the NVIDIA Jetson (Orin, JetPack 6.1) so you can test
it on the dog without touching the robot's main container.

This mirrors the GLIM build recipe from P2Dingo's `Dockerfile.jetson` (same base
image, OpenCV equivs mock, GTSAM 4.3a0 + gtsam_points + Iridescence from source
with CUDA arch 87, CycloneDDS) but is **scoped to GLIM only** and builds the local
repo (`COPY .`) instead of cloning `koide3/glim` upstream.

## GPU

- **GLIM runs on the GPU.** `glim` and `gtsam_points` are built with
  `BUILD_WITH_CUDA=ON` (CUDA arch 87 / Orin), and the baked `config/config.json`
  selects the GPU modules (`config_odometry_gpu` / `config_sub_mapping_gpu` /
  `config_global_mapping_gpu`).
- **GPU / CUDA / NVIDIA OpenCV access is set up the same way as P2Dingo's robot
  container:** `runtime: nvidia` + `NVIDIA_VISIBLE_DEVICES=all` +
  `NVIDIA_DRIVER_CAPABILITIES=all`. The base image ships the NVIDIA CUDA-built
  OpenCV; the equivs mock keeps it (ROS won't pull a non-CUDA OpenCV over it) and
  the entrypoint `LD_PRELOAD`s it + GTSAM to avoid the mixed-OpenCV double-free.

## Distinct names (won't clash with the robot/sim)

| | image | container |
|---|---|---|
| robot main stack | `p2dingo` | `p2dingo` |
| sim GLIM | `glim_cyclone` | `glim_ros2` |
| **this fork test** | **`glim_p2_jetson`** | **`glim_p2_test`** |

## Build & run

Must be built with the **repo root** as context (the Dockerfile does
`COPY . $WS/src/glim`). Easiest via compose:

```bash
# from the repo root
docker compose -f docker/jetson/docker-compose.yaml up --build
```

or plain docker:

```bash
docker build -f docker/jetson/Dockerfile -t glim_p2_jetson .
docker run --rm -it --name glim_p2_test \
  --net host --ipc host --privileged \
  --runtime nvidia \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  --ulimit memlock=-1 \
  glim_p2_jetson
```

The default command runs `glim_rosnode` against `/glim/config`.

## Config

The image bakes this repo's `config/` at `/glim/config` as a default. To test the
fork against the **robot-tuned** config, mount the robot's `glim_config` over it
(uncomment the volume in `docker-compose.yaml`), e.g.:

```
- /path/to/P2Dingo/Isaac/go2_omniverse/glim_config:/glim/config:ro
```

## How it interoperates on the dog

Runs with `--net host` + CycloneDDS (`ROS_DOMAIN_ID=0`), so it sees the LiDAR/IMU
topics published by the robot container. To test the fork **in place of** the
robot's GLIM, stop the robot's `glim_rosnode` first — both use the node name
`glim_rosnode` and the topic `/glim_rosnode/points`, so running both at once will
conflict.

## Notes

- Requires **BuildKit** (the Docker default on JetPack 6.1) — the Dockerfile uses
  heredocs to stay self-contained. If you build with a legacy builder, run with
  `DOCKER_BUILDKIT=1`.
- Built with the viewer ON (Iridescence) for config parity with P2Dingo, whose
  GLIM config loads `libstandard_viewer.so` / `librviz_viewer.so`. If the dog has
  no display, trim `extension_modules` in your mounted `config_ros.json` to run
  headless.
- On clean shutdown (SIGINT — compose sends this) GLIM serializes its map dump to
  `/tmp/dump`, mounted to `./glim_dump/` on the host.
- CUDA arch is pinned to `87` (Orin). Change `CMAKE_CUDA_ARCHITECTURES` in the
  Dockerfile for a different Jetson.
