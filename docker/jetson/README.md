# GLIM fork — Jetson (dog) test container

Build **this GLIM fork** on the NVIDIA Jetson (Orin, JetPack 6.1) so you can test
it on the dog without touching the robot's main container.

This mirrors the GLIM build recipe from P2Dingo's `Dockerfile.jetson` (same base
image, OpenCV equivs mock, GTSAM 4.3a0 + gtsam_points from source with CUDA arch
87, CycloneDDS, Livox MID360 driver) but builds the local repo (`COPY .`) instead
of cloning `koide3/glim` upstream. GLIM is built **headless** (no viewer /
Iridescence), exactly like P2Dingo runs it on the dog.

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

The default command brings up the **MID360 driver + GPU GLIM** together (see below).

## Livox MID360

The container runs the MID360 driver itself (same SDK + driver + `MID360_config.json`
+ driver-only launch as P2Dingo), publishing `/livox/lidar` + `/livox/imu`, then
starts GLIM against them. The baked config's topics are remapped to those (the repo
default is Ouster `/os_cloud_node/*`); frames + `acc_scale` stay auto-detected.

The LiDAR IP (`192.168.123.20`) and host NIC (`192.168.123.18`) are baked into the
`MID360_config.json` heredoc in the Dockerfile — edit + rebuild if yours differ.
`--net host` gives the container access to that Livox network via the dog's NIC.

## Config

The image bakes this repo's `config/` at `/glim/config` as a default. To test the
fork against the **robot-tuned** config, mount the robot's `glim_config` over it
(uncomment the volume in `docker-compose.yaml`), e.g.:

```
- /path/to/P2Dingo/Isaac/go2_omniverse/glim_config:/glim/config:ro
```

## How it interoperates on the dog

The container runs its **own MID360 driver + GLIM** and shares the host stack
(`--net host`, CycloneDDS, `ROS_DOMAIN_ID=0`). Run it **standalone** — not at the
same time as the robot's main container, which runs its own Livox driver and
`glim_rosnode`. Two would clash on the MID360's UDP ports and on the `glim_rosnode`
node name. To A/B against the robot's GLIM, stop the robot stack first.

## Notes

- Requires **BuildKit** (the Docker default on JetPack 6.1) — the Dockerfile uses
  heredocs to stay self-contained. If you build with a legacy builder, run with
  `DOCKER_BUILDKIT=1`.
- **Headless** — built with `BUILD_WITH_VIEWER=OFF` (no Iridescence), like P2Dingo
  on the dog. The baked config loads only `librviz_viewer.so` (RViz markers over the
  network) + `libimu_prediction.so`; the GL viewer modules aren't built.
- The fork's `CMakeLists.txt` gates its ROS2 modules on `$ENV{ROS_VERSION} EQUAL 2`,
  which this base image doesn't export — the Dockerfile pins `ROS_VERSION=2` before
  `colcon` so that check passes (otherwise cmake errors with "Unknown arguments").
- On clean shutdown (SIGINT — compose sends this) GLIM serializes its map dump to
  `/tmp/dump`, mounted to `./glim_dump/` on the host.
- CUDA arch is pinned to `87` (Orin). Change `CMAKE_CUDA_ARCHITECTURES` in the
  Dockerfile for a different Jetson.
