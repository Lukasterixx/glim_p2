# Livox MID360 driver for glim_p2 (real-sensor mode)

These assets let the glim_p2 Docker container run the Livox MID360 driver itself,
publishing `/livox/lidar` (PointCloud2) and `/livox/imu`, and feed them into
GLIM's own LiDAR-IMU odometry. Everything lives inside this repo.

| Path | Purpose |
|---|---|
| `livox/MID360_config.json` | MID360 network config — lidar `192.168.123.20`, host `192.168.123.31` |
| `livox/mid360_driver_only.launch.py` | Driver-only launch (no RViz); frame `livox_frame`, 10 Hz |
| `docker/run_with_livox.sh` | Starts the driver, then GLIM; forwards SIGINT for a clean map dump |
| `docker/mid360/Dockerfile` | Builds the full glim_p2 stack + Livox SDK2 + `livox_ros_driver2` |
| `docker/mid360/docker-compose.yaml` | Real-sensor compose (context = repo root) |
| `docker/mid360/config/` | Real-sensor GLIM config (`/livox/lidar`, `/livox/imu`, own odometry) |

Key GLIM config differences from the repo default (`config/`):
`config_ros.json` → `points_topic=/livox/lidar`, `imu_topic=/livox/imu`,
`acc_scale=9.80665`, `imu/lidar_frame_id=livox_frame`, `publish_imu2lidar=false`,
`publish_tf=true`; `config_sensors.json` → `T_lidar_imu` identity.

## Network prerequisite (do this on the host first)

The MID360 talks over a dedicated wired Ethernet link. This PC must have a NIC
on the same subnet as the lidar, at the host IP baked into `MID360_config.json`
(`192.168.123.31`).

```bash
ip link show                                    # find your wired iface, e.g. enp3s0
sudo ip addr add 192.168.123.31/24 dev <iface>  # temporary; use netplan to persist
ping 192.168.123.20                             # should reply once cabled + powered
```

If `192.168.123.31` clashes with another device, change it in both `host_net_info`
blocks of `livox/MID360_config.json` and rebuild. The lidar IP (`192.168.123.20`)
is fixed by the sensor.

## Build & run

```bash
cd docker/mid360
xhost +local:docker                 # allow the container's GUI window
docker compose build
docker compose up
```

## Mapping vs. continue (automatic)

`run_with_livox.sh` picks the mode from the mounted `docker/mid360/dump/` dir:

- **empty dump → MAPPING**: GLIM builds a new map and saves it to `dump/` on clean
  shutdown (Ctrl+C / `docker compose down`).
- **dump has a saved map (`graph.bin`/`graph.txt`) → CONTINUE**: GLIM loads that
  map and keeps mapping onto it, as if it was never turned off. The full pipeline
  stays on (odometry + sub-mapping + global mapping); the loaded submaps come back
  as optimizable variables, the new session relocalizes onto them, and overlap-based
  loop closure locks old and new poses together. The prior map is never overwritten
  — the grown map is written to `dump/continued/`.

So the second run onwards automatically extends the first run's map. To map
again from scratch, clear the dump: `rm -rf docker/mid360/dump/*`.

Override the auto choice via `environment:` in `docker-compose.yaml` (or `-e`):
`GLIM_MODE=mapping` forces fresh mapping even with a map present; `GLIM_MODE=continue`
forces continue. `PRIOR_MAP_DIR` changes which dir is used as the prior map.

## Verify topics

```bash
docker exec -it glim_ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && ros2 topic hz /livox/lidar && ros2 topic echo /livox/imu --once'
```
