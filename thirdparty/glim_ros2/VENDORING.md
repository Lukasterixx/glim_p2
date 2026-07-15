# Vendored glim_ros2

This directory is a **vendored, in-tree copy** of the GLIM ROS 2 wrapper
(`glim_ros`, the `glim_rosnode` executable), taken so glim_p2 has full control
over the ROS-side code and can never hit a version mismatch with the `glim` core
in this repo.

## Provenance

- Upstream: https://github.com/koide3/glim_ros2
- Pinned commit: `fc72f4603ed2d3dffcdf94cda5617963afe57e82`
- Vendored as plain source (upstream `.git` / `.github` stripped).

## Local patch

- `src/glim_ros/rviz_viewer.cpp` carries glim_p2's `publish_tf` / `predict_odom_tf`
  TF-gate changes (the "sim-mode publish_tf patch"). It is committed here directly
  — there is no longer a Docker-build-time overlay. The canonical copy of the patch
  is this file.

## Updating

To re-sync with upstream:

```bash
git clone https://github.com/koide3/glim_ros2 /tmp/glim_ros2
cd /tmp/glim_ros2 && git checkout <new-commit>
# re-apply the rviz_viewer.cpp patch (diff this dir's copy against upstream first)
rm -rf /tmp/glim_ros2/.git /tmp/glim_ros2/.github
# then copy /tmp/glim_ros2/* over thirdparty/glim_ros2/, keeping the patched
# src/glim_ros/rviz_viewer.cpp, and update the pinned commit above.
```
