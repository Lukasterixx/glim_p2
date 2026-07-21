#!/bin/bash
# Start the Livox MID360 driver, then run GLIM (passed as "$@").
#
# The driver publishes PointCloud2 on /livox/lidar and IMU on /livox/imu; GLIM
# (glim_rosnode) subscribes to those. GLIM is the foreground/last child so it
# receives the compose stop_signal (SIGINT) for a clean shutdown + map dump,
# after which the Livox driver is torn down.
set -euo pipefail

# The container runs as root, so GLIM's map dump under /tmp/dump (bind-mounted to
# the host) would be written root-owned 755 — its subdirs (000000/, config/) then
# can't be deleted by the host user. umask 000 makes GLIM create the dump dirs
# 0777 / files 0666 so any host user can clear ./dump between runs.
umask 000

LIVOX_PID=""
GLIM_PID=""
RVIZ_PID=""

shutdown() {
  if [ -n "$RVIZ_PID" ] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    echo "[container] Stopping RViz..."
    kill -INT "$RVIZ_PID" 2>/dev/null || true
    wait "$RVIZ_PID" 2>/dev/null || true
  fi
  if [ -n "$GLIM_PID" ] && kill -0 "$GLIM_PID" 2>/dev/null; then
    echo "[container] Forwarding shutdown to GLIM..."
    kill -INT "$GLIM_PID" 2>/dev/null || true
    wait "$GLIM_PID" 2>/dev/null || true
  fi
  if [ -n "$LIVOX_PID" ] && kill -0 "$LIVOX_PID" 2>/dev/null; then
    echo "[container] Stopping Livox driver..."
    kill -INT "$LIVOX_PID" 2>/dev/null || true
    for _ in 1 2 3 4 5; do
      kill -0 "$LIVOX_PID" 2>/dev/null || { wait "$LIVOX_PID" 2>/dev/null || true; return; }
      sleep 1
    done
    kill -TERM "$LIVOX_PID" 2>/dev/null || true
    wait "$LIVOX_PID" 2>/dev/null || true
  fi
}

trap 'shutdown; exit 0' INT TERM

set +u
source /opt/ros/humble/setup.bash
source /root/ws_livox/install/setup.bash
source /root/ros2_ws/install/setup.bash
set -u

# The MID360 streams points over UDP at a high rate; small kernel buffers drop
# packets. Requires a privileged container (see docker-compose.real.yaml).
if sysctl -w net.core.rmem_max=2147483647 >/dev/null 2>&1; then
  echo "[container] Kernel UDP receive buffer max raised."
else
  echo "[container] WARNING: could not raise net.core.rmem_max; run privileged for reliable Livox UDP."
fi
sysctl -w net.core.rmem_default=2147483647 >/dev/null 2>&1 || true
sysctl -w net.core.wmem_max=2147483647     >/dev/null 2>&1 || true
sysctl -w net.core.wmem_default=2147483647 >/dev/null 2>&1 || true

echo "[container] Starting Livox MID360 driver -> /livox/lidar, /livox/imu ..."
ros2 launch livox_ros_driver2 mid360_driver_only.launch.py &
LIVOX_PID=$!

# Give the C++ driver time to spawn, then pin it to real-time priority so the
# UDP receive thread is not starved (best-effort; needs SYS_NICE / privileged).
sleep "${LIVOX_PRIORITY_DELAY:-5}"
DRIVER_PID="$(pgrep -f livox_ros_driver2_node | head -n 1 || true)"
if [ -n "$DRIVER_PID" ] && chrt -f -p "${LIVOX_RT_PRIORITY:-90}" "$DRIVER_PID" 2>/dev/null; then
  echo "[container] Livox driver real-time priority set to ${LIVOX_RT_PRIORITY:-90}."
else
  echo "[container] WARNING: could not set Livox driver real-time priority."
fi

# --- GLIM mode selection -----------------------------------------------------
# There are exactly two modes:
#   mapping  : build a fresh map and save it to the dump dir on shutdown.
#   continue : load the saved map from the dump dir and keep mapping onto it
#              (the new session relocalizes onto it, then loop-closes against it).
# auto (default): pick `continue` if the dump already holds a saved map, else `mapping`.
# Override with GLIM_MODE=mapping|continue. `localize`/`localise` are accepted as
# deprecated aliases of `continue` (the old standalone localization mode is gone).
GLIM_CONFIG_SRC="${GLIM_CONFIG_SRC:-/glim/config}"
PRIOR_MAP_DIR="${PRIOR_MAP_DIR:-/tmp/dump}"
GLIM_MODE="${GLIM_MODE:-auto}"
GLIM_CONFIG_ACTIVE=/tmp/glim_config_active

# A valid GlobalMapping::save dump has a graph.bin/graph.txt at its root.
prior_map_exists() { [ -f "$PRIOR_MAP_DIR/graph.bin" ] || [ -f "$PRIOR_MAP_DIR/graph.txt" ]; }

case "$GLIM_MODE" in
  auto)                       prior_map_exists && GLIM_MODE=continue || GLIM_MODE=mapping ;;
  continue|cont)              GLIM_MODE=continue ;;
  localize|localise)
    echo "[container] NOTE: GLIM_MODE='$GLIM_MODE' is a deprecated alias for 'continue'."
    GLIM_MODE=continue ;;
  mapping|map)                GLIM_MODE=mapping ;;
  *) echo "[container] WARNING: unknown GLIM_MODE='$GLIM_MODE'; using auto"
     prior_map_exists && GLIM_MODE=continue || GLIM_MODE=mapping ;;
esac

# Work on a writable copy (the mounted /glim/config is read-only).
rm -rf "$GLIM_CONFIG_ACTIVE"
cp -r "$GLIM_CONFIG_SRC" "$GLIM_CONFIG_ACTIVE"

if [ "$GLIM_MODE" = "continue" ]; then
  echo "[container] Prior map found in $PRIOR_MAP_DIR -> CONTINUE mode (loading it and mapping onto it, as if GLIM was never turned off)."
  # Keep the normal mapping pipeline (odometry + sub + global mapping all on), but load the prior
  # dump and continue onto it: the loaded submaps come back as optimizable variables, the new session
  # relocalizes onto them (FPFH+RANSAC on the first new submap), and overlap-based loop closure locks
  # old and new poses together. The prior map is NEVER overwritten; the grown map is written to
  # $PRIOR_MAP_DIR/continued (kept under the mounted dump dir so it persists on the host).
  sed -i -E "s#(\"continue_from_map_path\"[[:space:]]*:[[:space:]]*)\"[^\"]*\"#\1\"$PRIOR_MAP_DIR\"#" \
    "$GLIM_CONFIG_ACTIVE/config_global_mapping_gpu.json"
  sed -i -E "s#(\"save_map_path\"[[:space:]]*:[[:space:]]*)\"[^\"]*\"#\1\"$PRIOR_MAP_DIR/continued\"#" \
    "$GLIM_CONFIG_ACTIVE/config_global_mapping_gpu.json"
else
  echo "[container] No prior map in $PRIOR_MAP_DIR -> MAPPING mode (map saved to $PRIOR_MAP_DIR on shutdown)."
fi

echo "[container] Starting GLIM ($GLIM_MODE)..."
"$@" --ros-args -p config_path:="$GLIM_CONFIG_ACTIVE" &
GLIM_PID=$!

# --- RViz (verification GUI) -------------------------------------------------
# Autolaunch RViz alongside GLIM so the predictor pose / odometry / decayed
# cloud are visible on the host X display. Needs the X11 socket mount + DISPLAY
# from docker-compose. Disable with ENABLE_RVIZ=0; override layout with RVIZ_CONFIG.
ENABLE_RVIZ="${ENABLE_RVIZ:-1}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$GLIM_CONFIG_SRC/glim_verify.rviz}"
if [ "$ENABLE_RVIZ" = "1" ]; then
  if [ -z "${DISPLAY:-}" ]; then
    echo "[container] ENABLE_RVIZ=1 but DISPLAY is unset; skipping RViz. (Pass DISPLAY + mount /tmp/.X11-unix.)"
  elif ! command -v rviz2 >/dev/null 2>&1; then
    echo "[container] ENABLE_RVIZ=1 but rviz2 is not installed in the image; skipping RViz."
  elif [ ! -f "$RVIZ_CONFIG" ]; then
    echo "[container] RViz config '$RVIZ_CONFIG' not found; launching RViz with defaults."
    rviz2 &
    RVIZ_PID=$!
  else
    echo "[container] Starting RViz ($RVIZ_CONFIG) on DISPLAY=$DISPLAY ..."
    rviz2 -d "$RVIZ_CONFIG" &
    RVIZ_PID=$!
  fi
fi

wait "$GLIM_PID"
STATUS=$?
shutdown
exit "$STATUS"
