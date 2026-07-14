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
# auto (default): if the prior-map dump already holds a saved map, localise
# within it; otherwise map fresh. Override with GLIM_MODE=mapping|localize.
GLIM_CONFIG_SRC="${GLIM_CONFIG_SRC:-/glim/config}"
PRIOR_MAP_DIR="${PRIOR_MAP_DIR:-/tmp/dump}"
GLIM_MODE="${GLIM_MODE:-auto}"
GLIM_CONFIG_ACTIVE=/tmp/glim_config_active

# A valid GlobalMapping::save dump has a graph.bin/graph.txt at its root.
prior_map_exists() { [ -f "$PRIOR_MAP_DIR/graph.bin" ] || [ -f "$PRIOR_MAP_DIR/graph.txt" ]; }

case "$GLIM_MODE" in
  auto)              prior_map_exists && GLIM_MODE=localize || GLIM_MODE=mapping ;;
  localize|localise) GLIM_MODE=localize ;;
  mapping|map)       GLIM_MODE=mapping ;;
  *) echo "[container] WARNING: unknown GLIM_MODE='$GLIM_MODE'; using auto"
     prior_map_exists && GLIM_MODE=localize || GLIM_MODE=mapping ;;
esac

# Work on a writable copy (the mounted /glim/config is read-only).
rm -rf "$GLIM_CONFIG_ACTIVE"
cp -r "$GLIM_CONFIG_SRC" "$GLIM_CONFIG_ACTIVE"

if [ "$GLIM_MODE" = "localize" ]; then
  echo "[container] Prior map found in $PRIOR_MAP_DIR -> LOCALISE mode (no new map is built)."
  # Odometry localises against the fixed prior map; sub/global mapping are turned
  # off so the prior dump is never overwritten while localizing.
  sed -i -E 's#("config_odometry"[[:space:]]*:[[:space:]]*)"[^"]*"#\1"config_odometry_localizer.json"#' \
    "$GLIM_CONFIG_ACTIVE/config.json"
  sed -i -E 's#("config_sub_mapping"[[:space:]]*:[[:space:]]*)"[^"]*"#\1"config_sub_mapping_passthrough.json"#' \
    "$GLIM_CONFIG_ACTIVE/config.json"
  sed -i -E 's#("enable_global_mapping"[[:space:]]*:[[:space:]]*)true#\1false#' \
    "$GLIM_CONFIG_ACTIVE/config_ros.json"
  # Keep the localizer's prior_map_path in sync with the mounted dump dir.
  sed -i -E "s#(\"prior_map_path\"[[:space:]]*:[[:space:]]*)\"[^\"]*\"#\1\"$PRIOR_MAP_DIR\"#" \
    "$GLIM_CONFIG_ACTIVE/config_odometry_localizer.json"
  echo "[container] Send an RViz '2D Pose Estimate' (/initialpose) to set the start pose."
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
