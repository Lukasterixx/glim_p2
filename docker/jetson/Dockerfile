# syntax=docker/dockerfile:1
# ------------------------------------------------------------------------------
# GLIM (this fork) on NVIDIA Jetson — JetPack 6.1 (R36.4.0), Orin.
#
# Mirrors the GLIM build recipe from P2Dingo's Dockerfile.jetson (same base image,
# OpenCV equivs mock, GTSAM 4.3a0 + gtsam_points from source with CUDA arch 87,
# CycloneDDS RMW, Livox MID360 driver) but is scoped to GLIM + LiDAR — no
# go2_control_cpp, web server, or D1 arm. GLIM is built HEADLESS (no viewer /
# Iridescence), exactly like P2Dingo runs it on the dog. Crucially it builds THIS
# repo (COPY .) instead of cloning upstream koide3/glim, so you can test the fork.
#
# Image name:     glim_p2_jetson   (cf. robot p2dingo / sim glim_cyclone)
# Container name: glim_p2_test     (see docker-compose.yaml)
#
# Build (context MUST be the repo root):
#   docker build -f docker/jetson/Dockerfile -t glim_p2_jetson .
# or:  docker compose -f docker/jetson/docker-compose.yaml build
# ------------------------------------------------------------------------------
FROM dustynv/ros:humble-desktop-l4t-r36.4.0

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

# ------------------------------------------------------------------------------
# ENVIRONMENT & PATHS
# ------------------------------------------------------------------------------
ENV WS=/root/ws
ENV GTSAM_PREFIX=/opt/gtsam_stack
ENV GTSAM_DIR=/root/gtsam
ENV GTSAM_POINTS_DIR=/root/gtsam_points

# ROS & DDS config — match the robot so this container interoperates with the
# already-running p2dingo container over host networking / CycloneDDS.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///root/cyclonedds_go2.xml
ENV ROS_DOMAIN_ID=0

# Compile parallelism (keep modest — Orin has limited RAM under heavy CUDA link).
ENV CMAKE_BUILD_PARALLEL_LEVEL=4

# Prioritize NVIDIA/local libs to prevent OpenCV version mixing.
ENV LD_LIBRARY_PATH=/lib:/usr/lib/aarch64-linux-gnu/nvidia:${LD_LIBRARY_PATH}

# Default GLIM config dir (baked from this repo's config/; override with a volume).
ENV GLIM_CONFIG_PATH=/glim/config

# ------------------------------------------------------------------------------
# SYSTEM SETUP: apt config & ROS key refresh
# ------------------------------------------------------------------------------
RUN echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries && \
    echo 'Acquire::http::Timeout "240";' >> /etc/apt/apt.conf.d/80-retries

RUN apt-get update || true && \
    apt-get install -y curl gnupg lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    rm -f /etc/apt/sources.list.d/ros2-latest.list || true

# ------------------------------------------------------------------------------
# FIX: Mock libopencv-dev to preserve NVIDIA-accelerated OpenCV
# ROS packages depend on libopencv-dev; installing the Ubuntu one would pull a
# non-CUDA OpenCV over the base image's NVIDIA build. Satisfy the dep with an
# empty equivs package instead. (Verbatim from P2Dingo's Dockerfile.jetson.)
# ------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y equivs && \
    echo "Section: misc" > opencv-dummy.control && \
    echo "Priority: optional" >> opencv-dummy.control && \
    echo "Standards-Version: 3.9.2" >> opencv-dummy.control && \
    echo "Package: libopencv-dev" >> opencv-dummy.control && \
    echo "Version: 4.5.4+dfsg-9ubuntu4" >> opencv-dummy.control && \
    echo "Provides: libopencv-core-dev, libopencv-flann-dev, libopencv-imgproc-dev, libopencv-imgcodecs-dev, libopencv-videoio-dev, libopencv-highgui-dev, libopencv-ml-dev, libopencv-features2d-dev, libopencv-calib3d-dev, libopencv-dnn-dev, libopencv-objdetect-dev, libopencv-photo-dev, libopencv-video-dev, libopencv-shape-dev, libopencv-stitching-dev, libopencv-superres-dev, libopencv-videostab-dev, libopencv-contrib-dev, libopencv-dev" >> opencv-dummy.control && \
    echo "Architecture: all" >> opencv-dummy.control && \
    echo "Description: Dummy package to satisfy ROS dependencies without removing NVIDIA OpenCV" >> opencv-dummy.control && \
    equivs-build opencv-dummy.control && \
    dpkg -i libopencv-dev_*.deb && \
    rm opencv-dummy.control libopencv-dev_*.deb

# ------------------------------------------------------------------------------
# DEPENDENCIES (GLIM + glim_ros2 build deps only)
# ------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential cmake wget unzip nano \
    python3-pip python3-rosdep python3-colcon-common-extensions \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-pcl-conversions ros-humble-pcl-ros \
    ros-humble-visualization-msgs \
    ros-humble-ament-cmake-auto \
    ros-humble-ament-lint-auto ros-humble-ament-lint-common \
    libpcl-dev libyaml-cpp-dev libeigen3-dev \
    libomp-dev libboost-all-dev libmetis-dev \
    libfmt-dev libspdlog-dev \
    libglm-dev libglfw3-dev libpng-dev libjpeg-dev \
    iproute2 iputils-ping \
 && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true && rosdep update

# ------------------------------------------------------------------------------
# BUILD PHASE 1: Third-party libs (GTSAM, gtsam_points) — headless, no Iridescence
# ------------------------------------------------------------------------------

# 1. GTSAM 4.3a0 (CUDA, no TBB, system Eigen) — matches P2Dingo's pin.
RUN git clone https://github.com/borglab/gtsam ${GTSAM_DIR} && \
    cd ${GTSAM_DIR} && git checkout 4.3a0 && \
    mkdir -p build && cd build && \
    cmake .. \
        -DCMAKE_INSTALL_PREFIX="${GTSAM_PREFIX}" \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DCMAKE_CUDA_ARCHITECTURES=87 \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF && \
    cmake --build . -- -j$(nproc) && \
    cmake --install . && \
    echo "${GTSAM_PREFIX}/lib" > /etc/ld.so.conf.d/gtsam.conf && ldconfig

# 2. gtsam_points (CUDA)
RUN git clone https://github.com/koide3/gtsam_points ${GTSAM_POINTS_DIR} && \
    cd ${GTSAM_POINTS_DIR} && \
    mkdir -p build && cd build && \
    cmake .. \
        -DCMAKE_INSTALL_PREFIX="${GTSAM_PREFIX}" \
        -DCMAKE_CUDA_ARCHITECTURES=87 \
        -DBUILD_WITH_CUDA=ON && \
    cmake --build . -- -j$(nproc) && \
    cmake --install . && \
    echo "${GTSAM_PREFIX}/lib" > /etc/ld.so.conf.d/gtsam_points.conf && ldconfig

# ------------------------------------------------------------------------------
# LIVOX DEPS + SDK
# ------------------------------------------------------------------------------
# Placed AFTER the (cached) GTSAM layers so adding LiDAR support doesn't invalidate
# the expensive gtsam / gtsam_points builds above.
#
# NOTE: No Iridescence / GLIM viewer — GLIM runs HEADLESS on the dog (built with
# BUILD_WITH_VIEWER=OFF below), exactly like P2Dingo. Visualize remotely via
# librviz_viewer.so (RViz markers over the network) instead.
ENV LIVOX_WS=/root/ws_livox
ENV LIVOX_SDK_PREFIX=/usr/local
ENV LIVOX_SDK_DIR=/root/Livox-SDK2

# rosidl toolchain for livox_ros_driver2's custom messages (CustomMsg/CustomPoint).
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-rosidl-typesupport-fastrtps-cpp \
    ros-humble-rosidl-typesupport-fastrtps-c \
 && rm -rf /var/lib/apt/lists/*

# Livox SDK2 (required by livox_ros_driver2) — built from source like P2Dingo.
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git ${LIVOX_SDK_DIR} && \
    cd ${LIVOX_SDK_DIR} && \
    mkdir -p build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX="${LIVOX_SDK_PREFIX}" && \
    make -j$(nproc) && \
    make install && \
    echo "${LIVOX_SDK_PREFIX}/lib" > /etc/ld.so.conf.d/livox_sdk.conf && \
    ldconfig

# ------------------------------------------------------------------------------
# BUILD PHASE 2: Workspace prep
# ------------------------------------------------------------------------------
WORKDIR $WS/src

# glim_ros2 provides the ROS node (glim_rosnode / package glim_ros). Upstream —
# only the glim core is forked.
RUN git clone https://github.com/koide3/glim_ros2.git ${WS}/src/glim_ros2
# vision_opencv rebuilt from source to fix the OpenCV version conflict against the
# base image's NVIDIA OpenCV (mirrors P2Dingo).
RUN git clone -b humble https://github.com/ros-perception/vision_opencv.git $WS/src/vision_opencv

# THE FORK: this repo becomes the `glim` package in the workspace.
# .dockerignore strips build/ install/ log/ docs/ docker/ from the context, so
# this copies a clean source tree.
COPY . $WS/src/glim

WORKDIR $WS
RUN source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true

# ------------------------------------------------------------------------------
# LIVOX MID360 DRIVER SOURCES (mirrors P2Dingo)
# ------------------------------------------------------------------------------
# Separate workspace ($LIVOX_WS), built later with the driver's own build.sh. The
# baked MID360_config.json + driver-only launch match P2Dingo's: the LiDAR is at
# 192.168.123.20 and the host NIC at 192.168.123.18 on the dog's Livox network.
# Edit these two heredocs (and rebuild) if your MID360 IP/host IP differ.
WORKDIR $LIVOX_WS/src
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git

RUN cat > $LIVOX_WS/src/livox_ros_driver2/config/MID360_config.json <<'EOF'
{
 "lidar_summary_info" : {
   "lidar_type": 8
 },
 "MID360": {
   "lidar_net_info" : {
     "cmd_data_port": 56100,
     "push_msg_port": 56200,
     "point_data_port": 56300,
     "imu_data_port": 56400,
     "log_data_port": 56500
   },
   "host_net_info" : {
     "cmd_data_ip" : "192.168.123.18",
     "cmd_data_port": 56101,
     "push_msg_ip": "192.168.123.18",
     "push_msg_port": 56201,
     "point_data_ip": "192.168.123.18",
     "point_data_port": 56301,
     "imu_data_ip" : "192.168.123.18",
     "imu_data_port": 56401,
     "log_data_ip" : "",
     "log_data_port": 56501
   }
 },
 "lidar_configs" : [
   {
     "ip" : "192.168.123.20",
     "pcl_data_type" : 1,
     "pattern_mode" : 0,
     "extrinsic_parameter" : {
       "roll": 0.0,
       "pitch": 0.0,
       "yaw": 0.0,
       "x": 0,
       "y": 0,
       "z": 0
     }
   }
 ]
}
EOF

RUN cat > $LIVOX_WS/src/livox_ros_driver2/launch_ROS2/mid360_driver_only.launch.py <<'EOF'
import os
from launch import LaunchDescription
from launch_ros.actions import Node

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-PointCloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # frequency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = os.path.join(cur_path, '../config')
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code},
]


def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
    )

    # No RVIZ node here – visualization will be done from your PC.
    return LaunchDescription([
        livox_driver
    ])
EOF

# ------------------------------------------------------------------------------
# BUILD PHASE 3: Compilation (mirrors P2Dingo's strict ordering)
# ------------------------------------------------------------------------------
SHELL ["/bin/bash", "-c"]

# Build the Livox ROS driver in its own workspace via the driver's build.sh
# (installs to $LIVOX_WS/install), exactly as P2Dingo does.
RUN source /opt/ros/humble/setup.bash && \
    cd $LIVOX_WS/src/livox_ros_driver2 && \
    ./build.sh humble

# NOTE: this base image's setup.bash doesn't export ROS_VERSION, but the fork's
# CMakeLists guards its ROS2 modules (incl. imu_prediction) on
# `if(DEFINED ENV{ROS_VERSION} AND $ENV{ROS_VERSION} EQUAL 2)`. With ROS_VERSION
# empty that if() errors ("Unknown arguments"), so pin it before colcon.
WORKDIR $WS
RUN source /opt/ros/humble/setup.bash && \
    export ROS_VERSION=2 && \
    export CMAKE_PREFIX_PATH="${GTSAM_PREFIX}:${CMAKE_PREFIX_PATH}" && \
    \
    echo "--- Building cv_bridge (early) ---" && \
    colcon build --packages-select cv_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source install/setup.bash && \
    \
    echo "--- Building GLIM core (this fork, CUDA) ---" && \
    colcon build \
      --packages-select glim \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_WITH_CUDA=ON \
        -DCMAKE_CUDA_ARCHITECTURES=87 \
        -DBUILD_WITH_VIEWER=OFF \
        -DBUILD_WITH_MARCH_NATIVE=OFF \
        -DBUILD_TESTING=OFF && \
    source install/setup.bash && \
    \
    echo "--- Fixing GLIM include path ---" && \
    export CPLUS_INCLUDE_PATH="$WS/install/glim/include:${CPLUS_INCLUDE_PATH}" && \
    \
    echo "--- Building glim_ros (CUDA) ---" && \
    colcon build \
      --packages-select glim_ros \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_WITH_CUDA=ON \
        -DCMAKE_CUDA_ARCHITECTURES=87 \
        -DBUILD_WITH_VIEWER=OFF \
        -DBUILD_WITH_MARCH_NATIVE=OFF \
        -DBUILD_TESTING=OFF

# Persist the include-path fix for runtime.
ENV CPLUS_INCLUDE_PATH="${WS}/install/glim/include:${CPLUS_INCLUDE_PATH}"

# ------------------------------------------------------------------------------
# RUNTIME CONFIG: bake CycloneDDS profile + a default GLIM config
# ------------------------------------------------------------------------------
# CycloneDDS profile matching the robot (large socket buffers for LiDAR clouds).
RUN cat > /root/cyclonedds_go2.xml <<'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <Internal>
            <SocketReceiveBufferSize min="10MB"/>
            <SocketSendBufferSize min="10MB"/>
        </Internal>
        <Tracing>
            <Verbosity>warning</Verbosity>
            <OutputFile>stdout</OutputFile>
        </Tracing>
    </Domain>
</CycloneDDS>
EOF

# Bake this repo's config/ as the default GLIM config at /glim/config (override the
# whole dir with a volume mount for a custom one). Two adjustments for this container:
#   - subscribe to the in-container MID360 driver's /livox/lidar + /livox/imu (repo
#     default is Ouster /os_cloud_node/*); frames + acc_scale stay auto-detected.
#   - headless: drop viewer-only extension modules (not built with viewer OFF),
#     keeping librviz_viewer (RViz over the network) + imu_prediction, like P2Dingo.
RUN mkdir -p /glim && cp -r $WS/src/glim/config /glim/config && \
    sed -i 's#"/os_cloud_node/imu"#"/livox/imu"#; s#"/os_cloud_node/points"#"/livox/lidar"#' /glim/config/config_ros.json && \
    sed -i '/libmemory_monitor.so/d; /libstandard_viewer.so/d' /glim/config/config_ros.json

# ------------------------------------------------------------------------------
# ENTRYPOINT
# ------------------------------------------------------------------------------
RUN cat > /usr/local/bin/entrypoint.sh <<'EOF'
#!/usr/bin/env bash
set -eo pipefail

# Source ROS + the Livox driver workspace + the GLIM workspace.
source /opt/ros/humble/setup.bash
if [[ -f /root/ws_livox/install/setup.bash ]]; then
  source /root/ws_livox/install/setup.bash
fi
if [[ -f /root/ws/install/setup.bash ]]; then
  source /root/ws/install/setup.bash
fi

# Prioritize NVIDIA/local libs.
export LD_LIBRARY_PATH=/lib:/usr/lib/aarch64-linux-gnu/nvidia:${LD_LIBRARY_PATH}

# Force-preload NVIDIA OpenCV + GTSAM to avoid the double-free from mixed OpenCV
# builds (mirrors P2Dingo). Resolve the .so version at runtime so this survives a
# base-image OpenCV bump.
preload=""
for lib in libopencv_core libopencv_highgui libopencv_imgproc; do
  f=$(ls /usr/lib/${lib}.so.* 2>/dev/null | head -n1 || true)
  [[ -n "$f" ]] && preload="${preload:+$preload:}$f"
done
[[ -f /opt/gtsam_stack/lib/libgtsam.so ]] && preload="${preload:+$preload:}/opt/gtsam_stack/lib/libgtsam.so"
export LD_PRELOAD="$preload"

# Best-effort large UDP buffers for LiDAR clouds (needs --privileged; ignore if not).
sysctl -w net.core.rmem_max=2147483647    >/dev/null 2>&1 || true
sysctl -w net.core.rmem_default=2147483647 >/dev/null 2>&1 || true
sysctl -w net.core.wmem_max=2147483647    >/dev/null 2>&1 || true
sysctl -w net.core.wmem_default=2147483647 >/dev/null 2>&1 || true

if [[ $# -eq 0 ]]; then
  set -- bash
fi
echo "[entrypoint] glim_p2 test container ready. Executing: $*"
exec "$@"
EOF
RUN chmod +x /usr/local/bin/entrypoint.sh

# Combined bring-up: MID360 driver + forked GPU GLIM (mirrors P2Dingo's
# run_with_livox.sh, trimmed to just LiDAR + GLIM).
RUN cat > /usr/local/bin/run_glim_livox.sh <<'EOF'
#!/usr/bin/env bash
set -eo pipefail

LAUNCH_PID=""
GLIM_PID=""

shutdown() {
  echo "[glim_p2] shutdown requested; stopping GLIM + Livox driver..."
  # SIGINT GLIM first so it serializes its map dump to /tmp/dump cleanly.
  [ -n "$GLIM_PID" ]   && kill -INT "$GLIM_PID"   2>/dev/null || true
  [ -n "$LAUNCH_PID" ] && kill -INT "$LAUNCH_PID" 2>/dev/null || true
  wait || true
}
trap 'shutdown; exit 0' INT TERM

# Large UDP buffers for the MID360 point stream (needs --privileged).
for k in rmem_max rmem_default wmem_max wmem_default; do
  sysctl -w net.core.$k=2147483647 >/dev/null 2>&1 || true
done

echo "[glim_p2] starting Livox MID360 driver (publishes /livox/lidar + /livox/imu)..."
ros2 launch livox_ros_driver2 mid360_driver_only.launch.py &
LAUNCH_PID=$!

# Give the driver a moment to spawn, then raise it to real-time priority.
sleep 5
DRIVER_PID=$(pgrep -f livox_ros_driver2_node | head -n1 || true)
if [ -n "$DRIVER_PID" ]; then
  if chrt -f -p 90 "$DRIVER_PID" 2>/dev/null; then
    echo "[glim_p2] real-time priority 90 set on livox driver (PID $DRIVER_PID)."
  else
    echo "[glim_p2] WARN: could not set RT priority (need --privileged / --cap-add SYS_NICE)."
  fi
else
  echo "[glim_p2] WARN: livox_ros_driver2_node not found; RT priority skipped."
fi

echo "[glim_p2] starting forked GPU GLIM (config: ${GLIM_CONFIG_PATH})..."
ros2 run glim_ros glim_rosnode --ros-args -p config_path:="${GLIM_CONFIG_PATH}" &
GLIM_PID=$!

wait
EOF
RUN chmod +x /usr/local/bin/run_glim_livox.sh

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /root/ws_livox/install/setup.bash' >> /root/.bashrc && \
    echo 'source /root/ws/install/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
# Default: bring up the MID360 driver + forked GPU GLIM together (standalone test
# on the dog). To run GLIM only (e.g. the robot's own driver is already up),
# override the command with:
#   ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/glim/config
CMD ["bash", "/usr/local/bin/run_glim_livox.sh"]
