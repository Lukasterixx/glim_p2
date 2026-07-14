# syntax=docker/dockerfile:1
# ------------------------------------------------------------------------------
# GLIM (this fork) on NVIDIA Jetson — JetPack 6.1 (R36.4.0), Orin.
#
# Mirrors the GLIM build recipe from P2Dingo's Dockerfile.jetson (same base image,
# OpenCV equivs mock, GTSAM 4.3a0 + gtsam_points + Iridescence from source with
# CUDA arch 87, CycloneDDS RMW) but is SCOPED TO GLIM ONLY — no go2_control_cpp,
# Livox driver, web server, or D1 arm. Crucially it builds THIS repo (COPY .)
# instead of cloning upstream koide3/glim, so you can test the fork on the dog.
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
# BUILD PHASE 1: Third-party libs (GTSAM, gtsam_points, Iridescence)
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

# 3. Iridescence (GLIM viewer). Built from source: the koide3 PPA has no arm64/L4T
#    packages. Build with viewer ON to keep config parity with P2Dingo (its GLIM
#    config lists libstandard_viewer.so / librviz_viewer.so). Run headless by
#    trimming extension_modules in your mounted config if the dog has no display.
RUN git clone --recursive https://github.com/koide3/iridescence /tmp/iridescence && \
    cd /tmp/iridescence && \
    mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/iridescence && \
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
# BUILD PHASE 3: Compilation (mirrors P2Dingo's strict ordering)
# ------------------------------------------------------------------------------
SHELL ["/bin/bash", "-c"]
WORKDIR $WS
RUN source /opt/ros/humble/setup.bash && \
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
        -DBUILD_WITH_VIEWER=ON \
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

# Default GLIM config = this repo's config/ (upstream defaults). For a real dog
# run, mount the robot-tuned glim_config over /glim/config (see docker-compose).
RUN mkdir -p /glim && cp -r $WS/src/glim/config /glim/config

# ------------------------------------------------------------------------------
# ENTRYPOINT
# ------------------------------------------------------------------------------
RUN cat > /usr/local/bin/entrypoint.sh <<'EOF'
#!/usr/bin/env bash
set -eo pipefail

# Source ROS + workspace.
source /opt/ros/humble/setup.bash
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

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /root/ws/install/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
# Default: run the forked GLIM node against the mounted/baked config.
CMD ["bash", "-lc", "ros2 run glim_ros glim_rosnode --ros-args -p config_path:=${GLIM_CONFIG_PATH}"]
