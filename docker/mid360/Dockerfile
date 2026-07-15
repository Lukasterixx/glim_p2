FROM koide3/glim_ros2:humble_cuda12.2

# Use bash (login) so `source`/ROS setup scripts work in RUN steps.
SHELL ["/bin/bash", "-lc"]

# Install CycloneDDS to match the known-good GLIM Docker setup.
RUN apt-get update && \
    apt-get install -y ros-humble-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Match docker-compose.yaml unless explicitly overridden.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# CUDA arch of the build/target GPU. 89 = RTX 4080 (Ada, compute cap 8.9).
ARG CUDA_ARCH=89

# ===========================================================================
# LAYER ORDER: everything that rarely changes (gtsam_points, iridescence, the
# whole Livox stack) is built FIRST, and the glim source COPY + build is LAST.
# That way editing glim code only invalidates the final glim layers (~1 min);
# the expensive gtsam_points/Livox layers stay cached across code edits.
# ===========================================================================

# ---------------------------------------------------------------------------
# 1. Third-party libs. The base image ships gtsam_points 1.2.0, but this forked
#    glim_p2 (v1.2.2) requires gtsam_points >= 1.2.2, so rebuild it from source.
#    Iridescence is rebuilt for newer guik/glk APIs used by glim's standard_viewer.
# ---------------------------------------------------------------------------

# 1a. gtsam_points (CUDA), pinned via the thirdparty/gtsam_points submodule
#     (v1.2.2) -> overwrites the stale 1.2.0 in /usr/local. Vendored so it can
#     never drift from the version this glim_p2 needs. COPYing only this path keeps
#     the expensive build cached across glim code edits.
COPY thirdparty/gtsam_points /root/gtsam_points
RUN cd /root/gtsam_points && \
    cmake -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DBUILD_WITH_CUDA=ON \
      -DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCH} && \
    cmake --build build -- -j$(nproc) && \
    cmake --install build && \
    ldconfig

# 1b. Iridescence (OpenGL viewer lib) from source -> overwrites the stale one in /usr/local.
RUN rm -rf /tmp/iridescence && \
    git clone --recursive https://github.com/koide3/iridescence /tmp/iridescence && \
    cd /tmp/iridescence && \
    cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build build -- -j$(nproc) && \
    cmake --install build && \
    rm -rf /tmp/iridescence && \
    ldconfig

# ---------------------------------------------------------------------------
# 2. Livox MID360 driver. The base image ships NEITHER the Livox SDK2 nor the
#    livox_ros_driver2 ROS2 node, so build both from source. Built BEFORE the
#    glim COPY so glim code edits never invalidate this stack.
# ---------------------------------------------------------------------------
ENV LIVOX_WS=/root/ws_livox
ENV LIVOX_SDK_DIR=/root/Livox-SDK2
ENV LIVOX_SDK_PREFIX=/usr/local

# 2a. Livox SDK2 (low-level C++ SDK) -> /usr/local
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git ${LIVOX_SDK_DIR} && \
    cd ${LIVOX_SDK_DIR} && mkdir -p build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX="${LIVOX_SDK_PREFIX}" && \
    make -j$(nproc) && make install && \
    echo "${LIVOX_SDK_PREFIX}/lib" >/etc/ld.so.conf.d/livox_sdk.conf && \
    ldconfig

# 2b. livox_ros_driver2 build dependency: PCL (its CMakeLists does find_package(PCL)).
#     Also install rviz2 here so run_with_livox.sh can autolaunch the verification GUI
#     (see docker/mid360/config/glim_verify.rviz). Same pre-glim layer keeps the cache warm.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-humble-pcl-ros ros-humble-pcl-conversions libpcl-dev \
      ros-humble-rviz2 && \
    rm -rf /var/lib/apt/lists/*

# 2c. livox_ros_driver2 ROS2 node, built into its own overlay workspace. Copy the
#     MID360 network config (lidar 192.168.123.20, host 192.168.123.31) and the
#     driver-only launch into the package before build so they are installed.
#     build.sh exits 0 even when colcon fails, so assert the node binary exists.
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git ${LIVOX_WS}/src/livox_ros_driver2
COPY livox/MID360_config.json ${LIVOX_WS}/src/livox_ros_driver2/config/MID360_config.json
COPY livox/mid360_driver_only.launch.py ${LIVOX_WS}/src/livox_ros_driver2/launch_ROS2/mid360_driver_only.launch.py
RUN source /opt/ros/humble/setup.bash && \
    cd ${LIVOX_WS}/src/livox_ros_driver2 && \
    ./build.sh humble && \
    test -f ${LIVOX_WS}/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node

# 2d. Startup wrapper: bring up the Livox driver, then GLIM (see docker-compose).
COPY docker/run_with_livox.sh /usr/local/bin/run_with_livox.sh
RUN chmod +x /usr/local/bin/run_with_livox.sh && \
    echo 'source /root/ws_livox/install/setup.bash' >> /root/.bashrc

# ---------------------------------------------------------------------------
# 3. glim (core) + glim_ros2. LAST so iterating on glim code only re-runs these
#    layers; everything above stays cached.
# ---------------------------------------------------------------------------

# 3a. glim_ros2 is VENDORED in-tree at thirdparty/glim_ros2 (pinned to upstream
#     commit fc72f46 + the publish_tf rviz_viewer patch, committed directly — see
#     thirdparty/glim_ros2/VENDORING.md). COPY only that path first so glim_ros2
#     stays cached across glim-core edits.
#     WORKDIR must be a dir the rm below does NOT delete — the base image's WORKDIR
#     is /root/ros2_ws/src/glim, which we remove here, so pin it to /root/ros2_ws
#     first (otherwise the shell's CWD vanishes mid-RUN).
WORKDIR /root/ros2_ws
RUN rm -rf /root/ros2_ws/src/glim /root/ros2_ws/src/glim_ros2
COPY thirdparty/glim_ros2 /root/ros2_ws/src/glim_ros2

# The glim source — the layer you actually iterate on. Drop the nested vendored
# copy of glim_ros2 so colcon doesn't discover two `glim_ros` packages.
COPY . /root/ros2_ws/src/glim
RUN rm -rf /root/ros2_ws/src/glim/thirdparty/glim_ros2

# 3b. Build glim core first, then glim_ros (glim_ros2's package name), so the ROS
#     node + patched rviz_viewer + libodometry_estimation_external.so land in
#     /root/ros2_ws/install (already on AMENT_PREFIX_PATH / LD_LIBRARY_PATH).
RUN source /opt/ros/humble/setup.bash && \
    export MAKEFLAGS="-j$(nproc)" && \
    cd /root/ros2_ws && \
    colcon build --packages-select glim \
      --parallel-workers $(nproc) \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCH} && \
    source install/setup.bash && \
    export CPLUS_INCLUDE_PATH="/root/ros2_ws/install/glim/include:${CPLUS_INCLUDE_PATH}" && \
    colcon build --packages-select glim_ros \
      --parallel-workers $(nproc) \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCH}

# Persist the include-path fix for runtime.
ENV CPLUS_INCLUDE_PATH="/root/ros2_ws/install/glim/include:${CPLUS_INCLUDE_PATH}"
