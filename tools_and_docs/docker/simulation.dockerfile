################################################################################
# Import ROS2 layer from aircraft.dockerfile, GCS layer from ground.dockerfile #
################################################################################
FROM transitional-ros2-gcs-image AS ros2-gcs-image

################################################################################
# Add Gazebo Sim ###############################################################
################################################################################
FROM ros2-gcs-image AS ros2-gcs-gz-image

# Gazebo Harmonic
# Based on https://gazebosim.org/docs/harmonic/install_ubuntu/
RUN apt update \
    && apt install -y --no-install-recommends \
        lsb-release gnupg \
    && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt update \
    && apt install -y --no-install-recommends \
        gz-harmonic ros-jazzy-ros-gz-bridge \
        libgz-transport13-* libgz-msgs10-dev \
        python3-gz-transport13 python3-gz-msgs10 \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*
# Keep OSRF's gz sim visible after sourcing ROS (gz_tools_vendor replaces the default config path)
ENV GZ_CONFIG_PATH=/usr/share/gz
# Run with $ gz sim

################################################################################
# Add PX4 ######################################################################
################################################################################
FROM ros2-gcs-gz-image AS ros2-gcs-gz-px4-image

# PX4 SITL (NOTE: install PX4 tools first to avoid conflicts with ArduPilot, build later to customize)
# Based on https://docs.px4.io/v1.17/en/dev_setup/dev_env_linux_ubuntu
COPY /_github_clones/PX4-Autopilot /aas/github_apps/PX4-Autopilot
WORKDIR /aas/github_apps/PX4-Autopilot
# pandas>=3.1 requires numpy>=2, constraining numpy<2 installs pandas 3.0.6, keeping system apt numpy 1.26.4 (revise after Ubuntu 24)
RUN echo "numpy<2.0.0" > /etc/pip_constraints.txt \
    && PIP_CONSTRAINT=/etc/pip_constraints.txt bash ./Tools/setup/ubuntu.sh --no-sim-tools \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

################################################################################
# Add ArduPilot ################################################################
################################################################################
FROM ros2-gcs-gz-px4-image AS ros2-gcs-gz-px4-ardupilot-image

# ArduPilot SITL (temporarily as arduuser, then re chown to root)
# Based on https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux
# PIP_ONLY_BINARY=wxpython keeps apt's python3-wxgtk4.0 (for MAVProxy's console and map) instead of compiling wxPython from source (PyPI has no Linux wheels)
COPY /_github_clones/ardupilot /aas/github_apps/ardupilot
WORKDIR /aas/github_apps/ardupilot
RUN useradd -m -s /bin/bash arduuser \
    && echo "arduuser ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/arduuser && chmod 0440 /etc/sudoers.d/arduuser \
    && gosu arduuser git config --global --add safe.directory /aas/github_apps/ardupilot \
    && chown -R arduuser:arduuser /aas/github_apps/ardupilot \
    && USER=arduuser PIP_ONLY_BINARY=wxpython gosu arduuser bash ./Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
RUN gosu arduuser bash -c "cd /aas/github_apps/ardupilot && ./waf configure --board sitl && ./waf copter plane" \
    && chown -R root:root /aas/github_apps/ardupilot
# Run with $ /aas/github_apps/ardupilot/build/sitl/bin/arducopter
# Link the MAVProxy installed by install-prereqs-ubuntu.sh in ArduPilot's venv (isolated from the system numpy/OpenCV), e.g. to attach a console and map to a running SITL
RUN ln -s /home/arduuser/venv-ardupilot/bin/mavproxy.py /usr/local/bin/mavproxy.py

# ArduPilot Gazebo Plugin
# Based on https://ardupilot.org/dev/docs/sitl-with-gazebo.html
# Patched to wait for the first IMU message before the lock-step exchange (otherwise a SITL connected before Gazebo's first step deadlocks)
COPY /_github_clones/ardupilot_gazebo /aas/github_apps/ardupilot_gazebo
COPY simulation/simulation_resources/patches/ardupilot_gazebo-imu-lockstep.patch /aas/github_apps/ardupilot_gazebo-imu-lockstep.patch
WORKDIR /aas/github_apps/ardupilot_gazebo
RUN apt update \
    && apt install -y --no-install-recommends \
        rapidjson-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*
ENV GZ_VERSION=harmonic
RUN git apply ../ardupilot_gazebo-imu-lockstep.patch && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc)

################################################################################
# Add GStreamer, wave simulation, ZeroMQ #######################################
################################################################################
FROM ros2-gcs-gz-px4-ardupilot-image AS ros2-gcs-gz-px4-ardupilot-gst-waves-zmq-image

# Add GStreamer packages to stream the cameras to the aircraft containers
RUN apt update \
    && apt install -y --no-install-recommends \
        gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \ 
        gstreamer1.0-libav gstreamer1.0-gl \
        python3-gi gir1.2-gst-plugins-base-1.0 gir1.2-gstreamer-1.0 \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# Build the Gazebo wave plugin in github_ws/
# Based on https://github.com/srmainwaring/asv_wave_sim/blob/master/README.md
RUN apt-get update && \
    apt-get install -y --no-install-recommends libcgal-dev libfftw3-dev \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*
COPY /_github_clones/asv_wave_sim /aas/github_ws/src/asv_wave_sim
# Patch materials paths in waves/model.sdf
RUN sed -i 's|>materials/|>models://waves/materials/|g' /aas/github_ws/src/asv_wave_sim/gz-waves-models/world_models/waves/model.sdf
WORKDIR /aas/github_ws
# Explicitly use bash, not sh, to source and build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install \
    --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF -DCMAKE_CXX_STANDARD=17"
# Build the GUI plugin
WORKDIR /aas/github_ws/src/asv_wave_sim/gz-waves/src/gui/plugins/waves_control
RUN mkdir build && cd build && cmake .. && make

# Install ZeroMQ
RUN apt-get update && \
    apt-get install -y --no-install-recommends libzmq3-dev \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir --retries 5 pyzmq

################################################################################
# Add Betaflight with Gazebo SITL Plugin #######################################
################################################################################
FROM ros2-gcs-gz-px4-ardupilot-gst-waves-zmq-image AS ros2-gcs-gz-px4-ardupilot-gst-waves-zmq-betaflight-image

# Betaflight SITL and its Gazebo plugin
# Patched for multiple drones (UDP/TCP port offsets, MAVLink system IDs, per-drone IMU), sensor frame conventions, simulated GPS noise, MAVLink telemetry, and MSP2_POSITION_ESTIMATE
COPY /_github_clones/betaflight /aas/github_apps/betaflight
COPY simulation/simulation_resources/patches/betaflight-2026.6.2.patch /aas/github_apps/betaflight-2026.6.2.patch
WORKDIR /aas/github_apps/betaflight
RUN git apply ../betaflight-2026.6.2.patch && make TARGET=SITL
COPY /_github_clones/aeroloop_gazebo /aas/github_apps/aeroloop_gazebo
COPY simulation/simulation_resources/patches/aeroloop_gazebo-gz.patch /aas/github_apps/aeroloop_gazebo-gz.patch
WORKDIR /aas/github_apps/aeroloop_gazebo
RUN git apply ../aeroloop_gazebo-gz.patch && mkdir plugins/build && cd plugins/build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
# Run with $ /aas/github_apps/betaflight/obj/main/betaflight_SITL.elf

################################################################################
# Ephemeral stage to grab AAS PX4 custom airframes #############################
################################################################################
FROM ubuntu:24.04 AS airframe_filter_stage
COPY simulation/simulation_resources/aircraft_models/ /temp_folder
RUN mkdir /airframes
RUN find /temp_folder -type f -regex '.*/[0-9]+_.*' -exec cp {} /airframes/ \;

################################################################################
# Build PX4 SITL with AAS airframes using the airframe_filter_stage stage ######
################################################################################
FROM ros2-gcs-gz-px4-ardupilot-gst-waves-zmq-betaflight-image AS ros2-gcs-gz-px4custom-ardupilot-gst-waves-zmq-betaflight-image

# Apply PX4 patch (DDS Agent on custom IP, ...) created with $ git diff > ../px4-v1.17.0.patch
COPY simulation/simulation_resources/patches/px4-v1.17.0.patch /aas/github_apps/px4-v1.17.0.patch
WORKDIR /aas/github_apps/PX4-Autopilot
RUN git apply ../px4-v1.17.0.patch

# Replace dds_topics.yaml with custom topics
COPY simulation/simulation_resources/patches/dds_topics.yaml /aas/github_apps/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml

# Add PX4 Airframes ROMFS
COPY --from=airframe_filter_stage /airframes/ /aas/github_apps/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
WORKDIR /aas/github_apps/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes
RUN rm -f CMakeLists.txt && \
    echo "px4_add_romfs_files(" >> CMakeLists.txt && \
    find ./ -type f -printf '%f\n' | sed 's/^/  /' >> CMakeLists.txt && \
    echo ")" >> CMakeLists.txt

# Build PX4 SITL
WORKDIR /aas/github_apps/PX4-Autopilot
RUN make px4_sitl
# Run with $ /aas/github_apps/PX4-Autopilot/build/px4_sitl_default/bin/px4

################################################################################
# Copy AAS resources and build AAS ROS2 workspace ##############################
################################################################################
FROM ros2-gcs-gz-px4custom-ardupilot-gst-waves-zmq-betaflight-image AS simulation-manifests

# Keep only the package.xml files, so rosdep re-runs only when dependencies change (for offline builds)
COPY simulation/simulation_ws/src /src
RUN find /src -type f ! -name package.xml -delete

FROM ros2-gcs-gz-px4custom-ardupilot-gst-waves-zmq-betaflight-image AS simulation-dev-image

# Install the ROS 2 workspace dependencies
COPY --from=simulation-manifests /src /aas/simulation_ws/src
WORKDIR /aas/simulation_ws
RUN rosdep update
RUN apt update && rosdep install --from-paths src/ --ignore-src --rosdistro jazzy -y && apt clean && rm -rf /var/lib/apt/lists/*

# Build the ROS 2 workspace
COPY simulation/simulation_ws/src /aas/simulation_ws/src
# Explicitly use bash, not sh, to source and build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && (source /aas/github_ws/install/setup.bash || true) && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Copy resources and configuration files from this repository
COPY simulation/simulation_resources/ /aas/simulation_resources
RUN chmod +x /aas/simulation_resources/patches/create_*_drones_and_world.sh

# Copy QGC configuration (only for GND_CONTAINER=false)
COPY ground/ground_resources/patches/QGroundControl.ini /home/qgcuser/.config/QGroundControl/QGroundControl.ini
# Copy the Betaflight App launcher (only for AUTOPILOT=betaflight and GND_CONTAINER=false)
COPY ground/ground_resources/scripts/betaflight_app.py /aas/ground_resources/scripts/betaflight_app.py

# Build gz_gst_bridge
WORKDIR /aas/simulation_resources/comms/gz_gst_bridge
RUN mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make

# Source the workspaces
RUN echo "source /aas/github_ws/install/local_setup.bash" >> /root/.bashrc \
    && echo "source /aas/simulation_ws/install/local_setup.bash" >> /root/.bashrc
# If needed (but already in .bashrc) $ source /opt/ros/jazzy/setup.bash && source /aas/github_ws/install/setup.bash && source /aas/simulation_ws/install/setup.bash

# Final config
WORKDIR /aas
COPY simulation/simulation.yml.erb /aas/simulation.yml.erb
COPY simulation/simulation_resources/patches/tmux.conf /root/.tmux.conf
ENTRYPOINT ["tmuxinator", "start", "-p", "/aas/simulation.yml.erb"]
