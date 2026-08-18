################################################################################
# Import ROS2 layer from aircraft.dockerfile, QGC layer from ground.dockerfile #
################################################################################
FROM transitional-ros2-qgc-image AS ros2-qgc-image

################################################################################
# Add Gazebo Sim ###############################################################
################################################################################
FROM ros2-qgc-image AS ros2-qgc-gz-image

# Gazebo Harmonic
# Based on https://gazebosim.org/docs/harmonic/install_ubuntu/
RUN apt update \
    && apt install -y --no-install-recommends \
        lsb-release gnupg \
    && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt update \
    && apt install -y --no-install-recommends \
        gz-harmonic ros-humble-ros-gzharmonic \
        libgz-transport13-* libgz-msgs10-dev \
        python3-gz-transport13 python3-gz-msgs10 \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*
# Run with $ gz sim

################################################################################
# Add PX4 ######################################################################
################################################################################
FROM ros2-qgc-gz-image AS ros2-qgc-gz-px4-image

# PX4 SITL (NOTE: install PX4 tools first to avoid conflicts with ArduPilot, build later to customize)
# Based on https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html
COPY /_github_clones/PX4-Autopilot /aas/github_apps/PX4-Autopilot
WORKDIR /aas/github_apps/PX4-Autopilot
RUN bash ./Tools/setup/ubuntu.sh --no-sim-tools \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

################################################################################
# Add ArduPilot ################################################################
################################################################################
FROM ros2-qgc-gz-px4-image AS ros2-qgc-gz-px4-ardupilot-image

# ArduPilot SITL (temporarily as arduuser, then re chown to root)
# Based on https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux
COPY /_github_clones/ardupilot /aas/github_apps/ardupilot
WORKDIR /aas/github_apps/ardupilot
RUN useradd -m -s /bin/bash arduuser \
    && echo "arduuser ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/arduuser && chmod 0440 /etc/sudoers.d/arduuser \
    && gosu arduuser git config --global --add safe.directory /aas/github_apps/ardupilot \
    && chown -R arduuser:arduuser /aas/github_apps/ardupilot \
    && USER=arduuser gosu arduuser bash ./Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
RUN gosu arduuser bash -c "cd /aas/github_apps/ardupilot && ./waf configure --board sitl && ./waf build" \
    && chown -R root:root /aas/github_apps/ardupilot
# Run with $ /aas/github_apps/ardupilot/build/sitl/bin/arducopter

# ArduPilot Gazebo Plugin
# Based on https://ardupilot.org/dev/docs/sitl-with-gazebo.html
COPY /_github_clones/ardupilot_gazebo /aas/github_apps/ardupilot_gazebo
WORKDIR /aas/github_apps/ardupilot_gazebo
RUN apt update \
    && apt install -y --no-install-recommends \
        rapidjson-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir --upgrade pip \
    && pip3 install --no-cache-dir --resume-retries 5 "numpy<2" mavproxy
ENV GZ_VERSION=harmonic
RUN mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc)

# Pre-build in the Docker image to speed up the first use of sim_vehicle.py
RUN /aas/github_apps/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter \
    && /aas/github_apps/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane

################################################################################
# Ephemeral stage to grab AAS PX4 custom airframes #############################
################################################################################
FROM ubuntu:22.04 AS airframe_filter_stage
COPY simulation/simulation_resources/aircraft_models/ /temp_folder
RUN mkdir /airframes
RUN find /temp_folder -type f -regex '.*/[0-9]+_.*' -exec cp {} /airframes/ \;

################################################################################
# Build PX4 SITL with AAS airframes using the airframe_filter_stage stage ######
################################################################################
FROM ros2-qgc-gz-px4-ardupilot-image AS ros2-qgc-gz-px4custom-ardupilot-image

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
# Add GStreamer, MAVLink, flight_review, wave simulation, ZeroMQ ###############
################################################################################
FROM ros2-qgc-gz-px4custom-ardupilot-image AS ros2-qgc-gz-px4custom-ardupilot-gst-logs-waves-zmq-image

# Add GStreamer packages to stream the cameras to the aircraft containers
RUN apt update \
    && apt install -y --no-install-recommends \
        gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \ 
        gstreamer1.0-libav gstreamer1.0-gl \
        python3-gi gir1.2-gst-plugins-base-1.0 gir1.2-gstreamer-1.0 \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# Add pymavlink and mavproxy to quickly inspect MAVLink streams
RUN pip3 install --no-cache-dir --upgrade pip \
    && pip3 install --no-cache-dir --resume-retries 5 pymavlink pyserial mavproxy future
# Check with $ python3 -c "import pymavlink; print(pymavlink.__version__)"

# Install https://github.com/PX4/flight_review to inspect PX4 SITL logs
RUN apt-get update && \
    apt-get install -y --no-install-recommends sqlite3 libfftw3-bin libfftw3-dev \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*
COPY /_github_clones/flight_review /aas/github_apps/flight_review
WORKDIR /aas/github_apps/flight_review/app
RUN python3 -m venv /px4fr-env \
    && /px4fr-env/bin/pip3 install --no-cache-dir --upgrade pip && \
    /px4fr-env/bin/pip3 install --no-cache-dir --resume-retries 5 -r requirements.txt

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
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install \
    --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON -DCMAKE_CXX_STANDARD=17"
# Build the GUI plugin
WORKDIR /aas/github_ws/src/asv_wave_sim/gz-waves/src/gui/plugins/waves_control
RUN mkdir build && cd build && cmake .. && make

# Install ZeroMQ
RUN apt-get update && \
    apt-get install -y --no-install-recommends libzmq3-dev \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir --upgrade pip \
    && pip3 install --no-cache-dir --resume-retries 5 pyzmq

################################################################################
# Copy AAS resources and build AAS ROS2 workspace ##############################
################################################################################
FROM ros2-qgc-gz-px4custom-ardupilot-gst-logs-waves-zmq-image AS simulation-dev-image

# Build the ROS 2 workspace
COPY simulation/simulation_ws/src /aas/simulation_ws/src
WORKDIR /aas/simulation_ws
RUN rosdep update
RUN apt update && rosdep install --from-paths src/ --ignore-src --rosdistro humble -y && apt clean && rm -rf /var/lib/apt/lists/*
# Explicitly use bash, not sh, to source and build the workspace
RUN bash -c "source /opt/ros/humble/setup.bash && (source /aas/github_ws/install/setup.bash || true) && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Copy resources and configuration files from this repository
COPY simulation/simulation_resources/ /aas/simulation_resources
RUN chmod +x /aas/simulation_resources/patches/create_ardupilot_drones_and_world.sh

# Copy QGC configuration (only for GND_CONTAINER=false)
COPY ground/ground_resources/patches/QGroundControl.ini /home/qgcuser/.config/QGroundControl/QGroundControl.ini

# Build gz_gst_bridge
WORKDIR /aas/simulation_resources/comms/gz_gst_bridge
RUN mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make

# Source the workspaces
RUN echo "source /aas/github_ws/install/setup.bash" >> /root/.bashrc \
    && echo "source /aas/simulation_ws/install/setup.bash" >> /root/.bashrc
# If needed (but already in .bashrc) $ source /opt/ros/humble/setup.bash && source /aas/github_ws/install/setup.bash && source /aas/simulation_ws/install/setup.bash

# Final config
WORKDIR /aas
COPY simulation/simulation.yml.erb /aas/simulation.yml.erb
COPY simulation/simulation_resources/patches/tmux.conf /root/.tmux.conf
ENTRYPOINT ["tmuxinator", "start", "-p", "/aas/simulation.yml.erb"]
