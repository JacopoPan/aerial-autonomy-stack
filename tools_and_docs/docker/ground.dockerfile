################################################################################
# Import ROS2 layer from aircraft.dockerfile ###################################
################################################################################
FROM transitional-ros2-image AS ros2-image

################################################################################
# Add QGroundControl ###########################################################
################################################################################
FROM ros2-image AS ros2-qgc-image

# QGroundControl (as qgcuser)
# Based on https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html
WORKDIR /
RUN useradd -m -s /bin/bash qgcuser \
    && usermod -aG dialout qgcuser
# RUN apt-get remove modemmanager -y
RUN apt update \
    && apt install -y --no-install-recommends \
        gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
        libfuse2 \
        libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* \
    && wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage && \
    chmod +x /QGroundControl-x86_64.AppImage && \
    /QGroundControl-x86_64.AppImage --appimage-extract && \
    rm /QGroundControl-x86_64.AppImage
# Run with $ gosu qgcuser /squashfs-root/AppRun

# Install wmctrl and xrandr to resize QGC window
RUN apt update \
    && apt install -y --no-install-recommends \
        wmctrl x11-xserver-utils \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

################################################################################
# Add GStreamer and MAVLink ####################################################
################################################################################
FROM ros2-qgc-image AS ros2-qgc-gst-mavlink-image

# Add GStreamer packages to stream the cameras to the aircraft containers
RUN apt update \
    && apt install -y --no-install-recommends \
        gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-libav \
        python3-gi gir1.2-gst-plugins-base-1.0 gir1.2-gstreamer-1.0 \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# Add pymavlink and mavproxy to quickly inspect MAVLink streams
RUN pip3 install --no-cache-dir --upgrade pip \
    && pip3 install --no-cache-dir --resume-retries 5 pymavlink pyserial mavproxy future
# Check with $ python3 -c "import pymavlink; print(pymavlink.__version__)"

# Install mavlink-router
RUN apt update \
    && apt install -y --no-install-recommends \
        git meson ninja-build pkg-config gcc g++ systemd \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*
COPY /_github_clones/mavlink-router /aas/github_apps/mavlink-router
WORKDIR /aas/github_apps/mavlink-router
RUN meson setup build . --buildtype=release \
    && ninja -C build \
    && ninja -C build install
# Check with $ mavlink-routerd --version

# Add MAVLink 2 C library
COPY /_github_clones/c_library_v2 /usr/local/include/mavlink/

################################################################################
# Copy AAS resources and build AAS ROS2 workspace ##############################
################################################################################
FROM ros2-qgc-gst-mavlink-image AS ground-dev-image

# Build the ROS 2 workspace
COPY ground/ground_ws/src /aas/ground_ws/src
WORKDIR /aas/ground_ws
RUN rosdep update
RUN apt update && rosdep install --from-paths src/ --ignore-src --rosdistro humble -y && apt clean && rm -rf /var/lib/apt/lists/*
# Explicitly use bash, not sh, to source and build the workspace
RUN bash -c "source /opt/ros/humble/setup.bash && (source /aas/github_ws/install/setup.bash || true) && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Copy resources and configuration files from this repository
COPY ground/ground_resources/ /aas/ground_resources
COPY ground/ground_resources/patches/QGroundControl.ini /home/qgcuser/.config/QGroundControl/QGroundControl.ini

# Copy sensor configuration
COPY simulation/simulation_resources/aircraft_models/sensor_config.yaml /aas/ground_resources/sensor_config.yaml

# Source the workspaces
RUN echo "source /aas/ground_ws/install/setup.bash" >> /root/.bashrc
# If needed (but already in .bashrc) $ source /opt/ros/humble/setup.bash && source /aas/ground_ws/install/setup.bash

# Final config
WORKDIR /aas
COPY ground/ground.yml.erb /aas/ground.yml.erb
COPY simulation/simulation_resources/patches/tmux.conf /root/.tmux.conf
ENTRYPOINT ["tmuxinator", "start", "-p", "/aas/ground.yml.erb"]
