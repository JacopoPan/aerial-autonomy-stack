# Use an ARG to selectively build or skip the advanced odometry, SLAM packages
ARG BUILD_ADVANCED_ODOM=true
################################################################################
# Pick amd64 (for simulation) or arm64 (on Jetson) image from the NGC Catalog ##
################################################################################
FROM nvcr.io/nvidia/cuda:13.3.1-cudnn-runtime-ubuntu24.04 AS base_amd64
FROM nvcr.io/nvidia/cuda:13.3.1-tensorrt-devel-ubuntu24.04 AS base_arm64

################################################################################
# Add tools and ROS2 ###########################################################
################################################################################
FROM base_${TARGETARCH} AS ros2-image

# See: https://forums.developer.nvidia.com/t/cuda-13-2-1-runtime-image-cant-run-on-r39-2-sample-rootfs-for-orin-nano/372683
ENV NVIDIA_DISABLE_REQUIRE=1

# Tell apt (and other Debian tools) not to prompt for user input during package installs
ENV DEBIAN_FRONTEND=noninteractive

# Ubuntu 24 blocks system-wide pip installs by default, --break-system-packages allows global pip installs
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# Install general use tools
RUN apt update \
    && apt install -y --no-install-recommends \
        wget gosu htop vim ruby tmux xclip net-tools iproute2 iputils-ping netcat-openbsd \
        python3-pip python3-venv \
        mesa-utils \
    && gem install tmuxinator \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Jazzy
# Based on https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
ENV LANG=en_US.UTF-8
RUN apt update \
    && apt install -y --no-install-recommends \
        locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && apt install -y --no-install-recommends \
        software-properties-common curl \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \
        $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt update \
    && apt install -y --no-install-recommends \
        ros-jazzy-desktop ros-dev-tools \
        ros-jazzy-bondcpp ros-jazzy-ament-cmake-clang-format \
        ros-jazzy-vision-msgs \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* \
    && echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && rosdep init

# Install Zenoh ROS2 bridge
RUN curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" | tee -a /etc/apt/sources.list > /dev/null \
    && apt-get update && \
    apt-get install -y --no-install-recommends zenoh-bridge-ros2dds \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

################################################################################
# Add PX4 messages #############################################################
################################################################################
FROM ros2-image AS ros2-px4msgs-image

# Build PX4 messages
COPY /_github_clones/px4_msgs /aas/github_ws/src/px4_msgs
WORKDIR /aas/github_ws
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro jazzy -y && apt clean && rm -rf /var/lib/apt/lists/*
# Explicitly use bash, not sh, to source and build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

################################################################################
# Add uXRCE-DDS agent ##########################################################
################################################################################
FROM ros2-px4msgs-image AS ros2-px4msgs-dds-image

# XRCE-DDS
# Based on https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html#installing-the-agent-standalone
COPY /_github_clones/Micro-XRCE-DDS-Agent /aas/github_apps/Micro-XRCE-DDS-Agent
WORKDIR /aas/github_apps/Micro-XRCE-DDS-Agent
RUN mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    ldconfig
# Run with $ MicroXRCEAgent udp4 -p 8888

################################################################################
# Add MAVROS ###################################################################
################################################################################
FROM ros2-px4msgs-dds-image AS ros2-px4msgs-dds-mavros-image

# MAVROS
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*
# Re-try install_geographiclib_datasets.sh if egm96-5.pgm was not downloaded (timeout is ~10x the time needed on a healthy link)
RUN for i in 1 2 3; do \
        rm -f /usr/share/GeographicLib/geoids/egm96-5*; \
        timeout 180 /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh; \
        test -r /usr/share/GeographicLib/geoids/egm96-5.pgm && break; \
        sleep 5; \
    done && test -r /usr/share/GeographicLib/geoids/egm96-5.pgm
# Run with $ ros2 launch mavros apm.launch fcu_url:=[URI]

################################################################################
# Add GStreamer, OpenCV, and Ultralytics YOLO ##################################
################################################################################
FROM ros2-px4msgs-dds-mavros-image AS ros2-px4msgs-dds-mavros-yolo-image

# In Ubuntu 24, package python3-numpy is on version 1.26.4, check with $ dpkg -l | grep python3-numpy
# We constrain pip install to <2.0.0 for system Python's OpenCV ABI compatibility
# Check with $ python3 -c "import numpy; print(numpy.__version__)"
RUN echo "numpy<2.0.0" > /etc/pip_constraints.txt
ENV PIP_CONSTRAINT=/etc/pip_constraints.txt
# Point the GCC compiler to the new pip NumPy headers (instead of the apt ones) to prevent ROS2 colcon from crashing
ENV CPATH=/usr/local/lib/python3.12/dist-packages/numpy/core/include

# Add GStreamer, Python OpenCV packages
RUN apt update \
    && apt install -y --no-install-recommends \
        gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav \
        python3-opencv \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# Install YOLO and ONNX in virtual environment /yolo-env/
# See https://github.com/ultralytics/ultralytics/blob/main/README.md and https://onnxruntime.ai/getting-started
RUN python3 -m venv /yolo-env \
    && /yolo-env/bin/pip3 install --no-cache-dir --upgrade pip && \
    /yolo-env/bin/pip3 install --no-cache-dir --retries 5 ultralytics onnx
# Check YOLO with $ /yolo-env/bin/python3 -c "import ultralytics; print(ultralytics.__version__)"
# NOTE: the venv avoids shadowing the system Python's OpenCV (with GStreamer support) with a newer one without GStreamer support
# Check with $ python3 -c "import cv2; print(cv2.getBuildInformation())"
# Versus $ /yolo-env/bin/python3 -c "import cv2; print(cv2.getBuildInformation())"

################################################################################
# amd64 stage for ONNX Runtime GPU: from wheel for CUDA support in simulation ##
# Mutually exclusive with the next stage #######################################
################################################################################
FROM ros2-px4msgs-dds-mavros-yolo-image AS image-with-hardware-specific-ort_amd64
# Add ONNX Runtime with GPU (CUDA) support for system Python
RUN pip3 install --no-cache-dir --retries 5 onnxruntime-gpu
# Check with $ python3 -c "import onnxruntime as ort; print(ort.__version__); print(ort.get_available_providers())"

################################################################################
# arm64 stage for ONNX Runtime GPU: from source for TensorRT support on Jetson #
# Mutually exclusive with the previous stage ###################################
################################################################################
FROM ros2-px4msgs-dds-mavros-yolo-image AS image-with-hardware-specific-ort_arm64
# Build ONNX Runtime from source with Jetson (TensorRT) support for system Python
# Based on https://onnxruntime.ai/docs/build/eps.html#nvidia-jetson-tx1tx2nanoxavierorin
# CMAKE_CUDA_ARCHITECTURES=87 based on: https://developer.nvidia.com/cuda-gpus
# Use CMAKE_CUDA_ARCHITECTURES=native if running within the container
# WARNING: this step takes up to 45'
COPY /_github_clones/onnxruntime /aas/github_apps/onnxruntime
RUN apt update && \
    apt install -y --no-install-recommends \
        build-essential software-properties-common libopenblas-dev \
        libpython3.12-dev python3-pip python3-dev python3-setuptools python3-wheel && \
    pip3 install --no-cache-dir --upgrade "cmake>=3.28" && \
    cd /aas/github_apps/onnxruntime/ && \
    CUDACXX="/usr/local/cuda/bin/nvcc" ./build.sh --config Release --update --build --parallel --build_wheel \
        --use_tensorrt --cuda_home /usr/local/cuda --cudnn_home /usr/lib/aarch64-linux-gnu \
        --tensorrt_home /usr/lib/aarch64-linux-gnu \
        --skip_tests --cmake_extra_defines 'CMAKE_CUDA_ARCHITECTURES=87' \
        'onnxruntime_BUILD_UNIT_TESTS=OFF' 'onnxruntime_USE_FLASH_ATTENTION=OFF' \
        'onnxruntime_USE_MEMORY_EFFICIENT_ATTENTION=OFF' \
        'CMAKE_POLICY_VERSION_MINIMUM=3.5' \
        --allow_running_as_root && \
    cd /aas/github_apps/onnxruntime/build/Linux/Release/dist && \
    pip3 install onnxruntime_gpu-*-linux_aarch64.whl && \
    cd /aas/github_apps/onnxruntime/build/Linux/Release && \
    sudo make install && \
    sudo ldconfig && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*
ENV PYTHONPATH=/aas/github_apps/onnxruntime/build/Linux/Release
# Check with $ python3 -c "import onnxruntime as ort; print(ort.__version__); print(ort.get_available_providers())"

# Also install DeepStream 9.1 on Orin to use NVIDIA accelerated GStreamer preprocessing (e.g. nvdewarper)
# Based on https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_Installation.html
WORKDIR /
RUN apt update \
    && apt install -y --no-install-recommends \
        libssl3 libssl-dev \
        libgstreamer1.0-0 libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav \
        libgstrtspserver-1.0-0 libjansson4 libyaml-cpp-dev libmosquitto1 \
    && curl -LO 'https://github.com/NVIDIA/DeepStream/releases/download/v9.1.0/deepstream-9.1_9.1.0-1_arm64.deb' \
    && apt-get install -y ./deepstream-9.1_9.1.0-1_arm64.deb \
    && rm -f deepstream-9.1_9.1.0-1_arm64.deb \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

################################################################################
# Add odometry packages ########################################################
################################################################################
FROM image-with-hardware-specific-ort_${TARGETARCH} AS ros2-px4msgs-dds-mavros-yolo-ort-simple-odom-image

# Install the Livox SDK (SuperOdom requirement)
COPY /_github_clones/Livox-SDK2 /aas/github_apps/Livox-SDK2
WORKDIR /aas/github_apps/Livox-SDK2
RUN mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# Install the Livox ROS2 driver (SuperOdom requirement), based on https://github.com/Livox-SDK/livox_ros_driver2/blob/master/README.md
# And https://github.com/Livox-SDK/livox_ros_driver2/blob/master/build.sh
COPY /_github_clones/livox_ros_driver2 /aas/github_ws/src/livox_ros_driver2
WORKDIR /aas/github_ws/
RUN cp -f src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml \
    && cp -rf src/livox_ros_driver2/launch_ROS2 src/livox_ros_driver2/launch
# Explicitly use bash, not sh, to source and build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy -DCMAKE_BUILD_TYPE=Release"

# Install KISS-ICP, based on https://github.com/PRBonn/kiss-icp/blob/main/README.md
RUN pip3 install --no-cache-dir --upgrade "cmake>=3.24"
COPY /_github_clones/kiss-icp /aas/github_ws/src/kiss-icp
WORKDIR /aas/github_ws
# Explicitly use bash, not sh, to source and build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-skip livox_ros_driver2 --cmake-args -DCMAKE_BUILD_TYPE=Release"

################################################################################
# Empty branch to skip the build of advanced odometry, SLAM packages ###########
# Mutually exclusive with the next stage #######################################
################################################################################
FROM ros2-px4msgs-dds-mavros-yolo-ort-simple-odom-image AS advanced-odom-false
# Do nothing

################################################################################
# Branch including the build of advanced odometry, SLAM packages ###############
# Mutually exclusive with the previous stage ###################################
################################################################################
FROM ros2-px4msgs-dds-mavros-yolo-ort-simple-odom-image AS advanced-odom-true

# Install OpenVINS, based on https://docs.openvins.com/gs-installing.html
RUN apt-get update && \
    apt-get install -y --no-install-recommends libeigen3-dev libboost-all-dev libceres-dev libopencv-contrib-dev \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*
COPY /_github_clones/open_vins /aas/github_ws/src/open_vins
# Fix for ROS 2 Jazzy compatibility: image_transport and cv_bridge dropped their .h headers in favour of .hpp
RUN sed -i -e 's|\(image_transport/image_transport\)\.h>|\1.hpp>|' \
    -e 's|\(cv_bridge/cv_bridge\)\.h>|\1.hpp>|' \
    /aas/github_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.h
WORKDIR /aas/github_ws
# Explicitly use bash, not sh, to source and build the workspace
# Limiting resource usage to avoid freezes on resource-constrained hosts
RUN MAKEFLAGS='-j4' NINJAJOBS='-j4' bash -c "source /opt/ros/jazzy/setup.bash && colcon build --event-handlers console_cohesion+ --packages-select ov_core ov_init ov_msckf ov_eval --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Release"

# Install SPARK-FAST-LIO, based on https://github.com/MIT-SPARK/spark-fast-lio#package-how-to-install
COPY /_github_clones/spark-fast-lio /aas/github_ws/src/spark-fast-lio
WORKDIR /aas/github_ws
# Explicitly use bash, not sh, to source and build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build --packages-up-to spark_fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Install SuperOdom dependencies, based on https://github.com/superxslam/SuperOdom#-3-installation
# Sophus on 2021's commit https://github.com/strasdat/Sophus/commit/97e7161
WORKDIR /aas/github_apps/
RUN mkdir Sophus \
    && wget -qO- https://github.com/strasdat/Sophus/archive/97e7161.tar.gz | tar -xz -C Sophus --strip-components=1 \
    && cd Sophus \
    && mkdir build && cd build \
    && cmake .. -DBUILD_TESTS=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    && make -j$(nproc) \
    && make install
# gtsam on 2024's commit https://github.com/borglab/gtsam/commit/4abef92, also required by KISS-Matcher
WORKDIR /aas/github_apps/
RUN mkdir gtsam \
    && wget -qO- https://github.com/borglab/gtsam/archive/4abef92.tar.gz | tar -xz -C gtsam --strip-components=1 \
    && cd gtsam \
    && mkdir build && cd build \
    && cmake -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 .. \
    && make -j$(nproc) \
    && make install
# f68321e tag is release 2.1.0 https://github.com/ceres-solver/ceres-solver/releases/tag/2.1.0
WORKDIR /aas/github_apps/
RUN apt-get update && \
    apt-get install -y --no-install-recommends libgoogle-glog-dev \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir ceres-solver \
    && wget -qO- https://github.com/ceres-solver/ceres-solver/archive/f68321e.tar.gz | tar -xz -C ceres-solver --strip-components=1 \
    && cd ceres-solver \
    && mkdir build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir --resume-retries 5 rerun-sdk
# Add rviz_2d_overlay_plugins, based on https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins#rviz_2d_overlay_plugins
RUN mkdir -p /aas/github_ws/src/rviz_2d_overlay_plugins && \
    SHA=$(git ls-remote https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins.git refs/heads/main | cut -f1) && [ -n "$SHA" ] && \
    wget -qO- https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins/archive/${SHA}.tar.gz | tar -xz -C /aas/github_ws/src/rviz_2d_overlay_plugins --strip-components=1 && \
    echo "rviz_2d_overlay_plugins main ${SHA} $(find /aas/github_ws/src/rviz_2d_overlay_plugins -maxdepth 1 -type f -printf '%TF\n' | head -1)" >> /aas/repo_dep_branch_heads.txt
WORKDIR /aas/github_ws
# Explicitly use bash, not sh, to source and build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build --packages-select rviz_2d_overlay_msgs rviz_2d_overlay_plugins --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Install SuperOdom, based on https://github.com/superxslam/SuperOdom#-3-installation
COPY /_github_clones/SuperOdom /aas/github_ws/src/SuperOdom
WORKDIR /aas/github_ws
# Explicitly use bash, not sh, to source and build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && source /aas/github_ws/install/setup.bash && colcon build --packages-up-to super_odometry --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Install KISS-Matcher, based on https://github.com/MIT-SPARK/KISS-Matcher/tree/main/ros#gear-how-to-build--run
COPY /_github_clones/KISS-Matcher /aas/github_ws/src/KISS-Matcher
WORKDIR /aas/github_ws
# Explicitly use bash, not sh, to source and build the workspace, pass CMAKE_POLICY_VERSION_MINIMUM as env var for nested builds
RUN CMAKE_POLICY_VERSION_MINIMUM=3.5 bash -c "source /opt/ros/jazzy/setup.bash && colcon build --packages-select kiss_matcher_ros --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Install mimosa, based on https://github.com/ntnu-arl/mimosa/tree/dev/ros2#common-setup
RUN apt-get update && \
    apt-get install -y --no-install-recommends libgoogle-glog-dev libspdlog-dev ros-jazzy-pcl-ros \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /aas/mimosa_custom_gtsam_ws/src
COPY /_github_clones/mimosa /aas/mimosa_custom_gtsam_ws/src/mimosa
# Download config_utilities (branch: dev/mimosa), gtsam (commit c952ef9, see issue: https://github.com/ntnu-arl/mimosa/issues/19), and gtsam_points (branch: minimal_updated)
RUN mkdir -p config_utilities \
    && SHA=$(git ls-remote https://github.com/ntnu-arl/config_utilities.git refs/heads/dev/mimosa | cut -f1) && [ -n "$SHA" ] \
    && wget -qO- https://github.com/ntnu-arl/config_utilities/archive/${SHA}.tar.gz | tar -xz -C config_utilities --strip-components=1 \
    && echo "config_utilities dev/mimosa ${SHA} $(find config_utilities -maxdepth 1 -type f -printf '%TF\n' | head -1)" >> /aas/repo_dep_branch_heads.txt \
    && mkdir -p gtsam \
    && wget -qO- https://github.com/ntnu-arl/gtsam/archive/c952ef9.tar.gz | tar -xz -C gtsam --strip-components=1 \
    && mkdir -p gtsam_points \
    && SHA=$(git ls-remote https://github.com/ntnu-arl/gtsam_points.git refs/heads/minimal_updated | cut -f1) && [ -n "$SHA" ] \
    && wget -qO- https://github.com/ntnu-arl/gtsam_points/archive/${SHA}.tar.gz | tar -xz -C gtsam_points --strip-components=1 \
    && echo "gtsam_points minimal_updated ${SHA} $(find gtsam_points -maxdepth 1 -type f -printf '%TF\n' | head -1)" >> /aas/repo_dep_branch_heads.txt
WORKDIR /aas/mimosa_custom_gtsam_ws
# Explicitly use bash, not sh, to source and build the workspace
# Build mimosa's GTSAM fork and gtsam_points with -DBUILD_SHARED_LIBS=OFF -DGTSAM_BUILD_SHARED_LIBRARY=OFF, not to shadow the system-wide GTAM used by SuperOdom, KISS-Matcher
RUN bash -c "source /opt/ros/jazzy/setup.bash && source /aas/github_ws/install/setup.bash && \
    colcon build --packages-select gtsam gtsam_points --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_USE_QUATERNIONS=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_WITH_TBB=OFF \
    -DBUILD_SHARED_LIBS=OFF -DGTSAM_BUILD_SHARED_LIBRARY=OFF"
# Build the rest of the mimosa workspace with the static GTSAM from mimosa's fork (limiting resource usage to avoid freezes on resource-constrained hosts)
RUN MAKEFLAGS='-j4' NINJAJOBS='-j4' bash -c "source /opt/ros/jazzy/setup.bash && source /aas/github_ws/install/setup.bash && source /aas/mimosa_custom_gtsam_ws/install/setup.bash && \
    colcon build --packages-up-to mimosa --packages-skip gtsam gtsam_points --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Release"

# Install rovio (ROS 2 porting of https://github.com/ethz-asl/rovio), based on https://github.com/JacopoPan/rovio_ros2#installation
RUN apt-get update && \
    apt-get install -y --no-install-recommends freeglut3-dev libglew-dev \
    # valgrind \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir kindr \
    && SHA=$(git ls-remote https://github.com/ethz-asl/kindr.git refs/heads/master | cut -f1) && [ -n "$SHA" ] \
    && wget -qO- https://github.com/ethz-asl/kindr/archive/${SHA}.tar.gz | tar -xz -C kindr --strip-components=1 \
    && echo "kindr master ${SHA} $(find kindr -maxdepth 1 -type f -printf '%TF\n' | head -1)" >> /aas/repo_dep_branch_heads.txt \
    && cd kindr \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    && make install
COPY /_github_clones/rovio /aas/github_ws/src/rovio
WORKDIR /aas/github_ws
# Explicitly use bash, not sh, to source and build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && source /aas/github_ws/install/setup.bash && source /aas/mimosa_custom_gtsam_ws/install/setup.bash && \
    colcon build --packages-up-to rovio --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON -DENABLE_VALGRIND_COMPATIBILITY=OFF"

################################################################################
# Add analysis tools and YOLO models ###########################################
################################################################################
FROM advanced-odom-${BUILD_ADVANCED_ODOM} AS ros2-px4msgs-dds-mavros-yolo-ort-odom-analysis-models-image

# Add pymavlink and PlotJuggler for debugging, testing, and analysis
RUN pip3 install --no-cache-dir --retries 5 pymavlink pyserial
# Check with $ python3 -c "import pymavlink; print(pymavlink.__version__)"
RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-jazzy-plotjuggler \
    ros-jazzy-plotjuggler-ros ros-jazzy-rosbag2-storage-mcap \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# Save the YOLO model weights (ONNX, Opset 12) and class names
WORKDIR /aas/yolo
# Model options (from fastest to most accurate, <10MB to >100MB): yolo26n, yolo26s, yolo26m, yolo26l, yolo26x
# Export standard 640 static as yolo26n_640.onnx and smaller 320 static as yolo26n_320.onnx
RUN /yolo-env/bin/python3 -c "from ultralytics import YOLO; YOLO('yolo26n.pt').export(format='onnx', opset=12, imgsz=640)" && \
    mv yolo26n.onnx yolo26n_640.onnx && \
    /yolo-env/bin/python3 -c "from ultralytics import YOLO; YOLO('yolo26n.pt').export(format='onnx', opset=12, imgsz=320)" && \
    mv yolo26n.onnx yolo26n_320.onnx && \
    /yolo-env/bin/python3 -c "import json; from ultralytics import YOLO; print(json.dumps(YOLO('yolo26n.pt').names))" | grep '{' > coco.json && \
    rm yolo26n.pt

################################################################################
# Copy AAS resources and build AAS ROS2 workspace ##############################
################################################################################
FROM ros2-px4msgs-dds-mavros-yolo-ort-odom-analysis-models-image AS aircraft-dev-image

# Build the ROS 2 workspace (NOTE: also includes ground_system_msgs from the ground_ws)
COPY ground/ground_ws/src/ground_system_msgs /aas/aircraft_ws/src/ground_system_msgs
COPY aircraft/aircraft_ws/src /aas/aircraft_ws/src
COPY tools_and_docs/tests/.clang-tidy /aas/aircraft_ws/src/.clang-tidy
WORKDIR /aas/aircraft_ws
RUN rosdep update
RUN apt update && apt install -y python3-simpleeval && rosdep install --from-paths src/ --ignore-src --rosdistro jazzy -y --skip-keys "px4_msgs" && apt clean && rm -rf /var/lib/apt/lists/*
# Explicitly use bash, not sh, to source and build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && source /aas/github_ws/install/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Copy resources and configuration files from this repository
COPY aircraft/aircraft_resources/ /aas/aircraft_resources
COPY aircraft/aircraft_resources/patches/kiss_icp.rviz /aas/github_ws/src/kiss-icp/ros/rviz/kiss_icp.rviz
COPY aircraft/aircraft_resources/patches/apm_pluginlists.yaml /opt/ros/jazzy/share/mavros/launch/apm_pluginlists.yaml
RUN ln -s /aas/aircraft_resources/patches/cancellable_action.py /usr/local/bin/cancellable_action \
    && chmod +x /aas/aircraft_resources/patches/cancellable_action.py

# Copy sensor configuration
COPY simulation/simulation_resources/aircraft_models/sensor_config.yaml /aas/aircraft_resources/sensor_config.yaml

# Source the workspaces
RUN echo "source /aas/github_ws/install/setup.bash" >> /root/.bashrc \
    && echo "source /aas/mimosa_custom_gtsam_ws/install/setup.bash" >> /root/.bashrc \
    && echo "source /aas/aircraft_ws/install/setup.bash" >> /root/.bashrc
# If needed (but already in .bashrc) $ source /opt/ros/jazzy/setup.bash && source /aas/github_ws/install/setup.bash && source /aas/mimosa_custom_gtsam_ws/install/setup.bash && source /aas/aircraft_ws/install/setup.bash

# Final config
WORKDIR /aas
COPY aircraft/aircraft.yml.erb /aas/aircraft.yml.erb
COPY simulation/simulation_resources/patches/tmux.conf /root/.tmux.conf
ENTRYPOINT ["tmuxinator", "start", "-p", "/aas/aircraft.yml.erb"]
