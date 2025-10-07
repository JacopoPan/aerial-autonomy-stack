# aerial-autonomy-stack

*Aerial autonomy stack* (AAS) is a software stack to:

1. **Develop** end-to-end drone autonomy with ROS2
2. **Simulate** perception and control in SITL, with YOLOv8, LiDAR and PX4 or ArduPilot
3. **Deploy** in real drones with JetPack on NVIDIA Orin

https://github.com/user-attachments/assets/c194ada6-2996-4bfa-99e9-32b45e29281d

## Features

- Support for **multiple quadrotors and VTOLs** based on either **PX4 or ArduPilot**
- Vehicle-agnostic **ROS2** action-based autopilot interface (*via* XRCE-DDS and MAVROS)
- Support for **YOLOv8** (with ONNX GPU Runtimes) and **LiDAR Odometry** (with [KISS-ICP](https://github.com/PRBonn/kiss-icp))
- **Dockerized simulation** based on [`nvcr.io/nvidia/cuda:12.8.1-cudnn-runtime-ubuntu22.04`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda/tags)
- **Dockerized deployment** based on [`nvcr.io/nvidia/l4t-jetpack:r36.4.0`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack/tags)
- **Windows 11 compatibility** with GPU support *via* WSLg
- **3D worlds** for PX4 and ArduPilot software-in-the-loop (SITL) simulation
- [Zenoh](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds) inter-vehicle ROS2 bridge
- Support for [PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard.html) mode (e.g. CTBR/`VehicleRatesSetpoint` for agile, GNSS-denied flight) 
- Support for [ArduPilot Guided](https://ardupilot.org/copter/docs/ac2_guidedmode.html) mode (i.e. `setpoint_velocity`, `setpoint_accel` references)
- Logs analysis with [`flight_review`](https://github.com/PX4/flight_review) (`.ulg`), MAVExplorer (`.bin`), and [PlotJuggler](https://github.com/facontidavide/PlotJuggler) (`rosbag`)
- Support for Gazebo's **wind effects** plugin
- **Steppable simulation**

<details>
<summary>AAS leverages the following frameworks: <i>(click to expand)</i></summary>

> [*Ubuntu 22.04*](https://ubuntu.com/about/release-cycle) (LTS, ESM 4/2032), [*`nvidia-driver-580`*](https://developer.nvidia.com/datacenter-driver-archive) (latest as of 9/2025), [*Docker Engine v28*](https://docs.docker.com/engine/release-notes/28/) (latest as of 9/2025), [*ROS2 Humble*](https://docs.ros.org/en/rolling/Releases.html) (LTS, EOL 5/2027), [*Gazebo Sim Harmonic*](https://gazebosim.org/docs/latest/releases/) (LTS, EOL 9/2028), [*PX4 1.16*](https://github.com/PX4/PX4-Autopilot/releases) interfaced *via* [XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS/releases), [*ArduPilot 4.6*](https://github.com/ArduPilot/ardupilot/releases) interfaced *via* [MAVROS](https://github.com/mavlink/mavros/releases), [*YOLOv8*](https://github.com/ultralytics/ultralytics/releases) on [*ONNX Runtime 1.22*](https://onnxruntime.ai/getting-started) (latest stable releases as of 8/2025), [*L4T 36* (Ubuntu 22-based)/*JetPack 6*](https://developer.nvidia.com/embedded/jetpack-archive) (for deployment only, latest major release as of 8/2025), [WSLg](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps) (for simulation and development on Windows 11 only)

</details>

---

## Part 1: Installation

> [!IMPORTANT]
> AAS is developed using Ubuntu 22.04 with `nvidia-driver-580` on an i9-13 with RTX 3500 and an i7-11 with RTX 3060—an NVIDIA GPU *is* required for ideal performance
> 
> **To setup the requirements: (i) Ubuntu, (ii) NVIDIA driver, (iii) Docker Engine, and (iv) NVIDIA Container Toolkit (with NVIDIA NGC API Key), read [`REQUIREMENTS_UBUNTU.md`](/supplementary/REQUIREMENTS_UBUNTU.md)**
>
> Windows 11 support is available *via* WSLg, read [`REQUIREMENTS_WSL.md`](/supplementary/REQUIREMENTS_WSL.md)

> [!WARNING]
> The 1st build takes ~45GB of space and ~25' with good internet (`Ctrl + c` and restart if needed)


```sh
# Install dependencies (git, Git LFS, Xterm)
sudo apt update
sudo apt install -y git git-lfs xterm xfonts-base
git lfs install

# Clone this repo
mkdir -p ~/git && cd ~/git
git clone https://github.com/JacopoPan/aerial-autonomy-stack.git

# Build the Docker images
cd ~/git/aerial-autonomy-stack/scripts
./sim_build.sh # The first build takes ~25', subsequent ones will take seconds to minutes thanks to the Docker cache
```

> Latest weekly builds with `sim_build.sh`: 
> [![simulation-image amd64](https://github.com/JacopoPan/aerial-autonomy-stack/actions/workflows/weekly-simulation-amd64-build.yml/badge.svg)](https://github.com/JacopoPan/aerial-autonomy-stack/actions/workflows/weekly-simulation-amd64-build.yml)
> [![aircraft-image amd64](https://github.com/JacopoPan/aerial-autonomy-stack/actions/workflows/weekly-aircraft-amd64-build.yml/badge.svg)](https://github.com/JacopoPan/aerial-autonomy-stack/actions/workflows/weekly-aircraft-amd64-build.yml)

---

## Part 2: Simulation and Development

```sh
# Start a simulation
cd ~/git/aerial-autonomy-stack/scripts
AUTOPILOT=px4 NUM_QUADS=1 NUM_VTOLS=1 WORLD=swiss_town ./sim_run.sh # Check the script for more options (note: ArduPilot SITL takes ~40s to be ready to arm)
```

> [!NOTE]
> On a low-mid range laptop—i7-11 with 16GB RAM and RTX 3060—AAS simulates three PX4 quads with camera and LiDAR at 99% real-time-factor (note that ArduPilot faster physics updates and more complex worlds have higher computational demands). Make sure you run `sudo prime-select nvidia` and rebooted to effectively leverage GPU compute.

![worlds](https://github.com/user-attachments/assets/45a2f2ad-cc31-4d71-aa2e-4fe542a59a77)

Available `WORLD`s:
- `apple_orchard`, a GIS world created using [BlenderGIS](https://github.com/domlysz/BlenderGIS)
- `impalpable_greyness`, (default) an empty world with simple shapes
- `shibuya_crossing`, a 3D world adapted from [cgtrader](https://www.cgtrader.com/)
- `swiss_town`, a photogrammetry world courtesy of [Pix4D / pix4d.com](https://support.pix4d.com/hc/en-us/articles/360000235126)

To advance the simulation in **discrete time steps**, e.g. 1s, from a terminal on the host, run:

```sh
docker exec simulation-container bash -c " \
  gz service -s /world/\$WORLD/control \
    --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
    --req 'multi_step: 250, pause: true'" # Adjust multi_step based on the value of max_step_size in the world's .sdf (defaults: 250 for PX4, 1000 for ArduPilot)
```

To add or disable [**wind effects**](https://github.com/gazebosim/gz-sim/blob/gz-sim10/examples/worlds/wind.sdf), from a terminal on the host, run:

```sh
docker exec simulation-container bash -c " \
  gz topic -t /world/\$WORLD/wind/ -m gz.msgs.Wind \
  -p 'linear_velocity: {x: 0.0 y: 3.0}, enable_wind: true'" # Positive X blows from the West, positive Y blows from the South

docker exec simulation-container bash -c " \
  gz topic -t /world/\$WORLD/wind/ -m gz.msgs.Wind \
  -p 'enable_wind: false'" # Disable WindEffects
```

### Fly a Mission

```sh
cd ~/git/aerial-autonomy-stack/scripts
AUTOPILOT=px4 NUM_QUADS=1 ./sim_run.sh # Or `ardupilot`, or `NUM_VTOLS=1`

# In aircraft 1's terminal
ros2 run mission mission --conops yalla \
  --ros-args -r __ns:=/Drone$DRONE_ID -p use_sim_time:=true # This mission is a simple takeoff, followed by an orbit, and landing for any vehicle

# Finally, in the simulation's terminal
/simulation_resources/patches/plot_logs.sh # Analyze the flight logs
```

To create a new mission, read the banner comments in [`ardupilot_interface.hpp`](/aircraft/aircraft_ws/src/autopilot_interface/src/ardupilot_interface.hpp) and [`px4_interface.hpp`](/aircraft/aircraft_ws/src/autopilot_interface/src/px4_interface.hpp) for command line examples of takeoff, orbit, reposition, offboard, land

Once flown from CLI, implemented your mission in [`MissionNode.conops_callback()`](/aircraft/aircraft_ws/src/mission/mission/mission_node.py)

![interface](https://github.com/user-attachments/assets/71b07851-42dd-45d4-a9f5-6b5b00cd85bc)

### Development within Live Containers

Launching the `sim_run.sh` script with `MODE=dev`, does **not** start the simulation and mounts folders `simulation_resources`, `aircraft_resources`, and `ros2_ws/src` as volumes to more easily track, commit, push changes while building and testing them within the containers

```sh
cd ~/git/aerial-autonomy-stack/scripts
MODE=dev ./sim_run.sh # Images are pre-built but the ros2_ws/src/ and *_resources/ folders are mounted from the host
```

> [!TIP]
> <details>
> <summary>AAS Structure <i>(click to expand)</i></summary>
> 
> ```sh
> aerial-autonomy-stack
> │
> ├── aircraft
> │   ├── aircraft_ws
> │   │   └── src
> │   │       ├── autopilot_interface # Ardupilot/PX4 high-level actions (Takeoff, Orbit, Offboard, Land)
> │   │       ├── mission             # Orchestrator of the actions in `autopilot_interface` 
> │   │       ├── offboard_control    # Low-level references for the Offboard action in `autopilot_interface` 
> │   │       ├── state_sharing       # Publisher of the `/state_sharing_drone_N` topic broadcasted by Zenoh
> │   │       └── yolo_inference      # GStreamer video acquisition and publisher of YOLO bounding boxes
> │   │
> │   └── aircraft.yml.erb            # Aircraft docker tmux entrypoint
> │
> ├── scripts
> │   ├── docker
> │   │   ├── Dockerfile.aircraft     # Docker image for aircraft simulation and deployment
> │   │   └── Dockerfile.simulation   # Docker image for Gazebo and SITL simulation
> │   │
> │   ├── deploy_build.sh             # Build `Dockerfile.aircraft` for arm64/Orin
> │   ├── deploy_run.sh               # Start the aircraft docker on arm64/Orin
> │   │
> │   ├── sim_build.sh                # Build both dockerfiles for amd64/simulation
> │   └── sim_run.sh                  # Start the simulation
> │
> └── simulation
>     ├── simulation_resources
>     │   ├── aircraft_models
>     │   │   ├── alti_transition_quad # ArduPilot VTOL
>     │   │   ├── iris_with_ardupilot  # ArduPilot quad
>     │   │   ├── sensor_camera
>     │   │   ├── sensor_lidar
>     │   │   ├── standard_vtol        # PX4 VTOL
>     │   │   └── x500                 # PX4 quad
>     │   └── simulation_worlds
>     │       ├── apple_orchard.sdf
>     │       ├── impalpable_greyness.sdf
>     │       ├── shibuya_crossing.sdf
>     │       └── swiss_town.sdf
>     │
>     ├── simulation_ws
>     │   └── src
>     │       └── ground_system        # Publisher of topic `/tracks` broadcasted by Zenoh
>     │
>     └── simulation.yml.erb           # Simulation docker tmux entrypoint
> ```
> </details>
> 
> <details>
> <summary>Tmux and Docker Shortcuts <i>(click to expand)</i></summary>
> 
> - Move between Tmux windows with `Ctrl + b`, then `n`, `p`
> - Move between Tmux panes with `Ctrl + b`, then `arrow keys`
> - Enter copy mode to scroll back with `Ctrl + [`, then `arrow keys`, exit with `q`
> - Split a Tmux window with `Ctrl + b`, then `"` (horizontal) or `%` (vertical)
> - Detach Tmux with `Ctrl + b`, then `d`
> ```sh
> tmux list-sessions # List all sessions
> tmux attach-session -t [session_name] # Reattach a session
> tmux kill-session -t [session_name] # Kill a session
> tmux kill-server # Kill all sessions
> ```
> Docker hygiene:
> ```sh
> docker ps -a # List containers
> docker stop $(docker ps -q) # Stop all containers
> docker container prune # Remove all stopped containers
> 
> docker images # List images
> docker image prune # Remove untagged images
> docker rmi <image_name_or_id> # Remove a specific image
> docker builder prune # Clear the cache system wide
> ```
> </details>

---

## Part 3: Jetson Deployment

> [!IMPORTANT]
> These instructions are tested on a [Holybro Jetson Baseboard](https://holybro.com/products/pixhawk-jetson-baseboard) (Pixhawk 6X + NVIDIA Orin NX 16GB)
> 
> **To setup (i.a) PX4's DDS UDP client or (i.b) ArduPilot serial MAVLink bridge, (ii) JetPack 6, (iii) Docker Engine, and (iv) NVIDIA Container Toolkit (with NVIDIA NGC API Key) on Orin, read [`SETUP_AVIONICS.md`](/supplementary/SETUP_AVIONICS.md)**

```sh
# Install dependencies (git, Git LFS)
sudo apt update
sudo apt install -y git git-lfs
git lfs install

# Clone this repo
mkdir -p ~/git && cd ~/git
git clone git@github.com:JacopoPan/aerial-autonomy-stack.git

# On Jetson Orin NX, build for arm64 with TensorRT support
cd ~/git/aerial-autonomy-stack/scripts
./deploy_build.sh # The first build takes ~1h (mostly to build onnxruntime-gpu from source)
```

> Latest weekly build with `deploy_build.sh`:
> [![aircraft-image arm64](https://github.com/JacopoPan/aerial-autonomy-stack/actions/workflows/weekly-aircraft-arm64-build.yml/badge.svg)](https://github.com/JacopoPan/aerial-autonomy-stack/actions/workflows/weekly-aircraft-arm64-build.yml)

```sh
# On Jetson Orin NX, start and attach an aircraft-container (e.g., from ssh)
DRONE_TYPE=quad AUTOPILOT=px4 DRONE_ID=1 CAMERA=true LIDAR=false ./deploy_run.sh
docker exec -it aircraft-container tmux attach
```

---

## Future Work / Ideas for Contributions

- Support for [SPARK-FAST-LIO](https://github.com/MIT-SPARK/spark-fast-lio)/[SuperOdom](https://github.com/superxslam/SuperOdom)
- Support for [JSBSim](https://github.com/JSBSim-Team/jsbsim) flight dynamics
- Support for [ArduPilot's DDS interface](https://ardupilot.org/dev/docs/ros2-interfaces.html)
- Support for a [Isaac Sim](https://github.com/isaac-sim/IsaacSim) higher fidelity rendering
- Support for [Betaflight SITL](https://betaflight.com/docs/development/SITL) interfaced *via* [MultiWii Serial Protocol (MSP)](https://github.com/betaflight/betaflight/tree/master/src/main/msp)

---
> You've done a man's job, sir. I guess you're through, huh?

<!-- 

## TODOs

- https://developer.nvidia.com/embedded/learn/tutorials/first-picture-csi-usb-camera
- https://github.com/Livox-SDK/livox_ros_driver2
- Add state estimation package/node
- Add bounding-box-based Offboard
- For PX4 quad max tilt maneuver, zero the anti-windup gain: const float arw_gain = 2.f / _gain_vel_p(0);
- ????
- Profit

Add to AVIONICS.md
- The Holybro Jetson Baseboard comes with an (i) integrated 4-way (Orin, 6X, RJ-45, JST) Ethernet switch and (ii) two JST USB 2.0 that can be connected to ASIX Ethernet adapters to create additional network interfaces
- Make sure to configure Orin, 6X's XRCE-DDS, IP radio, Zenoh, etc. consistently with your network setup; the camera acquisition pipeline should be setup in `yolo_inference_node.py`, the LiDAR should publish on topic `/lidar_points` for KISS-ICP (if necessary, discuss in the [Issues](https://github.com/JacopoPan/aerial-autonomy-stack/issues))

SITL architectures
- https://docs.px4.io/main/en/simulation/#sitl-simulation-environment
- https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html#sitl-architecture

### Known Issues

- wmctrl does not work as-is in WSLg
- QGC is started with a virtual joystick (with low throttle if using only VTOLs and centered throttle if there are quads), this is reflective of real-life but note that this counts as "RC loss" when switching focus from one autopilot instance to another
- ArduPilot CIRCLE mode for quads require to explicitly center the virtual throttle with 'rc 3 1500' to keep altitude
- Gazebo WindEffects plugin is disabled/not working for PX4 standard_vtol
- Command 178 MAV_CMD_DO_CHANGE_SPEED is accepted but not effective in changing speed for ArduPilot VTOL
- ArduPilot SITL for Iris uses option -f that also sets "external": True, this is not the case for the Alti Transition from ArduPilot/SITL_Models 
- In ArdupilotInterface's action callbacks, std::shared_lock<std::shared_mutex> lock(node_data_mutex_); could be used on the reads of lat_, lon_, alt_
- In yolo_inference_node.py, cannot open GPU accelerated (nvh264dec) GStreamer pipeline with cv2.VideoCapture, might need to recompile OpenCV to have both CUDA and GStreamer support (or use python3-gi gir1.2-gst-plugins-base-1.0 gir1.2-gstreamer-1.0 and circumvent OpenCV)
- QGC does not save roll and pitch in the telemetry bar for PX4 VTOLs (MAV_TYPE 22)

-->
