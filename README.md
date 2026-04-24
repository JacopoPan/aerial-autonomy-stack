# aerial-autonomy-stack

<img align="right" height="120" alt="mission-patch" src="https://github.com/user-attachments/assets/252cc420-9527-4342-9ffd-c6bec77ebdf3" />

*Aerial autonomy stack* (AAS) is an all-in-one software stack to:

1. **Develop** multi-drone autonomy—with ROS2, PX4, and ArduPilot
2. **Simulate** faster-than-real-time perception and control—with YOLO and 3D LiDAR
3. **Deploy** in real drones—with JetPack and NVIDIA Orin

For an example bill of materials, read [`BOM.md`](/supplementary/BOM.md); for motivation, read [`RATIONALE.md`](/supplementary/RATIONALE.md); if you wish, cite this work as:

```bibtex
@misc{panerati2026aas,
      title={{\ttfamily aerial-autonomy-stack}---a faster-than-real-time, autopilot-agnostic, {ROS2} framework to simulate and deploy perception-based drones}, 
      author={Jacopo Panerati and Sina Sajjadi and Sina Soleymanpour and Varunkumar Mehta and Iraj Mantegh},
      year={2026},
      eprint={2602.07264},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2602.07264}}
```

<details>
<summary><b>Feature list</b> <i>(click to expand)</i></summary>

- **PX4 and ArduPilot multi-vehicle** simulation (**quadrotors and VTOLs**)
- ROS2 action-based autopilot interface (*via* XRCE-DDS or MAVROS)
- **YOLO** (with ONNX GPU Runtimes) and **LiDAR** Odometry (with [KISS-ICP](https://github.com/PRBonn/kiss-icp))
- 3D worlds for perception-based simulation
- **Steppable** [Gymnasium environment](https://gymnasium.farama.org/index.html) and **faster-than-real-time**, **multi-instance** simulation
- Gazebo's wind effects plugin
- **Dockerized simulation** based on [Ubuntu with CUDA and cuDNN](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda/tags)
- **Dockerized deployment** based on [NVIDIA JetPack](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack/tags) with [DeepStream](https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_Installation.html#platform-and-os-compatibility)
- **Windows 11** compatibility *via* WSL
- Multi-**Jetson-in-the-loop (HITL) simulation** to test NVIDIA- and ARM-based on-board compute
- Dual network to separate simulated sensors (`SIM_SUBNET`) and inter-vehicle comms (`AIR_SUBNET`)
- [Zenoh](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds) inter-vehicle ROS2 bridge
- [PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard.html) interface (e.g. CTBR/`VehicleRatesSetpoint` for agile, GNSS-denied flight) 
- [ArduPilot Guided](https://ardupilot.org/copter/docs/ac2_guidedmode.html) interface (i.e. `setpoint_velocity`, `setpoint_accel` references)
- Logs analysis with [`flight_review`](https://github.com/PX4/flight_review) (`.ulg`), MAVExplorer (`.bin`), and [PlotJuggler](https://github.com/facontidavide/PlotJuggler) (`rosbag`)

</details>

https://github.com/user-attachments/assets/57e5bc91-8bee-4bae-8f81-a9aacef471e7

## 0. Architecture

```mermaid
%%{init: {'theme': 'base', 'themeVariables': { 'fontFamily': 'monospace'}}}%%
flowchart TB
    subgraph aas [" "]
        subgraph sim ["#nbsp;simulation#nbsp;container#nbsp;(amd64)"]
            sitl("[N x] PX4 || <br/> ArduPilot SITL"):::resource
            gz(Gazebo Sim):::resource
            subgraph models [Models]
                drones(aircraft_models/):::resource
                worlds(simulation_worlds/):::resource
            end

            drones --> gz
            worlds --> gz
            sitl <--> |"gz_bridge || ardupilot_gazebo"| gz
        end

        subgraph gnd ["#nbsp;ground#nbsp;container#nbsp;(amd64)"]
            mlrouter{{mavlink-router}}:::bridge
            ground_system[/ground_system\]:::algo
            qgc(QGroundControl):::resource
            zenoh_gnd{{zenoh-bridge}}:::bridge

            ground_system --> |"/tracks"| zenoh_gnd
            mlrouter <--> qgc
            mlrouter --> ground_system
        end

        subgraph air ["[N#nbsp;x]#nbsp;aircraft#nbsp;container(s)#nbsp;(amd64,#nbsp;arm64)"]
            subgraph perception [Perception]
                yolo_py[/yolo_py/]:::algo
                kiss_icp[/kiss_icp/]:::algo
            end
            subgraph control [Control]
                offboard_control(offboard_control):::algo
                autopilot_interface(autopilot_interface):::algo
                mission(mission):::algo
            end
            ap_link{{"uxrce_dds <br/> || MAVROS"}}:::bridge
            subgraph swarm [Swarm]
                state_sharing[/state_sharing\]:::algo
            end
            zenoh_air{{zenoh-bridge}}:::bridge

            kiss_icp -.-> |"/TBD"| ap_link
            ap_link <--> autopilot_interface
            ap_link --> state_sharing
            yolo_py --> |"/detections"| offboard_control
            offboard_control --> |"/reference"| autopilot_interface
            mission --> |"ros2 action/srv"| autopilot_interface
            zenoh_air <--> |"/state_drone_n"| state_sharing
        end

        repo(((aerial#nbsp;autonomy#nbsp;stack)))
    end

    repo ~~~ gz
    gz --> |"gz_gst_bridge <br/> [SIM_SUBNET]"| yolo_py
    gz --> |"/lidar_points <br/> [SIM_SUBNET]"| kiss_icp
    sitl <--> |"UDP <br/> [SIM_SUBNET]"| ap_link
    sitl <--> |"MAVLink <br/> [SIM_SUBNET]"| mlrouter 
    zenoh_gnd <-.-> |"TCP <br/> [AIR_SUBNET]"| zenoh_air

    classDef bridge fill:#ffebd6,stroke:#f5a623,stroke-width:2px;
    classDef algo fill:#e1f5fe,stroke:#0277bd,stroke-width:2px;
    classDef resource fill:#e8f5e9,stroke:#2e7d32,stroke-width:2px;
    classDef blueStyle  fill:#e1f0ff,stroke:#666,stroke-width:2px;
    classDef whiteStyle fill:#f9f9f9,stroke:#666,stroke-width:1px,stroke-dasharray: 5 5;
    classDef greyStyle  fill:#eeeeee,stroke:#666,stroke-width:1px,stroke-dasharray: 5 5;

    class aas,repo blueStyle;
    class air,gnd,sim whiteStyle;
    class perception,control,models,swarm greyStyle;
    linkStyle 14,15,16,17 stroke:teal,stroke-width:3px;
    linkStyle 18 stroke:blue,stroke-width:4px;
```

<details>
<summary>Repository structure <i>(click to expand)</i></summary>

```sh
aerial-autonomy-stack
│
├── aas-gym
│   └── src
│       └── aas_gym
│           └── aas_env.py                            # aerial-autonomy-stack as a Gymnasium environment
│
├── aircraft
│   ├── aircraft_ws
│   │   └── src
│   │       ├── autopilot_interface                   # Ardupilot/PX4 high-level actions (Takeoff, Orbit, Offboard, Land)
│   │       ├── mission                               # Orchestrator of the actions in `autopilot_interface`
│   │       ├── offboard_control                      # Low-level references for the Offboard action in `autopilot_interface`
│   │       ├── state_sharing                         # Publisher of the `/state_sharing_drone_N` topic broadcasted by Zenoh
│   │       └── yolo_py                               # GStreamer video acquisition and publisher of YOLO bounding boxes
│   │
│   └── aircraft.yml.erb                              # Aircraft docker tmux entrypoint
│
├── ground
│   ├── ground_ws
│   │   └── src
│   │       └── ground_system                         # Publisher of topic `/tracks` broadcasted by Zenoh
│   │
│   └── ground.yml.erb                                # Ground docker tmux entrypoint
│
├── scripts
│   ├── docker
│   │   ├── Dockerfile.aircraft                       # Docker image for aircraft simulation and deployment
│   │   ├── Dockerfile.ground                         # Docker image for ground system simulation and deployment
│   │   └── Dockerfile.simulation                     # Docker image for SITL and HITL simulation
│   │
│   ├── deploy_build.sh                               # Build `Dockerfile.aircraft` for arm64/Orin
│   ├── deploy_run.sh                                 # Start the aircraft docker on arm64/Orin or the ground docker on amd64 (deploy or HITL)
│   │
│   ├── gym_run.py                                    # Examples for the Gymnasium aas-gym package
│   │
│   ├── sim_build.sh                                  # Build all dockerfiles for amd64/simulation
│   └── sim_run.sh                                    # Start the simulation (SITL or HITL)
│
└── simulation
    ├── simulation_resources
    │   ├── aircraft_models
    │   │   ├── alti_transition_quad                  # ArduPilot VTOL model
    │   │   ├── iris_with_ardupilot                   # ArduPilot quad model
    │   │   ├── sensor_camera                         # Camera model
    │   │   ├── sensor_lidar                          # LiDAR model
    │   │   ├── standard_vtol                         # PX4 VTOL model
    │   │   └── x500                                  # PX4 quad model
    │   └── simulation_worlds
    │       ├── apple_orchard.sdf
    │       ├── impalpable_greyness.sdf
    │       ├── shibuya_crossing.sdf
    │       └── swiss_town.sdf
    │
    └── simulation.yml.erb                            # Simulation docker tmux entrypoint
```
</details>


<details>
<summary>Dependency management <i>(click to expand)</i></summary>

- [x] Host OS: [Ubuntu 24.04/22.04 (LTS, ESM 4/2034)](https://ubuntu.com/about/release-cycle)
- [ ] Jetpack: [6.2.1 (rev. 1) [L4T 36.4.4, Ubuntu 22-based]](https://developer.nvidia.com/embedded/jetpack-archive)
    - **TODO: test on JP 6.2.2 [L4T 36.5.0, Ubuntu 22-based]**
- [x] [`nvidia-driver-580`](https://developer.nvidia.com/datacenter-driver-archive)
    - **NOTE: `nvidia-driver-590` does not support the [presets in Ubuntu 22's GStreamer 1.20](https://docs.nvidia.com/video-technologies/video-codec-sdk/13.0/deprecation-notices/index.html) and it requires updating the `amd64` base images to Ubuntu 24 or compiling [GStreamer 1.24](https://discourse.gstreamer.org/t/nvcodec-nvenc-nvidia-deprecates-support-for-old-videocodec-sdk-h-264-hevc-encoder-presets-with-driver-r550-in-q124/182) from source**
    - **AAS sticks with `nvidia-driver-580` and Ubuntu 22 `amd64` base images for parity with the L4T 36.x, Ubuntu 22-based `arm64` base image**
- [x] [Docker Engine v29](https://docs.docker.com/engine/release-notes/)
- [x] [NVIDIA Container Toolkit 1.19](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html)
- [x] `amd64` base image: [`cuda:12.9.1-cudnn-runtime-ubuntu22.04`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda/tags)
    - **NOTE: `onnxruntime-gpu` 1.23 does not support CUDA 13**
- [x] `arm64`/Jetson base image: [`l4t-jetpack:r36.4.0`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack/tags)
- [x] [DeepStream 7.1](https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_Installation.html#platform-and-os-compatibility)
    - **NOTE: latest DeepStream supported on Orin Series**
- [x] [ROS2 Humble (LTS, EOL 5/2027)](https://docs.ros.org/en/rolling/Releases.html)
- [x] [Gazebo Sim Harmonic (LTS, EOL 9/2028)](https://gazebosim.org/docs/latest/releases/)
- [x] [PX4 1.16.2](https://github.com/PX4/PX4-Autopilot/releases)
- [x] [ArduPilot 4.6.3](https://github.com/ArduPilot/ardupilot/releases)
- [x] [YOLO26](https://github.com/ultralytics/ultralytics/releases)
- [x] [ONNX Runtime 1.23.2](https://github.com/microsoft/onnxruntime/releases)
    - **NOTE: updating to 1.24 from wheel requires switching to Python >3.11/Ubuntu 24**
</details>

## 1. Installation

```sh
sudo apt update && sudo apt install -y git xterm xfonts-base wget unzip

git clone https://github.com/JacopoPan/aerial-autonomy-stack.git
cd aerial-autonomy-stack/scripts/

./check_requirements.sh                               # If needed, refer to REQUIREMENTS_UBUNTU.md and REQUIREMENTS_WSL.md to install the requirements

# Option A: local build
./sim_build.sh                                        # The 1st build takes ~30GB and ~30' with good internet (`Ctrl + c` and restart if needed, cached stages will be preserved)

# Option B: pull pre-built images (updated every Friday night)
for name in aircraft ground simulation; do
  docker pull ghcr.io/jacopopan/${name}-image:latest
  docker tag ghcr.io/jacopopan/${name}-image:latest ${name}-image:latest
done
```

<div align="right">
  <a href="https://github.com/JacopoPan/aerial-autonomy-stack/actions/workflows/aas-amd64-build-and-test.yml">
    <img src="https://github.com/JacopoPan/aerial-autonomy-stack/actions/workflows/aas-amd64-build-and-test.yml/badge.svg" alt="aas build-and-test amd64">
  </a>
</div>

> [!NOTE]
> AAS is tested on Ubuntu 24.04/22.04 with `nvidia-driver-580` using an i7-11 with 16GB RAM and RTX 3060
>
> Read [`REQUIREMENTS_UBUNTU.md`](/supplementary/REQUIREMENTS_UBUNTU.md) (or [`REQUIREMENTS_WSL.md`](/supplementary/REQUIREMENTS_WSL.md) for Windows 11) to install the requirements

## 2. Simulation

![workspace](https://github.com/user-attachments/assets/ad909fcc-69de-44ac-84b3-c5bc7a1c896f)

```sh
# Start AAS
cd aerial-autonomy-stack/scripts
AUTOPILOT=px4 NUM_QUADS=1 NUM_VTOLS=1 WORLD=swiss_town HEADLESS=false RTF=3.0 ./sim_run.sh    # Start a simulation, check the script for more options (note: ArduPilot SITL checks take ~30-40s of simulated time before being ready to arm)

# Simulation options:
# AUTOPILOT=px4, ardupilot
# HEADLESS/CAMERA/LIDAR=true, false
# NUM_QUADS/NUM_VTOLS=0, 1, ...
# WORLD=impalpable_greyness, apple_orchard, shibuya_crossing, swiss_town
# RTF=1.0, 2.0, ... (real-time-factor, use 0.0 for "as fast as possible)
# INSTANCE=0, 1, ... (integer ID to run multiple parallel simulations)
```

In a new terminal:
```sh
# Fly all drones
for ID in {1..2}; do
  docker exec -d aircraft-container-inst0_$ID bash -c "source /opt/ros/humble/setup.bash &&
    source /aas/github_ws/install/setup.bash && source /aas/aircraft_ws/install/setup.bash &&
    ros2 run mission mission --conops yalla.yaml --ros-args -r __ns:=/Drone$ID -p use_sim_time:=true"
done
```

> [!TIP]
> Edit [`sensor_config.yaml`](simulation/simulation_resources/aircraft_models/sensor_config.yaml), then run `sim_build.sh` to customize the sensor parameters
>
> <details>
> <summary>Add or disable <b>wind effects</b>, in the <kbd>Simulation</kbd>'s Xterm terminal <i>(click to expand)</i></summary>
> 
> ```sh
> python3 /aas/simulation_resources/scripts/gz_wind.py --from_west 0.0 --from_south 3.0
> python3 /aas/simulation_resources/scripts/gz_wind.py --stop_wind
> ```
> </details>
> <details>
> <summary>Use <b>ROS2 drone motion primitives</b> from CLI <i>(click to expand)</i></summary>
>
> ```sh
> # Takeoff action (quads and VTOLs)
> cancellable_action "ros2 action send_goal /Drone${DRONE_ID}/takeoff_action autopilot_interface_msgs/action/Takeoff '{takeoff_altitude: 40.0, vtol_transition_heading: 330.0, vtol_loiter_nord: 200.0, vtol_loiter_east: 100.0, vtol_loiter_alt: 120.0}'"
>
> # Land (at home) action (quads and VTOLs)
> cancellable_action "ros2 action send_goal /Drone${DRONE_ID}/land_action autopilot_interface_msgs/action/Land '{landing_altitude: 60.0, vtol_transition_heading: 60.0}'"
>
> # Orbit action (quads and VTOLs)
> cancellable_action "ros2 action send_goal /Drone${DRONE_ID}/orbit_action autopilot_interface_msgs/action/Orbit '{east: 500.0, north: 0.0, altitude: 150.0, radius: 200.0}'"
>
> # Reposition service (quads only)
> ros2 service call /Drone${DRONE_ID}/set_reposition autopilot_interface_msgs/srv/SetReposition '{east: 50.0, north: 100.0, altitude: 60.0}'
>
> # Offboard action (PX4 quads and VTOLs offboard_setpoint_type: attitude = 0, rates = 1, trajectory = 2; ArduPilot quads offboard_setpoint_type: velocity = 3, acceleration = 4) 
> cancellable_action "ros2 action send_goal /Drone${DRONE_ID}/offboard_action autopilot_interface_msgs/action/Offboard '{offboard_setpoint_type: 1, max_duration_sec: 5.0}'"
>
> # SetSpeed service (always limited by the autopilot params, for quads applies from the next command, not effective on ArduPilot VTOLs) 
> ros2 service call /Drone${DRONE_ID}/set_speed autopilot_interface_msgs/srv/SetSpeed '{speed: 3.0}'
> ```
> To analyze the flight logs in the `Simulation`'s Xterm terminal:
> ```sh
> /aas/simulation_resources/scripts/plot_logs.sh                                                # Analyze the flight logs at http://10.42.90.100:5006/browse or in MAVExplorer
> ```
>
> To create a new mission, re-implement [`test_mission.yaml`](/aircraft/aircraft_resources/missions/test_mission.yaml)
> </details>
> <details>
> <summary><b>Develop within running containers</b> <i>(click to expand)</i></summary>
> 
> Launching the `sim_run.sh` script with `DEV=true`, does **not** start the simulation and mounts folders `[aircraft|ground|simulation]_resources`, `[aircraft|ground]_ws/src` as volumes to more easily track, commit, push changes while building and testing them within the containers:
> 
> ```sh
> cd aerial-autonomy-stack/scripts/
> DEV=true ./sim_run.sh                                                                       # Starts one simulation-image, one ground-image, and one aircraft-image where the *_resources/ and *_ws/src/ folders are mounted from the host
> ```
> 
> To build changes—**made on the host**—in the `Ground` or `QUAD` Xterm terminal:
> 
> ```sh
> cd /aas/aircraft_ws/                                                                        # Or cd /aas/ground_ws/
> colcon build --symlink-install
> ```
> 
> To start the simulation, in the `QUAD` Xterm terminal:
> 
> ```sh
> tmuxinator start -p /aas/aircraft.yml.erb
> ```
> 
> In the `Ground` Xterm terminal:
> ```sh
> tmuxinator start -p /aas/ground.yml.erb
> ```
> 
> In the `Simulation` Xterm terminal:
> ```sh
> tmuxinator start -p /aas/simulation.yml.erb
> ```
> 
> To end the simulation, in each terminal detach Tmux with `Ctrl + b`, then `d`; kill all lingering processes with `tmux kill-server && pkill -f gz`
> </details>
<!-- 
> <details>
> <summary>Use <b>Tmux shortcuts</b> to navigate windows and panes in Xterm <i>(click to expand)</i></summary>
>
> ```sh
> Ctrl + b, then n, p                   # Move between Tmux windows 
> Ctrl + b, then [arrow keys]           # Move between Tmux panes in a window (or use the mouse)
> Ctrl + [, then [arrow keys]           # Enter copy mode (to scroll back in a pane, or simply select-and-drag with the mouse to copy)
> Space                                 # Start selecting when in copy mode (move with arrow keys)
> y                                     # Yank/copy the selection to clipboard (paste with Ctrl + v or Ctrl + Shit + v)
> q                                     # Exit copy mode
> Ctrl + b, then "                      # Split a Tmux window horizontally
> Ctrl + b, then %                      # Split a Tmux window vertically
> Ctrl + b, then d                      # Detach Tmux
> ```
> ```sh
> tmux list-sessions                    # List all sessions
> tmux attach-session -t [session_name] # Reattach a session
> tmux kill-session -t [session_name]   # Kill a session
> tmux kill-server                      # Kill all sessions
> ```
> </details>
> <details>
> <summary>Periodically run <b>Docker cleanups</b> <i>(click to expand)</i></summary>
>
> ```sh
> docker ps -a                          # List containers
> docker stop $(docker ps -q)           # Stop all containers
> docker container prune -f             # Remove all stopped containers
> ```
> ```sh
> docker network ls                     # List docker networks
> docker network rm <network_name>      # Remove a specific network
> docker network prune -f               # Remove all unused networks
> docker system df                      # Check disk usage by images and cache
> docker system prune                   # Remove stopped containers, unused networks and cache, dangling images
> ```
> ```sh
> docker images                         # List images
> docker image prune                    # Remove untagged images
> docker rmi <image_name_or_id>         # Remove a specific image
> docker builder prune                  # Clear all dangling cache
> ```
> </details>
-->

![worlds](https://github.com/user-attachments/assets/b9f7635a-0b1f-4698-ba6a-70ab1b412aef)

> `WORLD`s (in clock-wise order): 
> *(i)* `apple_orchard`, a GIS world created using [BlenderGIS](https://github.com/domlysz/BlenderGIS)
> / *(ii)* `impalpable_greyness`, an empty world with simple shapes
> / *(iii)* `shibuya_crossing`, a 3D world adapted from [cgtrader](https://www.cgtrader.com/)
> / *(iv)* `swiss_town`, a photogrammetry world courtesy of [Pix4D / pix4d.com](https://support.pix4d.com/hc/en-us/articles/360000235126)

## 3. Jetson Deployment

```sh
sudo apt update && sudo apt install -y git

git clone https://github.com/JacopoPan/aerial-autonomy-stack.git
cd aerial-autonomy-stack/scripts/

./deploy_build.sh                                     # Build for arm64, on Jetson Orin NX the first build takes ~50', including building onnxruntime-gpu with TensorRT support from source
```

<div align="right">
  <a href="https://github.com/JacopoPan/aerial-autonomy-stack/actions/workflows/aircraft-arm64-build.yml">
    <img src="https://github.com/JacopoPan/aerial-autonomy-stack/actions/workflows/aircraft-arm64-build.yml/badge.svg" alt="aircraft-image arm64">
  </a>
</div>

>[!NOTE]
> AAS is tested on a [Holybro Jetson Baseboard](https://holybro.com/products/pixhawk-jetson-baseboard) with Pixhawk 6X and NVIDIA Orin NX 16GB on an X650
>
> Read [`SETUP_AVIONICS.md`](/supplementary/SETUP_AVIONICS.md) and [`BOM.md`](/supplementary/BOM.md) to setup the requirements on the Jetson and configure the Pixhawk

Start the `aircraft-image` on Jetson Orin NX:

```sh
cd aerial-autonomy-stack/scripts/
AUTOPILOT=px4 DRONE_ID=1 CAMERA=true LIDAR=false AIR_SUBNET=10.223 HEADLESS=true ./deploy_run.sh
# The 1st run of `./deploy_run.sh` requires ~10' to build the FP16 TensorRT cache

# Deployment options:
# DRONE_TYPE=quad, vtol
# AUTOPILOT=px4, ardupilot
# DRONE_ID=1, 2, ... (ROS_DOMAIN_ID of the drone, matching the MAV_SYS_ID/SYSID_THISMAV of the autpilot)
# HEADLESS/CAMERA/LIDAR=true, false
```

Start the `ground-image` on a laptop to connect QGC, Zenoh, SSH, and GStreamer:

```sh
cd aerial-autonomy-stack/scripts/
./sim_build.sh                                        # Build all images for amd64, including ground-image
GROUND=true NUM_QUADS=1 AIR_SUBNET=10.223 HEADLESS=false ./deploy_run.sh
```

<details>
<summary><b>Advanced Topic: HITL Simulation</b> <i>(click to expand)</i></summary>

> **Note:** HITL simulation validates the Jetson compute and the inter-vehicle network. 
> Use USB2.0 ASIX Ethernet adapters to add multiple network interfaces to the Jetson baseboards

Set up a LAN on an arbitrary `SIM_SUBNET` with netmask `255.255.0.0` (e.g. `172.30.x.x`) between:

- One simulation computer, with IP `[SIM_SUBNET].90.100`
- One ground computer, with IP `[SIM_SUBNET].90.101`
- `N` Jetson Baseboards with IPs `[SIM_SUBNET].90.1`, ..., `[SIM_SUBNET].90.N`

> **Optionally**, set up a second LAN :`AIR_SUBNET` with netmask `255.255.0.0` (e.g. `10.223.x.x`) between:
> 
> - One ground computer, with IP `[AIR_SUBNET].90.101`
> - `N` Jetson Baseboards with IPs `[AIR_SUBNET].90.1`, ..., `[AIR_SUBNET].90.N` 

First, start all aircraft containers, one on each Jetson (e.g. *via* SSH):
```sh
# On the Jetson with IPs ending in 90.1
HITL=true DRONE_ID=1 SIM_SUBNET=172.30 AIR_SUBNET=10.223 ./deploy_run.sh                      # Add HEADLESS=false if a screen is connected to the Jetson
```

```sh
# On the Jetson with IPs ending in 90.2
HITL=true DRONE_ID=2 SIM_SUBNET=172.30 AIR_SUBNET=10.223 ./deploy_run.sh                      # Add HEADLESS=false if a screen is connected to the Jetson
```

Then, start the simulation:
```sh
# On the computer with IPs ending in 90.100
HITL=true NUM_QUADS=2 SIM_SUBNET=172.30 ./sim_run.sh
```

Finally, start QGC and the Zenoh bridge:
```sh
# On the computer with IPs ending in 90.101
HITL=true GROUND=true NUM_QUADS=2 AIR_SUBNET=10.223 HEADLESS=false ./deploy_run.sh
```

> **Note:** running only the first 3 commands with `GND_CONTAINER=false` puts the Zenoh bridge on the `SIM_SUBNET`, removing the need for the optional `AIR_SUBNET` and the computer with IP ending in `90.101`

Once done, detach Tmux (and remove the containers) with `Ctrl + b`, then `d`
</details>

## 4. Gymnasium Environment

<details>
<summary>Using a Python <kbd>venv</kbd> or a <a href="https://docs.conda.io/projects/conda/en/stable/user-guide/install/linux.html"><kbd>conda</kbd></a> environment is optional but recommended <i>(click to expand)</i></summary>

```sh
wget https://repo.anaconda.com/archive/Anaconda3-2025.12-2-Linux-x86_64.sh # Or a newer version in https://repo.anaconda.com/archive/
bash Anaconda3-2025.12-2-Linux-x86_64.sh              # Install; start a new terminal
conda config --set auto_activate_base false           # Turn off auto initialization of (base); start a new terminal
conda update --all -n base -c defaults                # Update to the latest conda version
conda create -n aas python=3.12                       # Latest Python version beyond "bugfix" status https://devguide.python.org/versions/
```

**Optionally**, force CPU/GPU performance modes
```sh
sudo cpupower frequency-set -g performance            # Force CPU performance mode
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor # Check (does NOT persist across reboots)

sudo prime-select nvidia                              # Force GPU use instead of on-demand
prime-select query                                    # Check (persists across reboots)

sudo nvidia-smi -pm 1                                 # Prevent NVIDIA driver from going idle
nvidia-smi --query-gpu=persistence_mode --format=csv,noheader # Check (does NOT persist across reboots)
```
</details>

Install the `aas-gym` package (**after** completing the steps in ["Installation"](#1-installation)):
```sh
conda activate aas                                    # If using Anaconda
cd aerial-autonomy-stack/aas-gym/
pip3 install -e .
```

<div align="right">
  <a href="https://github.com/JacopoPan/aerial-autonomy-stack/actions/workflows/aas-gym-pip-install.yml">
    <img src="https://github.com/JacopoPan/aerial-autonomy-stack/actions/workflows/aas-gym-pip-install.yml/badge.svg" alt="aas-gym pip install">
  </a>
</div>

Use with:
```sh
conda activate aas                                    # If using Anaconda
cd aerial-autonomy-stack/scripts
python3 gym_run.py --mode step                        # Manually step AAS @1Hz
python3 gym_run.py --mode speedup                     # Speed-up test @50Hz
python3 gym_run.py --mode vectorenv-speedup           # Vectorized speed-up test @50Hz
```

<!--
WIP:
python3 gym_run.py --mode learn                       # Train and test a PPO agent

Debug with:
docker exec -it simulation-container-inst0 tmux attach
docker exec -it aircraft-container-inst0_1 tmux attach

Clean up with:
docker stop $(docker ps -q) && docker container prune -f && docker network prune -f
-->

---
> You've done a man's job, sir. I guess you're through, huh?

<!--

## License

Distributed under the MIT License. See `LICENSE.txt` for more information. Copyright (c) 2025 Jacopo Panerati

## Known Issues

- ArduPilot SITL for Iris uses option -f that also sets "external": True, this is not the case for the Alti Transition from ArduPilot/SITL_Models
- QGC will only connect to the first 10 ArduPilot vehicles when GND_CONTAINER=false because of settings in QGroundControl.ini
- On non-configured real-life AP, missing topics: ros2 topic echo /mavros/local_position/odom ros2 topic echo /mavros/home_position/home
- Gazebo WindEffects plugin is disabled/not working for PX4 standard_vtol
- Command 178 MAV_CMD_DO_CHANGE_SPEED is accepted but not effective in changing speed for ArduPilot VTOL
- In ArdupilotInterface's action callbacks, std::shared_lock<std::shared_mutex> lock(node_data_mutex_); could be used on the reads of lat_, lon_, alt_
- QGC does not save roll and pitch in the telemetry bar for PX4 VTOLs (MAV_TYPE 22)
- PX4 quad max tilt is limited by the anti-windup gain (zero it to deactivate it): const float arw_gain = 2.f / _gain_vel_p(0);

## Future Work

### Feature: Betaflight SITL

> Implement a C++ gz-transport/UDP bridge between Gazebo Sim and Betaflight SITL

- https://www.betaflight.com/docs/development/SITL
- https://github.com/Aeroloop/betaloop
- https://github.com/utiasDSL/gym-pybullet-drones/blob/a8c238c21c7586ee1735bafb358a4d5637402f14/gym_pybullet_drones/envs/BetaAviary.py#L111C1-L172C56

### More Out-there Ideas

> Potential for technical spikes/long-term, nice-to-have features

- Integrate a GIS world generator (e.g., Cesium)
    - https://github.com/CesiumGS/cesium-native
- Integrate a photorealistic simulator (e.g., IsaacSim)
    - https://github.com/PegasusSimulator/PegasusSimulator
- Integrate more realistic flight dynamics (e.g., JSBSim)
    - https://github.com/JSBSim-Team/jsbsim
- Integrate a VLA model bridging the `yolo_py` and `mission` packages
- Re-instate Gazebo Sim support for Pixhawk HITL simulation using MAVLink HIL_ interface
    - https://mavlink.io/en/messages/common.html
    - https://github.com/tiiuae/px4-gzsim-plugins/
    - https://docs.px4.io/main/en/simulation/hitl
    - https://ardupilot.org/dev/docs/hitl-simulators.html

-->
