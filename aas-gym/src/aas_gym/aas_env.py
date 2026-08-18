import numpy as np
import gymnasium as gym
import docker
import zmq
import time
import struct
import os
import subprocess
import shutil
import concurrent.futures

from docker.types import DeviceRequest


class AASEnv(gym.Env):
    metadata = {"render_modes": ["human", "ansi"]}

    def __init__(self,
            instance: int=0,
            gym_freq_hz: int=50,
            autopilot: str="px4",
            camera: bool=True,
            lidar: bool=True,
            odom: str="none",
            num_quads: int=1,
            render_mode=None,
            gpu: bool=True
        ):
        super().__init__()

        self.GYM_FREQ_HZ = gym_freq_hz
        self.GYM_INIT_DURATION = 80.0  # Seconds to run unpaused during reset (seconds)
        self.MAX_EPISODE_LENGTH_SEC = 300.0  # Max episode length in seconds (excluding init duration)
        
        # [DUMMY] Action Space: [/action] between -1.0 and 1.0
        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(1,), dtype=np.float32
        )
        # [DUMMY] Observation Space is the Gazebo Sim /clocl [seconds, nanoseconds]
        self.observation_space = gym.spaces.Box(
            low=np.array([0.0, 0.0], dtype=np.float64),
            high=np.array([np.inf, 1e9], dtype=np.float64),
            dtype=np.float64
        )
        # Initialize storage for the clock
        self.sim_sec = 0.0
        self.sim_nanosec = 0.0

        self.max_steps = int(self.MAX_EPISODE_LENGTH_SEC*self.GYM_FREQ_HZ)  # Max steps per episode
        self.step_count = 0
        
        # Rendering
        self.render_mode = render_mode

        # AAS Setup
        self.HEADLESS = False if self.render_mode == "human" else True # Only display GUIs if render_mode is "human"
        self.AUTOPILOT = autopilot
        self.CAMERA = camera
        self.LIDAR = lidar
        self.ODOM = odom
        self.NUM_QUADS = num_quads
        self.NUM_VTOLS = 0 # TODO add support beyond multicopters
        self.NUM_TAILS = 0 # TODO add support beyond multicopters
        self.WORLD = "impalpable_greyness"
        #
        self.SIM_SUBNET = "10.42"
        self.AIR_SUBNET = "10.22"
        self.SIM_ID = "100"
        # self.GROUND_ID = "101" # Unused
        #
        self.GND_CONTAINER = False # Do NOT use the ground-image to run Zenoh (nor QGC)
        self.RTF = 0.0 # As fast as possible (capped to 15x in the simulation.yml.erb)
        self.START_AS_PAUSED = True # Start the simulation paused and manually step with gz-sim WorldControl
        self.INSTANCE = instance
        #
        sim_parts = self.SIM_SUBNET.split('.')
        self.SIM_SUBNET = f"{sim_parts[0]}.{int(sim_parts[1]) + self.INSTANCE}"
        # air_parts = self.AIR_SUBNET.split('.') # Unused
        # self.AIR_SUBNET = f"{air_parts[0]}.{int(air_parts[1]) + self.INSTANCE}" # Unused
        #
        self.SIM_NET_NAME = f"aas-sim-network-inst{self.INSTANCE}"
        # self.AIR_NET_NAME = f"aas-air-network-inst{self.INSTANCE}" # Unused
        self.SIM_CONT_NAME = f"simulation-container-inst{self.INSTANCE}"
        # self.GND_CONT_NAME = f"ground-container-inst{self.INSTANCE}" # Unused

        # X Server access (this is redundancy for configure_host_x11() in gym_run.py)
        if shutil.which("xhost"):
            try:
                current_acls = subprocess.check_output(["xhost"], text=True)
                if "LOCAL:" not in current_acls and "local:docker" not in current_acls:
                    print("Granting X Server access to Docker containers...")
                    subprocess.run(["xhost", "+local:docker"], check=True)
                    print("X Server access granted.")
            except subprocess.CalledProcessError as e:
                print(f"Warning: Could not configure xhost: {e}")

        # Docker setup
        try:
            self.client = docker.from_env()
        except Exception as e:
            raise RuntimeError("Could not connect to the Docker daemon. Ensure Docker is running.") from e
        #
        def force_container_cleanup(name): # Needed to use AsyncVectorEnv
            try:
                old_container = self.client.containers.get(name)
                print(f"Found existing container '{name}'. Removing it...")
                old_container.remove(force=True)
                time.sleep(1.0)
            except docker.errors.NotFound:
                pass
            except Exception as e:
                print(f"Warning during cleanup of {name}: {e}")
        #
        networks_config = [
            {"name": self.SIM_NET_NAME, "subnet_base": self.SIM_SUBNET},
            # {"name": self.AIR_NET_NAME, "subnet_base": self.AIR_SUBNET} # Unused
        ]
        self.networks = {}
        for net_config in networks_config:
            net_name = net_config["name"]
            base_ip = net_config["subnet_base"]
            print(f"Setting up Docker Network: {net_name}...")
            try:
                existing_network = self.client.networks.get(net_name)
                existing_network.remove()
                print(f"Existing network '{net_name}' removed.")
            except docker.errors.NotFound:
                pass
            except docker.errors.APIError as e:
                print(f"Warning: Could not remove {net_name}: {e}")
            ipam_pool = docker.types.IPAMPool(
                subnet=f"{base_ip}.0.0/16",
                gateway=f"{base_ip}.0.1"
            )
            ipam_config = docker.types.IPAMConfig(
                pool_configs=[ipam_pool]
            )
            new_network = self.client.networks.create(
                net_name,
                driver="bridge",
                ipam=ipam_config
            )
            self.networks[net_name] = new_network
            print(f"Network '{net_name}' created on subnet {base_ip}.0.0/16")
        #
        env_display = os.environ.get('DISPLAY', '')
        env_xdg = os.environ.get('XDG_RUNTIME_DIR', '')
        gpu_requests = [
            DeviceRequest(count=-1, capabilities=[['gpu']]) # Replaces "--gpus all"
        ] if gpu else []
        self.ZMQ_IPC_SOCKET_DIR = '/tmp/aas_zmq_sockets'
        os.makedirs(self.ZMQ_IPC_SOCKET_DIR, exist_ok=True)
        volume_binds = {
            '/tmp/.X11-unix': {'bind': '/tmp/.X11-unix', 'mode': 'rw'}, # Replaces "--volume /tmp/.X11-unix:/tmp/.X11-unix:rw"
            self.ZMQ_IPC_SOCKET_DIR: {'bind': self.ZMQ_IPC_SOCKET_DIR, 'mode': 'rw'} # For ZMQ IPC sockets
        }
        device_binds = [
            '/dev/dri:/dev/dri:rwm' # Replaces "--device /dev/dri"
        ] if gpu else []
        #
        force_container_cleanup(self.SIM_CONT_NAME)
        print(f"Creating Simulation Container ({self.SIM_CONT_NAME})...")
        self.simulation_container = self.client.containers.create(
            "simulation-image:latest",
            name=self.SIM_CONT_NAME,
            tty=True, # Replaces -it
            detach=True,
            auto_remove=False,
            privileged=True, # Replaces --privileged
            volumes=volume_binds,
            devices=device_binds,
            device_requests=gpu_requests,
            environment={
                "DISPLAY": env_display,
                "QT_X11_NO_MITSHM": "1",
                **({"NVIDIA_DRIVER_CAPABILITIES": "all",
                    "__NV_PRIME_RENDER_OFFLOAD": "1",
                    "__GLX_VENDOR_LIBRARY_NAME": "nvidia"} if gpu else {}),
                "XDG_RUNTIME_DIR": env_xdg,
                "GST_DEBUG": "3",
                "AUTOPILOT": self.AUTOPILOT,
                "HEADLESS": str(self.HEADLESS).lower(),
                "CAMERA": str(self.CAMERA).lower(),
                "LIDAR": str(self.LIDAR).lower(),
                "NUM_QUADS": str(self.NUM_QUADS),
                "NUM_VTOLS": str(self.NUM_VTOLS),
                "NUM_TAILS": str(self.NUM_TAILS),
                "WORLD": self.WORLD,
                "SIMULATED_TIME": "true",
                "RTF": str(self.RTF),
                "START_AS_PAUSED": str(self.START_AS_PAUSED).lower(),
                "SIM_SUBNET": self.SIM_SUBNET,
                # "GROUND_ID": self.GROUND_ID,
                "GND_CONTAINER": str(self.GND_CONTAINER).lower(),
                "ROS_DOMAIN_ID": self.SIM_ID,
                # "HOST_INPUT_GID": "", # Only necessary to let qgcuser have permissions over /dev/input/, e.g., to use a joystick
                "GYMNASIUM" : "true",
                "GYM_FREQ_HZ" : str(self.GYM_FREQ_HZ),
                "GYM_INIT_DURATION" : str(self.GYM_INIT_DURATION),
                "INSTANCE": str(self.INSTANCE),
            }
        )
        print(f"Connecting {self.SIM_CONT_NAME} to {self.SIM_NET_NAME}...")
        self.networks[self.SIM_NET_NAME].connect(
            self.simulation_container,
            ipv4_address=f"{self.SIM_SUBNET}.90.{self.SIM_ID}"
        )
        # print(f"Connecting {self.SIM_CONT_NAME} to {self.AIR_NET_NAME}...")
        # self.networks[self.AIR_NET_NAME].connect(
        #     self.simulation_container,
        #     ipv4_address=f"{self.AIR_SUBNET}.90.{self.SIM_ID}"
        # )
        # self.simulation_container.start()
        #
        self.drone_types = (
            ["quad"] * self.NUM_QUADS +
            ["vtol"] * self.NUM_VTOLS +
            ["tail"] * self.NUM_TAILS
        )
        self.aircraft_containers = []
        for drone_id, drone_type in enumerate(self.drone_types, start=1):
            air_cont_name = f"aircraft-container-inst{self.INSTANCE}_{drone_id}"
            force_container_cleanup(air_cont_name)
            print(f"Creating Aircraft Container {air_cont_name}...")
            air_cont = self.client.containers.create(
                "aircraft-image:latest",
                name=air_cont_name,
                tty=True, # Replaces -it
                detach=True,
                auto_remove=False,
                privileged=True, # Replaces --privileged
                volumes=volume_binds,
                devices=device_binds,
                device_requests=gpu_requests,
                environment={
                    "DISPLAY": env_display,
                    "QT_X11_NO_MITSHM": "1",
                    **({"NVIDIA_DRIVER_CAPABILITIES": "all",
                        "__NV_PRIME_RENDER_OFFLOAD": "1",
                        "__GLX_VENDOR_LIBRARY_NAME": "nvidia"} if gpu else {}),
                    "XDG_RUNTIME_DIR": env_xdg,
                    "GST_DEBUG": "3",
                    "AUTOPILOT": self.AUTOPILOT,
                    "HEADLESS": str(self.HEADLESS).lower(),
                    "CAMERA": str(self.CAMERA).lower(),
                    "LIDAR": str(self.LIDAR).lower(),
                    "ODOM": self.ODOM,
                    "DRONE_TYPE": drone_type,
                    "DRONE_ID": str(drone_id),
                    "SIMULATED_TIME": "true",
                    "SIM_SUBNET": self.SIM_SUBNET,
                    # "AIR_SUBNET": self.AIR_SUBNET,
                    "SIM_ID": self.SIM_ID,
                    # "GROUND_ID": self.GROUND_ID,
                    "GND_CONTAINER": str(self.GND_CONTAINER).lower(),
                    "ROS_DOMAIN_ID": str(drone_id),
                    "GYMNASIUM" : "true",
                }
            )
            print(f"Connecting {air_cont_name} to {self.SIM_NET_NAME}...")
            self.networks[self.SIM_NET_NAME].connect(
                air_cont,
                ipv4_address=f"{self.SIM_SUBNET}.90.{drone_id}"
            )
            # print(f"Connecting {air_cont_name} to {self.AIR_NET_NAME}...")
            # self.networks[self.AIR_NET_NAME].connect(
            #     air_cont,
            #     ipv4_address=f"{self.AIR_SUBNET}.90.{drone_id}"
            # )
            # air_cont.start()
            self.aircraft_containers.append(air_cont)
        print("Docker setup complete. All containers are running and connected.")

        # ZeroMQ setup
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.REQ)
        self.ZMQ_TRANSPORT = "ipc" # "tcp" or "ipc", also uncomment the corresponding block in zeromq_bridge.cpp
        if self.ZMQ_TRANSPORT == "tcp":
            self.ZMQ_PORT = 5555
            self.ZMQ_IP = f"{self.SIM_SUBNET}.90.{self.SIM_ID}"

    def _get_obs(self):
        return np.array([self.sim_sec, self.sim_nanosec], dtype=np.float64)

    def _get_info(self):
        return {"sim_time_sec": self.sim_sec, "sim_time_nanosec": self.sim_nanosec}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)  # Handle seeding

        # Close existing ZMQ connection if any
        if self.socket:
            self.socket.close()
        # Restart Docker containers
        try:
            print("Restarting all containers in parallel...")
            with concurrent.futures.ThreadPoolExecutor() as executor:
                futures = [executor.submit(self.simulation_container.restart, timeout=1)]
                for air_cont in self.aircraft_containers:
                    futures.append(executor.submit(air_cont.restart, timeout=1))
                for future in concurrent.futures.as_completed(futures):
                    future.result()
        except Exception as e:
            print(f"Error restarting containers: {e}")
            raise e
        # Establish ZeroMQ connection
        self.socket = self.zmq_context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.REQ_RELAXED, 1) # Allow sending a new request if the previous one timed out
        self.socket.setsockopt(zmq.REQ_CORRELATE, 1) # Match replies to requests to avoid processing late replies
        self.socket.setsockopt(zmq.RCVTIMEO, 60 * 1000) # 60s, only a placeholder, will be changed below
        if self.ZMQ_TRANSPORT == "tcp":
            self.socket.connect(f"tcp://{self.ZMQ_IP}:{self.ZMQ_PORT}")
            print(f"ZeroMQ socket connected to {self.ZMQ_IP}:{self.ZMQ_PORT}")
        elif self.ZMQ_TRANSPORT == "ipc":
            ipc_file = f"{self.ZMQ_IPC_SOCKET_DIR}/bridge_inst{self.INSTANCE}.ipc"
            self.socket.connect(f"ipc://{ipc_file}")
            print(f"ZeroMQ socket connected via IPC: {ipc_file}")
        else:
            raise ValueError(f"Invalid ZMQ_TRANSPORT: {self.ZMQ_TRANSPORT}")
        ###########################################################################################
        # ZeroMQ REQ/REP to the ROS2 sim ##########################################################
        ###########################################################################################
        try:
            self.socket.setsockopt(zmq.RCVTIMEO, 300 * 1000) # Temporarily increase timeout to 300s to reset the simulation
            reset = 9999.0 # A special action to reset the environment
            action_payload = struct.pack('d', reset) # Serialize the action 
            for attempt in range(3):
                try:
                    self.socket.send(action_payload) # Send the REQ
                    reply_bytes = self.socket.recv() # Wait for the REP (synchronous block) this call will block until a reply is received or it times out
                    unpacked = struct.unpack('iI', reply_bytes) # Deserialize: i = int32 (sec), I = uint32 (nanosec)
                    self.sim_sec, self.sim_nanosec = unpacked
                    if self.sim_sec == 0 and self.sim_nanosec == 0:
                        raise ValueError("Received zero-time payload (C++ bridge timeout).")
                    self.start_sim_sec = float(self.sim_sec) + (float(self.sim_nanosec) * 1e-9)
                    break # Success
                except zmq.error.Again:
                    print(f"ZMQ Error: Reply from container timed out. Attempt {attempt + 1} of 3.")
                    if attempt == 2:
                        raise RuntimeError("Simulation reset failed: ZMQ timeout.") from None
                except (ValueError, struct.error):
                    print(f"ZMQ Error: Reply format error. Received garbage state. Attempt {attempt + 1} of 3.")
                    if attempt == 2:
                        raise RuntimeError("Simulation reset failed: Invalid data format.") from None
        finally:
            self.socket.setsockopt(zmq.RCVTIMEO, 60 * 1000) # Restore standard timeout (60s) for stepping
        ###########################################################################################
        ###########################################################################################
        ###########################################################################################
        self.step_count = 0
        
        if self.render_mode == "ansi":
            self._render_frame()

        return self._get_obs(), self._get_info()

    def step(self, action):
        force = action[0]
        ###########################################################################################
        # ZeroMQ REQ/REP to the ROS2 sim ##########################################################
        ###########################################################################################
        action_payload = struct.pack('d', force) # Serialize the action
        for attempt in range(3):
            try:
                self.socket.send(action_payload) # Send the REQ
                reply_bytes = self.socket.recv() # Wait for the REP (synchronous block) this call will block until a reply is received or it times out
                unpacked = struct.unpack('iI', reply_bytes) # Deserialize: i = int32 (sec), I = uint32 (nanosec)
                self.sim_sec, self.sim_nanosec = unpacked
                if self.sim_sec == 0 and self.sim_nanosec == 0:
                    raise ValueError("Received zero-time payload (C++ bridge timeout).")
                break # Success
            except zmq.error.Again:
                print(f"ZMQ Error: Reply from container timed out. Attempt {attempt + 1} of 3.")
                if attempt == 2:
                    raise RuntimeError("Simulation step failed: ZMQ timeout.") from None
            except (ValueError, struct.error):
                print(f"ZMQ Error: Reply format error. Received garbage state. Attempt {attempt + 1} of 3.")
                if attempt == 2:
                    raise RuntimeError("Simulation step failed: Invalid data format.") from None
        ###########################################################################################
        ###########################################################################################
        ###########################################################################################
        self.step_count += 1
        # Calculate reward
        reward = float(-1.0)
        # Check for termination
        terminated = False  # This is a continuing task, never "terminates"
        # Check for truncation (episode ends due to time limit)
        truncated = self.step_count >= self.max_steps
        # Get obs and info
        obs = self._get_obs()
        info = self._get_info()
        
        # Handle rendering
        if self.render_mode == "ansi":
            self._render_frame()

        return obs, reward, terminated, truncated, info

    def render(self):
        if self.render_mode == "ansi":
            self._render_frame()

    def _render_frame(self):
        bar_width = 40        
        current_abs_time = self.sim_sec + (self.sim_nanosec * 1e-9)
        start_time = getattr(self, 'start_sim_sec', 0.0)
        episode_time = current_abs_time - start_time
        progress = min(max(episode_time / self.MAX_EPISODE_LENGTH_SEC, 0.0), 1.0)
        filled_len = int(bar_width * progress)
        bar = '=' * filled_len + '-' * (bar_width - filled_len)
        print(f"\r[{bar}] {episode_time:6.2f}s / {self.MAX_EPISODE_LENGTH_SEC:.0f}s", end="")

    def close(self):
        if self.render_mode == "ansi":
            print() # Add a newline after the final render
        
        try:
            self.simulation_container.stop(timeout=1)
            self.simulation_container.remove(force=True)
            print(f"Simulation container '{self.simulation_container.name}' stopped.")
        except Exception:
            pass
        for container in self.aircraft_containers:
            try:
                container.stop(timeout=1)
                container.remove(force=True)
                print(f"Aircraft container '{container.name}' stopped.")
            except Exception:
                pass
        for net_name, network_obj in self.networks.items():
            try:
                network_obj.remove()
                print(f"Network {net_name} removed.")
            except Exception as e:
                print(f"Warning: Could not remove {net_name}: {e}")

        # Close ZMQ resources
        if self.socket:
            self.socket.close(linger=0)
        if self.zmq_context:
            self.zmq_context.term()
