#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Set up the aircraft
AUTOPILOT="${AUTOPILOT:-px4}" # Options: px4 (default), ardupilot
HEADLESS="${HEADLESS:-true}" # Options: true (default), false 
CAMERA="${CAMERA:-true}" # Options: true (default), false
LIDAR="${LIDAR:-true}" # Options: true (default), false
#
SIM_SUBNET="${SIM_SUBNET:-10.42}" # Simulation subnet (default = 10.42)
AIR_SUBNET="${AIR_SUBNET:-10.22}" # Inter-vehicle subnet (default = 10.22)
SIM_ID="${SIM_ID:-100}" # Last byte of the simulation container IP (default = 100)
GROUND_ID="${GROUND_ID:-101}" # Last byte of the simulation container IP (default = 101)
#
DRONE_TYPE="${DRONE_TYPE:-quad}" # Options: quad (default), vtol
DRONE_ID="${DRONE_ID:-1}" # Id of aircraft (default = 1)
#
DEV="${DEV:false}" # Options: true, false (default)
HITL="${HITL:-false}" # Options: true, false (default)
GND_CONTAINER="${GND_CONTAINER:-true}" # Options: true (default), false
# Only used by ground-container
NUM_QUADS="${NUM_QUADS:-1}" # Number of quadcopters (default = 1)
NUM_VTOLS="${NUM_VTOLS:-0}" # Number of VTOLs (default = 0)

GROUND="${GROUND:-false}" # Options: true, false (default)
if [[ "$GROUND" == "true" ]]; then
  # This is a bit hacky, but allows to use the deploy_run.sh script for the ground container
  # Without GPU requirements: --device /dev/dri --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all
  # Forcing HEADLESS to false, opening REMOTE_VIDEO_STREAMS and SSH_CONNECTIONS
  xhost +local:docker # Grant access to the X server
  docker run -it --rm \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    --env HEADLESS=false \
    --env NUM_QUADS=$NUM_QUADS --env NUM_VTOLS=$NUM_VTOLS \
    --env SIMULATED_TIME=$HITL \
    --env ROS_DOMAIN_ID=$GROUND_ID \
    --env AIR_SUBNET=$AIR_SUBNET \
    --env HOST_INPUT_GID=$(getent group input | cut -d: -f3) \
    --env REMOTE_VIDEO_STREAMS=true \
    --env SSH_CONNECTIONS=true \
    --net=host \
    --privileged \
    --name ground-container \
    ground-image
  exit 0
fi

# In dev mode, resources and workspaces are mounted from the host
if [[ "$DEV" == "true" ]]; then
  SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
  DEV_OPTS="--entrypoint /bin/bash"
  DEV_OPTS+=" -v ${SCRIPT_DIR}/../aircraft/aircraft_resources/:/aas/aircraft_resources:cached"
  DEV_OPTS+=" -v ${SCRIPT_DIR}/../aircraft/aircraft_ws/src:/aas/aircraft_ws/src:cached"
  DEV_OPTS+=" -v ${SCRIPT_DIR}/../ground/ground_ws/src/ground_system_msgs:/aas/aircraft_ws/src/ground_system_msgs:cached"
fi

if [ "$HEADLESS" = "false" ]; then
  # Grant access to the X server
  xhost +local:docker # Avoid this when building the TensorRT cache for the first time
fi

if [ "$HITL" = "true" ]; then
  DOCKER_RUN_FLAGS="-it --rm" # Interactive mode with auto-remove
else
  DOCKER_RUN_FLAGS="-d -t" # Detached mode
  if [[ "$DEV" == "true" ]]; then
    echo -e "\nWith DEV=true, attach directly to the bash shell:\n"
    echo -e "\t docker exec -it aircraft-container_$DRONE_ID bash\n"
  else
    echo -e "\nAttach to the Tmux session in the running 'aircraft-container':\n"
    echo -e "\t docker exec -it aircraft-container_$DRONE_ID tmux attach\n"
  fi
  echo -e "To stop all containers and remove stopped containers:\n"
  echo -e '\t docker stop $(docker ps -q) && docker container prune -f\n'
fi

# Launch the aircraft container
docker run $DOCKER_RUN_FLAGS \
  --runtime nvidia \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
  --volume /tmp/argus_socket:/tmp/argus_socket --volume ~/tensorrt_cache/:/tensorrt_cache --device=/dev/ttyTHS1:/dev/ttyTHS1 \
  --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR --env GST_DEBUG=3 \
  --env AUTOPILOT=$AUTOPILOT --env HEADLESS=$HEADLESS --env CAMERA=$CAMERA --env LIDAR=$LIDAR \
  --env HITL=$HITL --env SIMULATED_TIME=$HITL \
  --env DRONE_TYPE=$DRONE_TYPE --env DRONE_ID=$DRONE_ID \
  --env SIM_SUBNET=$SIM_SUBNET --env AIR_SUBNET=$AIR_SUBNET --env SIM_ID=$SIM_ID --env GROUND_ID=$GROUND_ID \
  --env GND_CONTAINER=$GND_CONTAINER \
  --env ROS_DOMAIN_ID=$DRONE_ID \
  --env REMOTE_VIDEO_STREAMS=true \
  --net=host \
  --privileged \
  --name aircraft-container_$DRONE_ID \
  ${DEV_OPTS} \
  aircraft-image

# Check ONNX runtimes
# DEV=true HEADLESS=false ./deploy_run.sh
# docker exec -it aircraft-container bash
# python3 -c "import onnxruntime as ort; print(ort.__version__); print(ort.get_available_providers())"
# tmuxinator start -p /aas/aircraft.yml.erb
