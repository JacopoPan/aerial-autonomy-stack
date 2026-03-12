#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Find the script's path
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

if [ "${CLEAN_BUILD:-false}" = "true" ]; then
  rm -rf "${SCRIPT_DIR}/../github_clones"
  docker rmi aircraft-image:latest ground-image:latest simulation-image:latest || true
  docker builder prune -f # If CLEAN_BUILD is "true", rebuild everything from scratch
fi

BUILD_DOCKER=true
if [ "${CLONE_ONLY:-false}" = "true" ]; then
  BUILD_DOCKER=false # If CLONE_ONLY is "true", disable the build steps
fi

# Create a folder (ignored by git) to clone GitHub repos
CLONE_DIR="${SCRIPT_DIR}/../github_clones"
mkdir -p "$CLONE_DIR"

REPOS=( # Format: "URL;BRANCH;LOCAL_DIR_NAME"
  # Simulation image
  "https://github.com/ArduPilot/ardupilot.git;Copter-4.6.2;ardupilot"
  "https://github.com/ArduPilot/ardupilot_gazebo.git;main;ardupilot_gazebo"
  "https://github.com/PX4/flight_review.git;main;flight_review"
  # Ground image
  "https://github.com/mavlink/c_library_v2;master;c_library_v2"
  "https://github.com/mavlink-router/mavlink-router;master;mavlink-router"
  # Aircraft image
  # Add repos for PDS-Swarm
  # FIX: (mateus) clonei manualmente pq por aqui reclamava de incorrect access rights
  "git@github.com:droneswarmufpe/Chorus.git;main;Chorus"
  "git@github.com:droneswarmufpe/MAVKit.git;main;MAVKit"
  "git@github.com:droneswarmufpe/RoboChart2Python-PDS;main;RoboChart2Python-PDS"
  "git@github.com:droneswarmufpe/Sistemas.git;main;Sistemas"
  "git@github.com:droneswarmufpe/Projeto-Enxame-Drones.git;main;Projeto-Enxame-Drones"
  # TODO: add SARP cloning here
  # "https://github.com/microsoft/onnxruntime.git;v1.22.1;onnxruntime" # Only for the deployment build
  "https://github.com/PRBonn/kiss-icp.git;main;kiss-icp"
)

pgrep ssh-agent > /dev/null || eval "$(ssh-agent -s)"

if ! ssh-add -l 2>/dev/null | grep -q "SHA256:"; then
  echo "Error: No SSH keys loaded in ssh-agent." >&2
  exit 1
fi

for repo_info in "${REPOS[@]}"; do
  IFS=';' read -r url branch dir <<< "$repo_info" # Split the string into URL, BRANCH, and DIR
  TARGET_DIR="${CLONE_DIR}/${dir}"
  if [ -d "$TARGET_DIR" ]; then
    cd "$TARGET_DIR"
    BRANCH=$(git branch --show-current)
    TAGS=$(git tag --points-at HEAD)
    echo "There is a clone of ${dir} on branch: ${BRANCH}, tags: [${TAGS}]"
    # The script does not automatically pull changes for already cloned repos (as they should be on fixed tags)
    # git pull
    # git submodule update --init --recursive --depth 1
    cd "$CLONE_DIR"
  else
    echo "Clone not found, cloning ${dir}..."
    TEMP_DIR="${TARGET_DIR}_temp"     
    rm -rf "$TEMP_DIR" # Clean up any failed clone from a previous run   
    git clone --depth 1 --branch "$branch" --recursive "$url" "$TEMP_DIR" && mv "$TEMP_DIR" "$TARGET_DIR"
  fi
done

if [ "$BUILD_DOCKER" = "true" ]; then
  # Make sure AAS's Git LFS simulation resources are pulled
  git lfs install
  git lfs pull

  # The first build takes ~15' and creates a 21GB image (8GB for ros-humble-desktop with nvidia runtime, 10GB for PX4 and ArduPilot SITL)
  docker build -t simulation-image -f "${SCRIPT_DIR}/docker/Dockerfile.simulation" "${SCRIPT_DIR}/.."

  # The first build takes <5' and creates an 9GB image (8GB for ros-humble-desktop with nvidia runtime)
  docker build -t ground-image -f "${SCRIPT_DIR}/docker/Dockerfile.ground" "${SCRIPT_DIR}/.."

  # The first build takes ~10' and creates an 18GB image (8GB for ros-humble-desktop with nvidia runtime, 7GB for YOLOv8, ONNX)
  docker build -t aircraft-image -f "${SCRIPT_DIR}/docker/Dockerfile.aircraft" "${SCRIPT_DIR}/.."
else
  echo -e "Skipping Docker builds"
fi