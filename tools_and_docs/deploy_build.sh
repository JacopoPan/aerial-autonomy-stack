#!/bin/bash

# Note: this script builds for arm64 and is tested on NVIDIA Jetson Orin NX 16GB

# Exit immediately if a command exits with a non-zero status
set -e

# Find the script's path
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

BUILD_ARGS=""
if [ "${CLEAN_BUILD:-false}" = "true" ]; then
  rm -rf "${SCRIPT_DIR}/../_github_clones"
  BUILD_ARGS="--no-cache" # If CLEAN_BUILD is "true", rebuild everything from scratch
  docker rmi aircraft-image:latest || true
  docker builder prune -f # Remove all dangling build cache to free up space
fi

BUILD_DOCKER=true
if [ "${CLONE_ONLY:-false}" = "true" ]; then
  BUILD_DOCKER=false # If CLONE_ONLY is "true", disable the build steps
fi

# Create a folder (ignored by git) to clone GitHub repos
CLONE_DIR="${SCRIPT_DIR}/../_github_clones"
mkdir -p "$CLONE_DIR"

REPOS=( # Format: "URL;BRANCH;LOCAL_DIR_NAME"
  # Aircraft image
  "https://github.com/microsoft/onnxruntime.git;v1.23.2;onnxruntime" # Only for the deployment build
  "https://github.com/PX4/px4_msgs.git;release/1.17;px4_msgs"
  "https://github.com/eProsima/Micro-XRCE-DDS-Agent.git;master;Micro-XRCE-DDS-Agent"
  "https://github.com/Livox-SDK/Livox-SDK2.git;master;Livox-SDK2" # Only for the deployment build
  "https://github.com/Livox-SDK/livox_ros_driver2.git;master;livox_ros_driver2" # Only for the deployment build
  "https://github.com/PRBonn/kiss-icp.git;main;kiss-icp"
)

for repo_info in "${REPOS[@]}"; do
  IFS=';' read -r url branch dir <<< "$repo_info" # Split the string into URL, BRANCH, and DIR
  TARGET_DIR="${CLONE_DIR}/${dir}"
  if [ -d "$TARGET_DIR" ]; then
    cd "$TARGET_DIR"
    BRANCH=$(git branch --show-current)
    TAGS=$(git tag --points-at HEAD)
    echo "There is a clone of ${dir} on branch: ${BRANCH}, tags: [${TAGS}]"
    # The script does not automatically pull changes for already cloned repos (as they should be on fixed tags)
    # This avoids breaking the Docker cache but it requires manually deleting the _github_clones folder for branch/tag updates
    # git pull
    # git submodule update --init --recursive --depth 1
    cd "$CLONE_DIR"
  else
    echo "Clone not found, cloning ${dir}..."
    TEMP_DIR="${TARGET_DIR}_temp"     
    rm -rf "$TEMP_DIR" # Clean up any failed clone from a previous run   
    git clone --depth 1 --shallow-submodules --branch "$branch" --recursive "$url" "$TEMP_DIR" && mv "$TEMP_DIR" "$TARGET_DIR"
  fi
done

if [ "$BUILD_DOCKER" = "true" ]; then
  # The first build takes ~50'
  docker build $BUILD_ARGS -t aircraft-image -f "${SCRIPT_DIR}/docker/aircraft.dockerfile" "${SCRIPT_DIR}/.."
else
  echo -e "Skipping Docker build"
fi
