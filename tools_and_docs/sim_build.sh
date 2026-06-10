#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Find the script's path
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# By default, skip building advanced odometry, SLAM packages
BUILD_ADVANCED_ODOM=${EXTRAS:-false}

BUILD_OPTS=""
if [ "${CLEAN_BUILD:-false}" = "true" ]; then
  rm -rf "${SCRIPT_DIR}/../_github_clones"
  BUILD_OPTS="--no-cache" # If CLEAN_BUILD is "true", rebuild everything from scratch
  docker rmi transitional-ros2-image:latest transitional-ros2-qgc-image:latest \
    aircraft-image:latest ground-image:latest simulation-image:latest || true
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
  # Simulation image
  "https://github.com/PX4/PX4-Autopilot.git;v1.17.0;PX4-Autopilot"
  "https://github.com/ArduPilot/ardupilot.git;Copter-4.6.3;ardupilot"
  "https://github.com/ArduPilot/ardupilot_gazebo.git;main;ardupilot_gazebo"
  "https://github.com/srmainwaring/asv_wave_sim.git;master;asv_wave_sim"
  "https://github.com/PX4/flight_review.git;main;flight_review"
  # Ground image
  "https://github.com/mavlink/c_library_v2.git;master;c_library_v2"
  "https://github.com/mavlink-router/mavlink-router.git;master;mavlink-router"
  # Aircraft image
  "https://github.com/PX4/px4_msgs.git;release/1.17;px4_msgs"
  "https://github.com/eProsima/Micro-XRCE-DDS-Agent.git;master;Micro-XRCE-DDS-Agent"
  "https://github.com/Livox-SDK/Livox-SDK2.git;master;Livox-SDK2"
  "https://github.com/Livox-SDK/livox_ros_driver2.git;master;livox_ros_driver2"
  "https://github.com/PRBonn/kiss-icp.git;main;kiss-icp"
  "https://github.com/rpng/open_vins.git;master;open_vins"
  "https://github.com/MIT-SPARK/spark-fast-lio.git;main;spark-fast-lio"
  "https://github.com/MIT-SPARK/KISS-Matcher.git;main;KISS-Matcher"
  "https://github.com/superxslam/SuperOdom.git;ros2;SuperOdom"
  "https://github.com/ntnu-arl/mimosa.git;dev/ros2;mimosa"
  "https://github.com/JacopoPan/rovio_ros2.git;main;rovio"
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

# Get simulation_assets from GitHub release
ASSETS_URL="https://github.com/JacopoPan/aerial-autonomy-stack/releases/download/v1.3.0/simulation_assets_v3.zip"
EXPECTED_HASH="c6ce5842af2ecefe9f123f8cb53c16c1cdb85964b66d8916aec4821178a012b5" # sha256sum simulation_assets_v3.zip
ZIP_FILE="$CLONE_DIR/simulation_assets_v3.zip"
DOWNLOAD_NEEDED=true
if [ -f "$ZIP_FILE" ]; then
  CURRENT_HASH=$(sha256sum "$ZIP_FILE" | awk '{print $1}')
  if [ "$CURRENT_HASH" = "$EXPECTED_HASH" ]; then
    echo "simulation_assets_v3.zip already downloaded"
    DOWNLOAD_NEEDED=false
  fi
fi
if [ "$DOWNLOAD_NEEDED" = "true" ]; then
  echo "Downloading simulation assets from $ASSETS_URL..."
  wget -q --show-progress \
      --tries=3 \
      --retry-connrefused \
      --retry-on-http-error=403,429,500,502,503,504 \
      --waitretry=5 \
      --timeout=15 \
      -O "$ZIP_FILE" "$ASSETS_URL"
  DOWNLOAD_HASH=$(sha256sum "$ZIP_FILE" | awk '{print $1}')
  if [ "$DOWNLOAD_HASH" != "$EXPECTED_HASH" ]; then
    echo -e "ERROR: The downloaded file hash is incorrect\nExpected: $EXPECTED_HASH\nGot: $DOWNLOAD_HASH"
    exit 1 # Stop the script
  fi
fi
# Unzip quietly (-q), overwrite (-o), into the repository root directory (-d) above tools_and_docs/ to merge into simulation/
unzip -q -o "$ZIP_FILE" -d "$SCRIPT_DIR/.."

if [ "$BUILD_DOCKER" = "true" ]; then
  # Build common layers reused between images
  docker build $BUILD_OPTS --target ros2-image -t transitional-ros2-image -f "${SCRIPT_DIR}/docker/aircraft.dockerfile" "${SCRIPT_DIR}/.."
  docker build $BUILD_OPTS --target ros2-qgc-image -t transitional-ros2-qgc-image -f "${SCRIPT_DIR}/docker/ground.dockerfile" "${SCRIPT_DIR}/.."
  # Build the 3 main images
  docker build $BUILD_OPTS --build-arg BUILD_ADVANCED_ODOM="${BUILD_ADVANCED_ODOM}" -t aircraft-image -f "${SCRIPT_DIR}/docker/aircraft.dockerfile" "${SCRIPT_DIR}/.."
  docker build $BUILD_OPTS -t ground-image -f "${SCRIPT_DIR}/docker/ground.dockerfile" "${SCRIPT_DIR}/.."
  docker build $BUILD_OPTS -t simulation-image -f "${SCRIPT_DIR}/docker/simulation.dockerfile" "${SCRIPT_DIR}/.."
else
  echo -e "Skipping Docker builds"
fi
