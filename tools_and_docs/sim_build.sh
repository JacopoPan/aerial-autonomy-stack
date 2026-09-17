#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Find the script's path
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Set up the build
BUILD_ADVANCED_ODOM="${BUILD_ADVANCED_ODOM:-false}" # Options: true, false (default), build the advanced odometry and SLAM packages
CLEAN_BUILD="${CLEAN_BUILD:-false}" # Options: true, false (default), rebuild everything from scratch
CLONE_ONLY="${CLONE_ONLY:-false}" # Options: true, false (default), clone the repos and skip the Docker builds

# Check env variables
source "${SCRIPT_DIR}/tests/check_env_vars.sh"
for v in BUILD_ADVANCED_ODOM CLEAN_BUILD CLONE_ONLY; do check_enum "$v" true false; done
print_envvars

BUILD_OPTS=""
if [ "$CLEAN_BUILD" = "true" ]; then
  rm -rf "${SCRIPT_DIR}/../_github_clones"
  BUILD_OPTS="--no-cache" # If CLEAN_BUILD is "true", rebuild everything from scratch
  docker rmi transitional-ros2-image:latest transitional-ros2-qgc-image:latest \
    aircraft-image:latest ground-image:latest simulation-image:latest || true
  docker builder prune -f # Remove all dangling build cache to free up space
fi

# Create a folder (ignored by git) to clone GitHub repos
CLONE_DIR="${SCRIPT_DIR}/../_github_clones"
mkdir -p "$CLONE_DIR"

REPOS=( # Format: "URL;COMMIT;LOCAL_DIR_NAME;TAG" (TAG is optional)
  # Simulation image
  "https://github.com/PX4/PX4-Autopilot.git;d6f12ad1c4f70ad3230afd7d86e971421e02fef4;PX4-Autopilot;v1.17.0"
  "https://github.com/ArduPilot/ardupilot.git;92b0cd788ec29406f26c6f9c31d5ceedbd1cc538;ardupilot;Copter-4.6.3"
  "https://github.com/ArduPilot/ardupilot_gazebo.git;082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5;ardupilot_gazebo"
  "https://github.com/srmainwaring/asv_wave_sim.git;ca8629df4e191235753dfae92ef725d30b923364;asv_wave_sim"
  "https://github.com/PX4/flight_review.git;ea5c16a3bb939f49e50296f953efac40c2456489;flight_review"
  # Ground image
  "https://github.com/mavlink/c_library_v2.git;11fe3b1af2df2e041373601a194b36e612f1c396;c_library_v2"
  "https://github.com/mavlink-router/mavlink-router.git;2362c620f483cef1edd574fb962a373a288e4b9e;mavlink-router"
  # Aircraft image
  "https://github.com/PX4/px4_msgs.git;86d8239e962f6939e05c3737784f60c02fa884db;px4_msgs;v1.17.0"
  "https://github.com/eProsima/Micro-XRCE-DDS-Agent.git;a88c712b41a01e92583f4723f7f4f686863cf657;Micro-XRCE-DDS-Agent;v3.0.2"
  "https://github.com/Livox-SDK/Livox-SDK2.git;08f523c930b2f0ba1e98a6afaa8d7476bf479908;Livox-SDK2"
  "https://github.com/Livox-SDK/livox_ros_driver2.git;4a1def929e5b59c7a8122d19fce6efba581ce9f7;livox_ros_driver2"
  "https://github.com/PRBonn/kiss-icp.git;1ffa7d7512f10bfc8b1185095011fa31184019e3;kiss-icp"
  "https://github.com/rpng/open_vins.git;69488123ed9362dd44b6f28e7f4680abbff1442b;open_vins"
  "https://github.com/MIT-SPARK/spark-fast-lio.git;17b36d293a14df37d57e1751a337a32e2f164692;spark-fast-lio"
  "https://github.com/MIT-SPARK/KISS-Matcher.git;8b58b118e00c7a5a22448741f8d956d638a44ae2;KISS-Matcher"
  "https://github.com/superxslam/SuperOdom.git;f10e65cd50007767b22e4c401689665e20d827d6;SuperOdom"
  "https://github.com/ntnu-arl/mimosa.git;43d44ae2d1bbfff7c8774a35d846451bc4e29878;mimosa"
  "https://github.com/JacopoPan/rovio_ros2.git;ff9b20e6801e9488cefc33d1a47821b8a009acea;rovio"
)

for repo_info in "${REPOS[@]}"; do
  IFS=';' read -r url commit dir tag <<< "$repo_info" # Split the string into URL, COMMIT, DIR, and optional TAG
  TARGET_DIR="${CLONE_DIR}/${dir}"
  if [ -d "$TARGET_DIR" ] && [ "$(git -C "$TARGET_DIR" rev-parse HEAD 2>/dev/null)" != "$commit" ]; then
    echo "${dir} is not at the pinned commit, re-cloning..."
    rm -rf "$TARGET_DIR" # Without this, changing a pin would silently keep the old clone
  fi
  if [ -d "$TARGET_DIR" ]; then
    TAGS=$(git -C "$TARGET_DIR" tag --points-at HEAD | paste -sd, -)
    HEAD_INFO=$(git -C "$TARGET_DIR" log -1 --format='%h %cs (%cr)')
    printf '%-30s %-20s %s\n' "$dir" "$TAGS" "$HEAD_INFO"
  else
    echo "Clone not found, cloning ${dir}..."
    TEMP_DIR="${TARGET_DIR}_temp"
    rm -rf "$TEMP_DIR" # Clean up any failed clone from a previous run
    for attempt in 1 2 3; do # Up to 3 tries
      git init -q "$TEMP_DIR" \
        && git -C "$TEMP_DIR" remote add origin "$url" \
        && git -C "$TEMP_DIR" fetch -q --depth 1 origin "$commit" \
        && { [ -z "$tag" ] || git -C "$TEMP_DIR" fetch -q --depth 1 origin tag "$tag"; } \
        && git -C "$TEMP_DIR" checkout -q "$commit" \
        && git -C "$TEMP_DIR" submodule update --init --recursive --depth 1 \
        && break
      echo "Clone of ${dir} failed (attempt ${attempt}/3), retrying..."
      rm -rf "$TEMP_DIR"
      sleep 2
    done
    [ -d "$TEMP_DIR" ] || { echo "ERROR: could not clone ${dir}"; exit 1; }
    mv "$TEMP_DIR" "$TARGET_DIR"
  fi
done

# Get simulation_assets from GitHub release
ASSETS_URL="https://github.com/JacopoPan/aerial-autonomy-stack/releases/download/v1.5.0/simulation_assets_v5.zip"
EXPECTED_HASH="b04b5c983929ffb633acc265ad258f1fdc37d3a2e5af4db7c95cf90369d552b2" # sha256sum simulation_assets_v5.zip
ZIP_FILE="$CLONE_DIR/simulation_assets_v5.zip" # Created by zipping the simulation/ folder with `zip -r simulation_assets_v5.zip simulation`
DOWNLOAD_NEEDED=true
if [ -f "$ZIP_FILE" ]; then
  CURRENT_HASH=$(sha256sum "$ZIP_FILE" | awk '{print $1}')
  if [ "$CURRENT_HASH" = "$EXPECTED_HASH" ]; then
    echo "simulation_assets_v5.zip already downloaded"
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

if [ "$CLONE_ONLY" = "true" ]; then
  echo -e "Skipping Docker builds"
else
  # Build common layers reused between images
  docker build $BUILD_OPTS --target ros2-image -t transitional-ros2-image -f "${SCRIPT_DIR}/docker/aircraft.dockerfile" "${SCRIPT_DIR}/.."
  docker build $BUILD_OPTS --target ros2-qgc-image -t transitional-ros2-qgc-image -f "${SCRIPT_DIR}/docker/ground.dockerfile" "${SCRIPT_DIR}/.."
  # Build the 3 main images
  docker build $BUILD_OPTS --build-arg BUILD_ADVANCED_ODOM="${BUILD_ADVANCED_ODOM}" -t aircraft-image -f "${SCRIPT_DIR}/docker/aircraft.dockerfile" "${SCRIPT_DIR}/.."
  docker build $BUILD_OPTS -t ground-image -f "${SCRIPT_DIR}/docker/ground.dockerfile" "${SCRIPT_DIR}/.."
  docker build $BUILD_OPTS -t simulation-image -f "${SCRIPT_DIR}/docker/simulation.dockerfile" "${SCRIPT_DIR}/.."
fi
