#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

if [ "$#" -ne 4 ]; then
  echo "Usage: $0 <num_quads> <num_vtols> <num_tails> <full_path_to_empty_world>"
  echo "Example: ./_create_ardupilot_world.sh 3 2 1 /aas/simulation_resources/simulation_worlds/impalpable_greyness.sdf"
  exit 1
fi

NUM_QUADS=$1
NUM_VTOLS=$2
NUM_TAILS=$3
BASE_WORLD_WITH_PATH=$4

# Resolve the path relative to the script's directory if it's not absolute
if [[ "$BASE_WORLD_WITH_PATH" != /* ]]; then
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  BASE_WORLD_WITH_PATH="${SCRIPT_DIR}/${BASE_WORLD_WITH_PATH}"
fi

# Create a copy of the template to work on
BASE_WORLD_DIR=$(dirname "$BASE_WORLD_WITH_PATH")
OUTPUT_FILE="${BASE_WORLD_DIR}/populated_ardupilot.sdf"
cp "$BASE_WORLD_WITH_PATH" "$OUTPUT_FILE"

# Capture the current real_time_factor value, default to 1.0 if not found
RTF_VALUE=$(sed -n '/<physics/,/<\/physics>/ s/.*<real_time_factor>\([^<]*\)<\/real_time_factor>.*/\1/p' "$OUTPUT_FILE")
RTF_VALUE=${RTF_VALUE:-1.0}

# IMPORTANT: this replaces the whole <physics> block with Ardupilot's SITL settings
#
# The default step size for Ardupilot SITL is 1ms (1000Hz), here it is set to 2ms (500Hz)
# Note that PX4 Gazebo simulation worlds use 4ms: https://github.com/PX4/PX4-gazebo-models/tree/main/worlds
# To do so, we also set SCHED_LOOP_RATE 250 (instead of the original 400 for Iris and 300 for Alti) in the vehicle's .params files
# This is required to pass pre-flight checks that expect sensor updates must be >1.8x faster (250 * 1.8 = 450 < 500Hz)
#
# For discussion on Gazebo faster-than-real-time multi-vehicle ArduPilot see also:
# https://discuss.ardupilot.org/t/multi-vehicle-faster-than-real-time-sitl-with-gazebo-harmonic/141068/3
# https://discuss.ardupilot.org/t/dual-vtail-mini-talon-gazebo-simulation-behaves-poorly/140919/11
ARDUPILOT_PHYSICS="    <physics name=\"2ms\" type=\"ignore\">\n      <max_step_size>0.002<\/max_step_size>\n      <real_time_factor>${RTF_VALUE}<\/real_time_factor>\n    <\/physics>"
sed -i -e "/<physics/,/<\/physics>/c\\
${ARDUPILOT_PHYSICS}" "$OUTPUT_FILE"

# This loop builds a single string containing all the <include> blocks
ALL_MODELS_XML=""
DRONE_ID=0

# Loop for quads
for i in $(seq 1 $NUM_QUADS); do
  DRONE_ID=$((DRONE_ID + 1))
  MODEL_XML="    <include>\n      <uri>model://iris_with_ardupilot_${DRONE_ID}</uri>\n      <pose degrees=\"true\">$(( (i-1) * 2 )) $(( -1 + (i-1) * 2 )) 0.75 0 0 0</pose>\n    </include>\n"
  ALL_MODELS_XML+=$MODEL_XML
done

# Loop for VTOLs
for i in $(seq 1 $NUM_VTOLS); do
  DRONE_ID=$((DRONE_ID + 1))
  MODEL_XML="    <include>\n      <uri>model://alti_transition_quad_${DRONE_ID}</uri>\n      <pose degrees=\"true\">$(( (i-1) * 2 )) $(( 2 + (i-1) * 2 )) 0.75 0 0 0</pose>\n    </include>\n"
  ALL_MODELS_XML+=$MODEL_XML
done

# Loop for tails
for i in $(seq 1 $NUM_TAILS); do
  DRONE_ID=$((DRONE_ID + 1))
  MODEL_XML="    <include>\n      <uri>model://swan_k1_hwing_${DRONE_ID}</uri>\n      <pose degrees=\"true\">$(( (i-1) * 2 )) $(( 5 + (i-1) * 2 )) 0.75 0 -90 0</pose>\n    </include>"
  ALL_MODELS_XML="${ALL_MODELS_XML}\n${MODEL_XML}"
done

# Read the file, replace the tag, and write the content back out
WORLD_CONTENT=$(cat "$OUTPUT_FILE")
echo "${WORLD_CONTENT//'</world>'/"$ALL_MODELS_XML</world>"}" > "$OUTPUT_FILE"
