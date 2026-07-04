#!/bin/bash

# This script dynamically generates the SDF files for multiple ArduPilot vehicles and a world SDF file containing them

# Exit immediately if a command exits with a non-zero status
set -e

if [ "$#" -ne 4 ]; then
  echo "Usage: $0 <num_quads> <num_vtols> <num_tails> <absolute_path_to_empty_world>"
  echo "Example: ./create_ardupilot_drones_and_world.sh 3 2 1 /aas/simulation_resources/simulation_worlds/impalpable_greyness.sdf"
  exit 1
fi

NUM_QUADS=$1
NUM_VTOLS=$2
NUM_TAILS=$3
BASE_WORLD_WITH_PATH=$4

# Paths to the base models
QUAD_MODEL_PATH="/aas/simulation_resources/aircraft_models/iris_with_ardupilot"
VTOL_MODEL_PATH="/aas/simulation_resources/aircraft_models/alti_transition_quad"
TAIL_MODEL_PATH="/aas/simulation_resources/aircraft_models/swan_k1_hwing"

# Check if model directories exist
check_model() {
    if [ ! -d "$1" ] && [ "$2" -gt 0 ]; then
        echo "Error: $3 model directory '$1' not found."
        exit 1
    fi
}
check_model "$QUAD_MODEL_PATH" "$NUM_QUADS" "Quad"
check_model "$VTOL_MODEL_PATH" "$NUM_VTOLS" "VTOL"
check_model "$TAIL_MODEL_PATH" "$NUM_TAILS" "Tail"

echo "Creating ${NUM_QUADS} quadcopter(s), ${NUM_VTOLS} VTOL(s), and ${NUM_TAILS} tailsitters(s)..."
BASE_AP_PLUGIN_PORT=9002
create_model() {
    local BASE_MODEL_PATH=$1
    local DRONE_ID=$2
    
    BASE_MODEL_NAME=$(basename "$BASE_MODEL_PATH")
    NEW_MODEL_NAME="${BASE_MODEL_NAME}_${DRONE_ID}"
    NEW_MODEL_DIR="${BASE_MODEL_PATH}/../${NEW_MODEL_NAME}"

    mkdir -p "$NEW_MODEL_DIR"
    cp "$BASE_MODEL_PATH"/model.sdf "$NEW_MODEL_DIR"/
    cp "$BASE_MODEL_PATH"/model.config "$NEW_MODEL_DIR"/

    sed -i "s/<model name=\"${BASE_MODEL_NAME}\">/<model name=\"${NEW_MODEL_NAME}\">/g" "${NEW_MODEL_DIR}/model.sdf"
    sed -i "s/<fdm_port_in>${BASE_AP_PLUGIN_PORT}<\/fdm_port_in>/<fdm_port_in>$(($BASE_AP_PLUGIN_PORT + ($DRONE_ID - 1) * 10))<\/fdm_port_in>/g" "${NEW_MODEL_DIR}/model.sdf"

    DEST_PARAMS="${NEW_MODEL_DIR}/ardupilot-4.6.params"
    cp "${BASE_MODEL_PATH}/ardupilot-4.6.params" "$DEST_PARAMS"
}

# Create a copy of the world SDF to add the drone models to
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

# This loops create the drone models (using the ArduPilot plugin on different ports starting from 9002) and adds them to the world SDF
ALL_MODELS_XML=""
DRONE_ID=0
# Loop for quads
for i in $(seq 1 $NUM_QUADS); do
  DRONE_ID=$((DRONE_ID + 1))
  create_model "$QUAD_MODEL_PATH" "$DRONE_ID"
  MODEL_XML="    <include>\n      <uri>model://iris_with_ardupilot_${DRONE_ID}</uri>\n      <pose degrees=\"true\">$(( (i-1) * 2 )) $(( -1 + (i-1) * 2 )) 0.75 0 0 0</pose>\n    </include>\n"
  ALL_MODELS_XML+=$MODEL_XML
done
# Loop for VTOLs
for i in $(seq 1 $NUM_VTOLS); do
  DRONE_ID=$((DRONE_ID + 1))
  create_model "$VTOL_MODEL_PATH" "$DRONE_ID"
  MODEL_XML="    <include>\n      <uri>model://alti_transition_quad_${DRONE_ID}</uri>\n      <pose degrees=\"true\">$(( (i-1) * 2 )) $(( 2 + (i-1) * 2 )) 0.75 0 0 0</pose>\n    </include>\n"
  ALL_MODELS_XML+=$MODEL_XML
done
# Loop for tails
for i in $(seq 1 $NUM_TAILS); do
  DRONE_ID=$((DRONE_ID + 1))
  create_model "$TAIL_MODEL_PATH" $DRONE_ID
  MODEL_XML="    <include>\n      <uri>model://swan_k1_hwing_${DRONE_ID}</uri>\n      <pose degrees=\"true\">$(( (i-1) * 2 )) $(( 5 + (i-1) * 2 )) 0.75 0 -90 0</pose>\n    </include>\n"
  ALL_MODELS_XML+=$MODEL_XML
done
# Insert the models right before the closing </world> tag
sed -i -e "/<\/world>/i\\
${ALL_MODELS_XML}" "$OUTPUT_FILE"
