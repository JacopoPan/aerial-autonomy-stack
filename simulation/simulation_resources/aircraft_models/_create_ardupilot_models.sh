#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

if [ "$#" -ne 3 ]; then
  echo "Usage: $0 <num_quads> <num_vtols> <camera_pitch>"
  echo "Example: ./_create_ardupilot_models.sh 2 1 -1.5707"
  exit 1
fi

NUM_QUADS=$1
NUM_VTOLS=$2
CAMERA_PITCH=$3

BASE_PORT=9002

# Paths to the base models
QUAD_MODEL_PATH="/aas/simulation_resources/aircraft_models/iris_with_ardupilot"
VTOL_MODEL_PATH="/aas/simulation_resources/aircraft_models/alti_transition_quad"

# Check if model directories exist
if [ ! -d "$QUAD_MODEL_PATH" ] && [ "$NUM_QUADS" -gt 0 ]; then
    echo "Error: Quad model directory '${QUAD_MODEL_PATH}' not found."
    exit 1
fi

if [ ! -d "$VTOL_MODEL_PATH" ] && [ "$NUM_VTOLS" -gt 0 ]; then
    echo "Error: VTOL model directory '${VTOL_MODEL_PATH}' not found."
    exit 1
fi

echo "Creating ${NUM_QUADS} quadcopter(s) and ${NUM_VTOLS} VTOL(s)..."

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
    sed -i "s|<topic>/${BASE_MODEL_NAME}|<topic>/${NEW_MODEL_NAME}|g" "${NEW_MODEL_DIR}/model.sdf"
    sed -i "s/<fdm_port_in>${BASE_PORT}<\/fdm_port_in>/<fdm_port_in>$(($BASE_PORT + ($DRONE_ID - 1) * 10))<\/fdm_port_in>/g" "${NEW_MODEL_DIR}/model.sdf"
    # Set camera pitch
    sed -i "s|<!--CAMERA_PITCH_PLACEHOLDER-->|${CAMERA_PITCH}|g" "${NEW_MODEL_DIR}/model.sdf"

    ### Payload plugin adaptations
    # Update topic tags: <topic>/base_model_name/... -> <topic>/new_model_name/...
    sed -i "s|<topic>/${BASE_MODEL_NAME}/|<topic>/${NEW_MODEL_NAME}/|g" "${NEW_MODEL_DIR}/model.sdf"
    sed -i "s|topic=\"/${BASE_MODEL_NAME}/|topic=\"/${NEW_MODEL_NAME}/|g" "${NEW_MODEL_DIR}/model.sdf"
    sed -i "s|<cmd_topic>/${BASE_MODEL_NAME}/|<cmd_topic>/${NEW_MODEL_NAME}/|g" "${NEW_MODEL_DIR}/model.sdf"
    
    # Specific replacement for the particle emitter topic path to support multiple drones
    # Matches /model/iris_with_ardupilot/model/... -> /model/iris_with_ardupilot_N/model/...
    sed -i "s|/model/${BASE_MODEL_NAME}/model/|/model/${NEW_MODEL_NAME}/model/|g" "${NEW_MODEL_DIR}/model.sdf"

    DEST_PARAMS="${NEW_MODEL_DIR}/ardupilot-4.6.params"
    cp "${BASE_MODEL_PATH}/ardupilot-4.6.params" "$DEST_PARAMS"
}

# Counter for unique port and model IDs
DRONE_ID=0

# Loop for quads
for i in $(seq 1 $NUM_QUADS); do
    DRONE_ID=$((DRONE_ID + 1))
    create_model "$QUAD_MODEL_PATH" "$DRONE_ID"
done

# Loop for VTOLs
for i in $(seq 1 $NUM_VTOLS); do
    DRONE_ID=$((DRONE_ID + 1))
    create_model "$VTOL_MODEL_PATH" "$DRONE_ID"
done

echo "Done."
