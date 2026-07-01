#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

if [ "$#" -ne 3 ]; then
  echo "Usage: $0 <num_quads> <num_vtols> <num_tails>"
  echo "Example: ./_create_ardupilot_models.sh 3 2 1"
  exit 1
fi

NUM_QUADS=$1
NUM_VTOLS=$2
NUM_TAILS=$3

BASE_PORT=9002

# Paths to the base models
QUAD_MODEL_PATH="/aas/simulation_resources/aircraft_models/iris_with_ardupilot"
VTOL_MODEL_PATH="/aas/simulation_resources/aircraft_models/alti_transition_quad"
TAIL_MODEL_PATH="/aas/simulation_resources/aircraft_models/swan_k1_hwing"

# Check if model directories exist
if [ ! -d "$QUAD_MODEL_PATH" ] && [ "$NUM_QUADS" -gt 0 ]; then
    echo "Error: Quad model directory '${QUAD_MODEL_PATH}' not found."
    exit 1
fi

if [ ! -d "$VTOL_MODEL_PATH" ] && [ "$NUM_VTOLS" -gt 0 ]; then
    echo "Error: VTOL model directory '${VTOL_MODEL_PATH}' not found."
    exit 1
fi

if [ ! -d "$TAIL_MODEL_PATH" ] && [ "$NUM_TAILS" -gt 0 ]; then
    echo "Error: Tail model directory '${TAIL_MODEL_PATH}' not found."
    exit 1
fi

echo "Creating ${NUM_QUADS} quadcopter(s), ${NUM_VTOLS} VTOL(s), and ${NUM_TAILS} tailsitters(s)..."

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
    sed -i "s/<fdm_port_in>${BASE_PORT}<\/fdm_port_in>/<fdm_port_in>$(($BASE_PORT + ($DRONE_ID - 1) * 10))<\/fdm_port_in>/g" "${NEW_MODEL_DIR}/model.sdf"

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

# Loop for tailsitters
for i in $(seq 1 $NUM_TAILS); do
    DRONE_ID=$((DRONE_ID + 1))
    create_model "$TAIL_MODEL_PATH" $DRONE_ID
done

echo "Done."
