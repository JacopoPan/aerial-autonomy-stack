#!/bin/bash

# This script generates the SDF files for multiple Betaflight quads and a world SDF file containing them

# Exit immediately if a command exits with a non-zero status
set -e

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <num_quads> <absolute_path_to_empty_world>"
  echo "Example: ./create_betaflight_drones_and_world.sh 3 /aas/simulation_resources/simulation_worlds/impalpable_greyness.sdf"
  exit 1
fi

NUM_QUADS=$1
BASE_WORLD_WITH_PATH=$2
QUAD_MODEL_PATH="/aas/simulation_resources/aircraft_models/iris_with_betaflight"
OUTPUT_FILE="$(dirname "$BASE_WORLD_WITH_PATH")/populated_betaflight.sdf"

echo "Creating ${NUM_QUADS} Betaflight quadcopter(s)..."
cp "$BASE_WORLD_WITH_PATH" "$OUTPUT_FILE"
for DRONE_ID in $(seq 1 "$NUM_QUADS"); do
  # The BetaflightPlugin ports (9002/9003) are offset like those of each SITL (betaflight_SITL.elf --port-offset in simulation.yml.erb)
  sed "s|<portOffset>0<|<portOffset>$(( 10 * (DRONE_ID - 1) ))<|" "${QUAD_MODEL_PATH}/model.sdf" > "${QUAD_MODEL_PATH}/model_${DRONE_ID}.sdf"
  # Insert the model right before the closing </world> tag, at the same positions as the PX4 and ArduPilot quads
  sed -i "/<\/world>/i <include><uri>model://iris_with_betaflight/model_${DRONE_ID}.sdf</uri><name>iris_with_betaflight_${DRONE_ID}</name><pose>$(( 2 * (DRONE_ID - 1) )) $(( 2 * (DRONE_ID - 1) )) 0.5 0 0 0</pose></include>" "$OUTPUT_FILE"
done
