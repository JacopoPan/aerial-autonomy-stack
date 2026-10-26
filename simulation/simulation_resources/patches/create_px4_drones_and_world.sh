#!/bin/bash

# This script generates a world SDF file containing multiple PX4 vehicles, for each PX4 SITL to attach to (PX4_GZ_MODEL_NAME in simulation.yml.erb)

# Exit immediately if a command exits with a non-zero status
set -e

if [ "$#" -ne 4 ]; then
  echo "Usage: $0 <num_quads> <num_vtols> <num_tails> <absolute_path_to_empty_world>"
  echo "Example: ./create_px4_drones_and_world.sh 3 2 1 /aas/simulation_resources/simulation_worlds/impalpable_greyness.sdf"
  exit 1
fi

OUTPUT_FILE="$(dirname "$4")/populated_px4.sdf"
cp "$4" "$OUTPUT_FILE"
INSTANCE=0 # px4 -i is 0-based

# Insert <count> models (named <model>_<instance>) right before the closing </world> tag, with poses x = 2i, y = 2i + <y_offset>
add_models() { # <model> <count> <y_offset> <z>
  for i in $(seq 0 $(( $2 - 1 ))); do
    sed -i "/<\/world>/i <include><uri>model://$1</uri><name>$1_${INSTANCE}</name><pose>$(( 2 * i )) $(( 2 * i + $3 )) $4 0 0 0</pose></include>" "$OUTPUT_FILE"
    INSTANCE=$(( INSTANCE + 1 ))
  done
}

echo "Creating $1 quadcopter(s), $2 VTOL(s), and $3 tailsitter(s)..."
add_models x500 "$1" 0 0.5
add_models standard_vtol "$2" 2 0.5
add_models quadtailsitter "$3" 4 0.25
