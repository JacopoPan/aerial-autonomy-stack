#!/bin/bash

# Adjust the configuratoin below as needed, then run with:
# $ conda activate aas
# $ ./performance.sh

if [[ "$CONDA_DEFAULT_ENV" != "aas" ]]; then
    echo "Error: The 'aas' conda environment is not active."
    echo "Please activate it with: conda activate aas"
    exit 1
fi

# Configuration
MODES=("speedup" "vectorenv-speedup")
AUTOPILOTS=("px4" "ardupilot")
SENSOR_SCENARIOS=("both" "no_camera" "no_lidar" "none")
REPETITIONS=1
MAX_RETRIES=3

# Docker clean-up helper function
cleanup_docker() {
    # Check if there are running containers before trying to stop them
    if [ -n "$(docker ps -q)" ]; then
        docker stop $(docker ps -q) >/dev/null 2>&1
    fi
    docker container prune -f >/dev/null 2>&1
    docker network prune -f >/dev/null 2>&1
    # Wait to let the os release socket file handles
    sleep 3
}

suite_start_time=$(date +%s)
{
    for mode in "${MODES[@]}"; do

        # 1. Handle vehicle (quad) counts based on mode
        if [ "$mode" == "speedup" ]; then
            quad_counts="1" # quad_counts="1 2 4 6"
        else
            quad_counts="1" # quad_counts="1 2 3"
        fi

        for autopilot in "${AUTOPILOTS[@]}"; do
            for quads in $quad_counts; do
                for scenario in "${SENSOR_SCENARIOS[@]}"; do

                    # 2. Scenarios
                    case $scenario in
                        "both") sensor_flags="--camera --lidar"; desc="both sensors" ;;
                        "no_camera") sensor_flags="--no-camera --lidar"; desc="no camera" ;;
                        "no_lidar") sensor_flags="--camera --no-lidar"; desc="no lidar" ;;
                        "none") sensor_flags="--no-camera --no-lidar"; desc="neither sensor" ;;
                    esac
                    echo "Running: $mode | $autopilot | $quads quads | $desc"

                    # 3. Execution loop with retries
                    speedup_values=()
                    
                    for (( i=1; i<=REPETITIONS; i++ )); do
                        success=false
                        attempt=1
                        
                        while [ $attempt -le $MAX_RETRIES ]; do

                            output=$(python3 ../scripts/gym_run.py \
                                --mode "$mode" \
                                --autopilot "$autopilot" \
                                --num_quads "$quads" \
                                --repetitions 1 \
                                $sensor_flags 2>&1)

                            exit_code=$?

                            if [ $exit_code -eq 0 ]; then
                                # Case A: SUCCESS
                                # Parse the "Avg Speedup" from the output (expected format: "Avg Speedup:        99.99x wall-clock")
                                val=$(echo "$output" | grep "Avg Speedup:" | sed -E 's/.*: +([0-9.]+)x.*/\1/')
                                
                                if [ -n "$val" ]; then
                                    speedup_values+=($val)
                                    success=true
                                    break # Exit retry loop
                                fi
                            fi

                            # Case B: FAIL
                            # If we end up here, exit_code != 0 OR we failed to parse the value
                            echo ">> Run $i/$REPETITIONS failed (Attempt $attempt/$MAX_RETRIES). Cleaning up and retrying..."
                            cleanup_docker
                            attempt=$((attempt+1))
                        done

                        if [ "$success" = false ]; then
                            echo ">> CRITICAL: Failed run $i after $MAX_RETRIES attempts. Skipping rest of this scenario."
                            break 
                        fi
                    done

                    # 4. Calculate and print statistics
                    if [ ${#speedup_values[@]} -gt 0 ]; then
                        vals_string=$(IFS=,; echo "${speedup_values[*]}")
                        stats=$(python3 -c "
import numpy as np
vals = [$vals_string]
mean = np.mean(vals)
std = np.std(vals)
print(f'{mean:.2f} {std:.2f}')
                    ")
                        read avg_speedup std_speedup <<< "$stats"
                        echo "Avg Speedup:        ${avg_speedup}x ± ${std_speedup}x wall-clock (Avg of ${#speedup_values[@]} runs)"
                    else
                        echo "Avg Speedup:        FAILED (0 successful runs)"
                    fi

                    # 5. Elapsed time update
                    current_time=$(date +%s)
                    elapsed=$(( current_time - suite_start_time ))                    
                    echo "Elapsed Time: ${elapsed}s"

                    # 6. Cooldown between scenarios
                    cleanup_docker
                    sleep 5

                done
            done
        done
    done
} | grep --line-buffered -E "Running:|Avg Speedup:|Elapsed Time:|CRITICAL"

# Performance results from 2026-04-16 on commit e6caaf621ca1cbb271be6c72a6e0da5f6fb50c31
# System: Alienware Alienware x17 R1 on Ubuntu 22.04.05 with 16GB RAM, Intel Core i7-11800H @ 2.30GHz x 16, GeForce RTX 3060 Mobile
# NVIDIA Driver Version: 580.126.09; CUDA Version: 13.0
#
# Running: speedup | px4 | 1 quads | both sensors
# Avg Speedup:        7.62x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 69s
# Running: speedup | px4 | 1 quads | no camera
# Avg Speedup:        8.19x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 140s
# Running: speedup | px4 | 1 quads | no lidar
# Avg Speedup:        7.91x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 213s
# Running: speedup | px4 | 1 quads | neither sensor
# Avg Speedup:        8.68x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 281s
# Running: speedup | ardupilot | 1 quads | both sensors
# Avg Speedup:        6.10x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 367s
# Running: speedup | ardupilot | 1 quads | no camera
# Avg Speedup:        6.59x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 448s
# Running: speedup | ardupilot | 1 quads | no lidar
# Avg Speedup:        6.30x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 532s
# Running: speedup | ardupilot | 1 quads | neither sensor
# Avg Speedup:        7.19x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 608s
# Running: vectorenv-speedup | px4 | 1 quads | both sensors
# Avg Speedup:        11.30x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 693s
# Running: vectorenv-speedup | px4 | 1 quads | no camera
# Avg Speedup:        14.76x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 768s
# Running: vectorenv-speedup | px4 | 1 quads | no lidar
# Avg Speedup:        13.24x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 846s
# Running: vectorenv-speedup | px4 | 1 quads | neither sensor
# Avg Speedup:        19.52x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 912s
# Running: vectorenv-speedup | ardupilot | 1 quads | both sensors
# Avg Speedup:        8.36x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 1033s
# Running: vectorenv-speedup | ardupilot | 1 quads | no camera
# Avg Speedup:        9.86x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 1139s
# Running: vectorenv-speedup | ardupilot | 1 quads | no lidar
# Avg Speedup:        9.34x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 1250s
# Running: vectorenv-speedup | ardupilot | 1 quads | neither sensor
# Avg Speedup:        14.60x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 1335s
