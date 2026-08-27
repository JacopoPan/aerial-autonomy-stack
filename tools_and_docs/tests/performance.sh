#!/bin/bash

# Adjust the configuration below as needed, then run with:
# $ cd aerial-autonomy-stack/tools_and_docs/
# $ conda activate aas
# $ ./tests/performance.sh

# (optional) CPU/GPU performance modes:
# sudo cpupower frequency-set -g performance                      # Force CPU performance mode
#   cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor     # Check (does NOT persist across reboots)
# sudo prime-select nvidia                                        # Force GPU use instead of on-demand
#   prime-select query                                            # Check (persists across reboots)
# sudo nvidia-smi -pm 1                                           # Prevent NVIDIA driver from going idle
#   nvidia-smi --query-gpu=persistence_mode --format=csv,noheader # Check (does NOT persist across reboots)

# Configuration (see also quad_counts below)
MODES=("speedup" "vectorenv-speedup")
AUTOPILOTS=("px4" "ardupilot")
SENSOR_SCENARIOS=("both" "no_camera" "no_lidar" "none")
REPETITIONS=1
MAX_RETRIES=3

# Check for conda environment
if [[ "$CONDA_DEFAULT_ENV" != "aas" ]]; then
    echo "Error: The 'aas' conda environment is not active."
    echo "Please activate it with: conda activate aas"
    exit 1
fi

# Find the script's path (and then gym_run.py script in tools_and_docs/)
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
GYM_RUN_SCRIPT="$SCRIPT_DIR/../gym_run.py"

# Docker clean-up helper function
cleanup_docker() {
    docker ps -q | xargs -r docker stop >/dev/null 2>&1
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

                            output=$(python3 "$GYM_RUN_SCRIPT" \
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
                                    speedup_values+=("$val")
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

# Performance results from 2026-08-26 on commit f660f05a53a013513fe32e45b52cc379ece70e80
# System: Lenovo ThinkPad P16 Gen 2 on Ubuntu 24.04.04 with 64GB RAM, Intel Core i9-13980HX x 32, NVIDIA RTX 3500 Ada Generation Laptop GPU
# Kernel: Linux 7.0.0-30-generic; NVIDIA Driver: 610.43.02; CUDA Version: 13.3
#
# Running: speedup | px4 | 1 quads | both sensors
# Avg Speedup:        7.49x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 49s
# Running: speedup | px4 | 1 quads | no camera
# Avg Speedup:        7.90x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 105s
# Running: speedup | px4 | 1 quads | no lidar
# Avg Speedup:        7.72x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 159s
# Running: speedup | px4 | 1 quads | neither sensor
# Avg Speedup:        8.21x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 210s
# Running: speedup | ardupilot | 1 quads | both sensors
# Avg Speedup:        5.43x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 284s
# Running: speedup | ardupilot | 1 quads | no camera
# Avg Speedup:        6.00x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 353s
# Running: speedup | ardupilot | 1 quads | no lidar
# Avg Speedup:        5.34x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 427s
# Running: speedup | ardupilot | 1 quads | neither sensor
# Avg Speedup:        6.50x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 490s
# Running: vectorenv-speedup | px4 | 1 quads | both sensors
# Avg Speedup:        14.90x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 549s
# Running: vectorenv-speedup | px4 | 1 quads | no camera
# Avg Speedup:        17.60x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 604s
# Running: vectorenv-speedup | px4 | 1 quads | no lidar
# Avg Speedup:        15.98x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 661s
# Running: vectorenv-speedup | px4 | 1 quads | neither sensor
# Avg Speedup:        20.87x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 706s
# Running: vectorenv-speedup | ardupilot | 1 quads | both sensors
# Avg Speedup:        10.99x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 792s
# Running: vectorenv-speedup | ardupilot | 1 quads | no camera
# Avg Speedup:        13.24x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 866s
# Running: vectorenv-speedup | ardupilot | 1 quads | no lidar
# Avg Speedup:        11.80x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 946s
# Running: vectorenv-speedup | ardupilot | 1 quads | neither sensor
# Avg Speedup:        11.68x ± 0.00x wall-clock (Avg of 1 runs)
# Elapsed Time: 1023s
