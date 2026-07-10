#!/bin/bash

# This scrip allows to analyze ArduPilot logs using MAVExplorer and PX4 logs with flight_review
# Note that the 3D and map features of flight_review require to add API keys to _github_clones/flight_review/app/config_default.ini

NUM_DRONES=$((NUM_QUADS + NUM_VTOLS + NUM_TAILS))
if [[ -n "$NUM_DRONES" && "$NUM_DRONES" =~ ^[0-9]+$ ]]; then
    
    if [ "$AUTOPILOT" == "ardupilot" ]; then
        for i in $(seq 1 $NUM_DRONES); do
            LOGS_BASE_DIR="/aas/ardu_sitl_${i}/logs"
            LASTLOG_PATH="${LOGS_BASE_DIR}/LASTLOG.TXT"
            log_index=$(cat "$LASTLOG_PATH" | tr -d '\r') # Remove potential carriage return characters
            filename=$(printf "%08d.BIN" "$log_index") # Create filenames like 00000001.BIN, 00000002.BIN, etc.
            logfile="${LOGS_BASE_DIR}/$filename"
            if [ -f "$logfile" ]; then
                echo "Opening log for drone $i: $logfile"
                MAVExplorer.py "$logfile"  &
            else
                echo "Log file not found for drone $i: $logfile"
            fi
        done

    elif [ "$AUTOPILOT" == "px4" ]; then
        # Get the container's IP address on aas-sim-network and extract the last octet
        SIM_SUBNET_IP=$(hostname -I | grep -oE "${SIM_SUBNET}\.[0-9]+\.[0-9]+" | head -n 1)
        SIM_ID=$(echo "$SIM_SUBNET_IP" | awk -F'.' '{print $NF}')

        cd /aas/github_apps/flight_review/ || exit 1
        /px4fr-env/bin/python3 ./app/setup_db.py
        /px4fr-env/bin/python3 ./app/serve.py --allow-websocket-origin=${SIM_SUBNET}.90.${SIM_ID}:5006 2>/dev/null & # Starting flight_review (suppress "Address already in use" when running this script more than once)
        sleep 2
        for i in $(seq 0 $((NUM_DRONES - 1))); do
            log_dir="/aas/github_apps/PX4-Autopilot/build/px4_sitl_default/rootfs/$i/log"
            # Find the latest dated folder and then the latest .ulg file inside it
            latest_date_dir=$(ls -td "$log_dir"/* | head -n 1)
            if [ -d "$latest_date_dir" ]; then
                latest_ulg=$(ls -t "$latest_date_dir"/*.ulg | head -n 1)
                if [ -n "$latest_ulg" ]; then
                    echo "Latest .ulg log for PX4 drone $i is: $(basename "$latest_ulg")"
                    python3 /aas/github_apps/PX4-Autopilot/Tools/upload_log.py --quiet --server=http://${SIM_SUBNET}.90.${SIM_ID}:5006 "$latest_ulg"
                else
                    echo "No .ulg logs found for PX4 drone $i in: $latest_date_dir"
                fi
            else
                echo "No log directory found for PX4 drone $i at: $log_dir"
            fi
        done
        echo ""
        echo "You can view the imported logs at: http://${SIM_SUBNET}.90.${SIM_ID}:5006/browse"

    else
        echo "Unknown AUTOPILOT"
        exit 0
    fi

else
    echo "Error: NUM_DRONES environment variable is not set or is not a valid number."
    exit 1
fi