#!/bin/bash

# Detect Operating System and runs the approptiate docker compose file
OS_TYPE="$(uname -s)"
IS_WSL=$(grep -i Microsoft /proc/version 2>/dev/null)

echo "--- S-W-A-R-M Simulation Launcher ---"

if [ -n "$IS_WSL" ]; then
    echo "Detected: Windows (WSL2)"
    docker compose -f docker-compose.yml -f docker-compose.windows.yml up -d
elif [ "$OS_TYPE" == "Darwin" ]; then
    echo "Detected: macOS"
    # Ensure XQuartz is ready
    if ! command -v xhost &> /dev/null; then
        echo "Error: XQuartz (xhost) not found. Please install it."
        exit 1
    fi
    xhost +localhost
    docker compose -f docker-compose.yml -f docker-compose.mac.yml up -d
else
    echo "Detected: Native Linux"
    xhost +local:docker
    docker compose up -d
fi

echo "Environment is starting..."
echo "To enter the container, run: docker exec -it swarm_sim_container bash"