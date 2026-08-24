#!/bin/bash

# Use with:
# $ cd aerial-autonomy-stack/tools_and_docs/
# $ ./tests/check_requirements.sh

HELP_URL="https://github.com/JacopoPan/aerial-autonomy-stack/blob/main/tools_and_docs/docs/REQUIREMENTS_UBUNTU.md"
if grep -q "Microsoft" /proc/version || grep -q "WSL" /proc/version; then
    echo "[INFO] WSL environment detected: some checks (NVIDIA Driver/CTK) may behave differently; open a GitHub issue, if necessary"
    HELP_URL="https://github.com/JacopoPan/aerial-autonomy-stack/blob/main/tools_and_docs/docs/REQUIREMENTS_WSL.md"
fi

if [ -f /etc/os-release ]; then
    source /etc/os-release
    MAJOR_VER="${VERSION_ID%%.*}"
    if [[ "$ID" == "ubuntu" && "$MAJOR_VER" -ge 24 ]]; then
        echo "[PASS] Host OS: tested with AAS (version: $PRETTY_NAME)"
    else
        echo "[WARN] Host OS: not tested with AAS (version: $PRETTY_NAME; recommended: Ubuntu 24.04 or newer)"
    fi
else
    echo "[WARN] Host OS: unknown (cannot find /etc/os-release)"
fi

if command -v nvidia-smi &> /dev/null; then
    DRIVER_VER=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader)
    DRIVER_MAJOR=${DRIVER_VER%%.*}
    if [ -f /etc/nv_tegra_release ]; then
        EXPECTED=595 # Version that ships with JetPack 7.2 for Jetson Orin
    else
        EXPECTED=610 # Version tested on amd64
    fi
    if [ "$DRIVER_MAJOR" -eq "$EXPECTED" ]; then
        echo "[PASS] NVIDIA Driver: tested with AAS (version: $DRIVER_VER)"
    else
        echo "[WARN] NVIDIA Driver: available but not tested with AAS (version: $DRIVER_VER; recommended: $EXPECTED)"
    fi
else
    echo "[FAIL] NVIDIA Driver: not found"
    echo "    Instructions: $HELP_URL"
fi

if docker info &> /dev/null; then
    DOCKER_VER=$(docker --version | awk '{print $3}' | tr -d ',')
    DOCKER_MAJOR=${DOCKER_VER%%.*}

    if [ "$DOCKER_MAJOR" -ge 29 ]; then
        echo "[PASS] Docker Engine: tested with AAS (version: $DOCKER_VER)"
    else
        echo "[WARN] Docker Engine: available but not tested with AAS (version: $DOCKER_VER; recommended: 29 or newer)"
    fi
else
    echo "[FAIL] Docker Engine: not installed or User not in docker group for sudo-less use"
    echo "    Instructions: $HELP_URL"
fi

if docker info 2>/dev/null | grep -i "runtimes.*nvidia" &> /dev/null; then
    CTK_VER=$(nvidia-ctk --version 2>/dev/null | awk '{print $6}')
    if [ "$(printf '%s\n' "1.20" "$CTK_VER" | sort -V | head -n1)" = "1.20" ]; then
        echo "[PASS] NVIDIA Container Toolkit: tested with AAS (version: $CTK_VER)"
    else
        echo "[WARN] NVIDIA Container Toolkit: available but not tested with AAS (version: $CTK_VER; recommended: 1.20 or newer)"
    fi
else
    echo "[FAIL] NVIDIA Container Toolkit: not detected"
    echo "    Instructions: $HELP_URL"
fi
