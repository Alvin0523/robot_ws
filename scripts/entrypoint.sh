#!/bin/bash
set -e

# 1. Source Global ROS (The Underlay)
source "/opt/ros/humble/setup.bash"

# 2. Source Local Workspace (The Overlay) - ONLY if built
if [ -d "install" ]; then
    source install/setup.bash
    echo "Sourced local workspace."
fi

# 3. Execute command
exec "$@"