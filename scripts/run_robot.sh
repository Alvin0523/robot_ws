#!/bin/bash

# --- Configuration ---
# The image you built (change this if you used a different name)
IMAGE_NAME="htx_robot:latest"
# Your container name
CONTAINER_NAME="ros_humble_robot"
# The path on your Jetson
HOST_WS="$HOME/workspaces/robot_ws"
# The path inside the container (MUST match your Dockerfile ENTRYPOINT)
INNER_WS="/home/agx/workspaces/robot_ws"

if [ "$(docker ps -q -f status=running -f name=$CONTAINER_NAME)" ]; then
    echo "✅ Container '$CONTAINER_NAME' is ALREADY running."
    echo "🔗 Attaching to existing container..."
    # 'exec' lets you open a new terminal inside the existing robot
    docker exec -it $CONTAINER_NAME bash
    exit 0
fi

# --- 3. Check: Is Container Stopped/Exited? ---
# Ref: Based on Isaac ROS run_dev.sh lines 139-141
if [ "$(docker ps -aq -f status=exited -f name=$CONTAINER_NAME)" ]; then
    echo "🧹 Removing stopped container..."
    docker rm $CONTAINER_NAME > /dev/null
fi

# --- Run Command ---
docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --network host \
    --privileged \
    --ipc=host \
    --runtime nvidia \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.Xauthority:/root/.Xauthority:rw \
    -v "$HOST_WS":"$INNER_WS" \
    "$IMAGE_NAME"