#!/bin/bash

# Create /tmp/.docker.xauth if it does not already exist.
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
fi

IMAGE_NAME=franka_teleop
xhost +

# PulseAudio socket for microphone passthrough
PULSE_SOCKET="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/pulse/native"
PULSE_COOKIE="$HOME/.config/pulse/cookie"

# Base docker run command
DOCKER_CMD="docker run \
    --privileged \
    --net=host \
    -it \
    --rm \
    --env='DISPLAY=$DISPLAY' \
    --env='QT_X11_NO_MITSHM=1' \
    --volume='/tmp/.X11-unix:/tmp/.X11-unix:rw' \
    --env='XAUTHORITY=$XAUTH' \
    --volume='$XAUTH:$XAUTH' \
    --volume='/dev/dri:/dev/dri' \
    --device=/dev/video0 \
    --device=/dev/video1 \
    --device=/dev/bus/usb \
    -v ${PWD}:${PWD} \
    --env='HISTFILE=/home/.bash_history' \
    --env='HISTFILESIZE=$HISTFILESIZE' \
    -v ~/.bash_history:/home/.bash_history \
    --device /dev/snd \
    -v /dev/shm:/dev/shm \
    -v /home/$USER/.vscode:/home/$USER/.vscode \
    -v /home/$USER/.vscode-server:/home/$USER/.vscode-server \
    -v /home/$USER/.config/Code:/home/$USER/.config/Code \
    --env='MPLCONFIGDIR=/home/$USER/.matplotlib' \
    --env='XDG_RUNTIME_DIR=/tmp/runtime-$USER'"

# PulseAudio passthrough (for microphone access)
if [ -S "$PULSE_SOCKET" ]; then
    DOCKER_CMD="$DOCKER_CMD \
    --env=PULSE_SERVER=unix:/tmp/pulseaudio.socket \
    --volume=$PULSE_SOCKET:/tmp/pulseaudio.socket"
    if [ -f "$PULSE_COOKIE" ]; then
        DOCKER_CMD="$DOCKER_CMD \
        --volume=$PULSE_COOKIE:/home/$USER/.config/pulse/cookie"
    fi
else
    echo "Warning: PulseAudio socket not found at $PULSE_SOCKET — microphone may not work."
fi

# Interactive GPU Configuration Selection
while true; do
    echo "----------------------------------------"
    echo "Select GPU Configuration:"
    echo "  1) NVIDIA (GPU acceleration)"
    echo "  2) Integrated (Intel/AMD or CPU fallback)"
    echo "----------------------------------------"
    read -p "Enter choice [1 or 2]: " gpu_choice

    case "$gpu_choice" in
        1)
            echo "Selected: NVIDIA GPU Mode"
            DOCKER_CMD="$DOCKER_CMD \
            --gpus all \
            --env='NVIDIA_DRIVER_CAPABILITIES=all' \
            --env='NVIDIA_VISIBLE_DEVICES=all'"
            break
            ;;
        2)
            echo "Selected: Integrated/Standard Mode"
            if [ -d "/dev/dri" ]; then
                DOCKER_CMD="$DOCKER_CMD --device=/dev/dri:/dev/dri"
            else
                echo "Warning: /dev/dri not found. Falling back to software rendering."
            fi
            break
            ;;
        *)
            echo "Invalid selection. Please type 1 or 2."
            ;;
    esac
done

# Add audio/video group access
if getent group video > /dev/null; then
    DOCKER_CMD="$DOCKER_CMD --group-add video"
fi
if getent group audio > /dev/null; then
    DOCKER_CMD="$DOCKER_CMD --group-add audio"
fi

# Execute
echo "Launching container..."
eval $DOCKER_CMD ${IMAGE_NAME} bash
