#!/bin/bash

# Create /tmp/.docker.xauth if it does not already exist.
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
fi

# TODO: Change image name if you wish (here and in build.sh).
IMAGE_NAME=franka_teleop
xhost +
docker run \
    --privileged \
    --net=host \
    -it \
    --rm \
    --gpus all \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="/dev/dri:/dev/dri" \
    --device=/dev/video0 \
    --device=/dev/video1 \
    --device=/dev/bus/usb \
    -v ${PWD}:${PWD} \
    --env="HISTFILE=/home/.bash_history" \
    --env="HISTFILESIZE=$HISTFILESIZE" \
    -v ~/.bash_history:/home/.bash_history \
    --device /dev/snd \
    -e PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native \
    -v ${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native \
    --group-add $(getent group audio | cut -d: -f3) \
    -v /dev/shm:/dev/shm \
    -v /home/$USER/.vscode:/home/$USER/.vscode \
    -v /home/$USER/.vscode-server:/home/$USER/.vscode-server \
    -v /home/$USER/.config/Code:/home/$USER/.config/Code \
    --env="MPLCONFIGDIR=/home/$USER/.matplotlib" \
    --env="XDG_RUNTIME_DIR=/tmp/runtime-$USER" \
    --group-add $(getent group audio | cut -d: -f3) \
    --group-add $(getent group video | cut -d: -f3) \
    ${IMAGE_NAME} \
    bash


