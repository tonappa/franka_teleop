# Change ROS version in build.sh, not here!
ARG BASE_IMAGE=osrf/ros
ARG BASE_TAG=noetic-desktop

FROM ${BASE_IMAGE}:${BASE_TAG}

ARG ROS_NUMBER=1

# ================================== Nvidia ================================== #

# Use the NVIDIA graphics card.
ENV NVARCH=x86_64

ENV NVIDIA_REQUIRE_CUDA="cuda>=12.1 brand=tesla,driver>=450,driver<451 brand=tesla,driver>=470,driver<471 brand=unknown,driver>=470,driver<471 brand=nvidia,driver>=470,driver<471 brand=nvidiartx,driver>=470,driver<471 brand=geforce,driver>=470,driver<471 brand=geforcertx,driver>=470,driver<471 brand=quadro,driver>=470,driver<471 brand=quadrortx,driver>=470,driver<471 brand=titan,driver>=470,driver<471 brand=titanrtx,driver>=470,driver<471 brand=tesla,driver>=510,driver<511 brand=unknown,driver>=510,driver<511 brand=nvidia,driver>=510,driver<511 brand=nvidiartx,driver>=510,driver<511 brand=geforce,driver>=510,driver<511 brand=geforcertx,driver>=510,driver<511 brand=quadro,driver>=510,driver<511 brand=quadrortx,driver>=510,driver<511 brand=titan,driver>=510,driver<511 brand=titanrtx,driver>=510,driver<511 brand=tesla,driver>=515,driver<516 brand=unknown,driver>=515,driver<516 brand=nvidia,driver>=515,driver<516 brand=nvidiartx,driver>=515,driver<516 brand=geforce,driver>=515,driver<516 brand=geforcertx,driver>=515,driver<516 brand=quadro,driver>=515,driver<516 brand=quadrortx,driver>=515,driver<516 brand=titan,driver>=515,driver<516 brand=titanrtx,driver>=515,driver<516 brand=tesla,driver>=525,driver<526 brand=unknown,driver>=525,driver<526 brand=nvidia,driver>=525,driver<526 brand=nvidiartx,driver>=525,driver<526 brand=geforce,driver>=525,driver<526 brand=geforcertx,driver>=525,driver<526 brand=quadro,driver>=525,driver<526 brand=quadrortx,driver>=525,driver<526 brand=titan,driver>=525,driver<526 brand=titanrtx,driver>=525,driver<526"
ENV NV_CUDA_CUDART_VERSION=12.1.105-1
ENV NV_CUDA_COMPAT_PACKAGE=cuda-compat-12-1

LABEL maintainer="NVIDIA CORPORATION <cudatools@nvidia.com>"

RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg2 curl ca-certificates && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/${NVARCH}/3bf863cc.pub | apt-key add - && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/${NVARCH} /" > /etc/apt/sources.list.d/cuda.list && \
    rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION=12.1.1

# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-cudart-12-1=${NV_CUDA_CUDART_VERSION} \
    ${NV_CUDA_COMPAT_PACKAGE} \
    && rm -rf /var/lib/apt/lists/*

# Required for nvidia-docker v1
RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf \
    && echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH=/usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility

# ============================================================================ #

# Prevent bash to ask for user input which may break the building process
ENV DEBIAN_FRONTEND=noninteractive

# Install sudo, and essential packages + add ROS repository and key
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt --mount=type=cache,sharing=locked,target=/var/lib/apt \
    apt-get update && apt-get install --no-install-recommends -qqy \
    bash-completion \
    build-essential \
    pip \
    git \
    gedit \
    sudo \
    python3-pip \
    wget \
    curl \
    python3-tk \
    software-properties-common \
    ca-certificates \
    gpg-agent

# Add kisak-mesa PPA for better Mesa/Integrated GPU support
RUN add-apt-repository -y ppa:kisak/kisak-mesa && \
    apt-get update && apt-get dist-upgrade -y && \
    apt-get install -y --no-install-recommends \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    mesa-utils && \
    rm -rf /var/lib/apt/lists/*

# pip install (core ML/vision packages)
# Pin mediapipe to 0.10.9 — newer versions (>=0.10.11) pull in jax/jaxlib
# which are unavailable for Python 3.8 on x86_64 and add 800+ MB of torch.
RUN pip3 install --default-timeout=1000 --no-cache-dir \
    opencv-python \
    pandas \
    "mediapipe==0.10.9" \
    OneEuroFilter \
    numpy \
    sympy

# Install ROS-specific packages (depending on ROS_NUMBER)
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt --mount=type=cache,sharing=locked,target=/var/lib/apt \
    if [ "${ROS_NUMBER}" = "1" ] ; then \
        apt-get update && apt-get install --no-install-recommends -qqy \
        ros-$ROS_DISTRO-catkin \
        python3-catkin-tools \
        python3-osrf-pycommon \
        ros-$ROS_DISTRO-franka-ros \
        ros-$ROS_DISTRO-moveit \
        ros-$ROS_DISTRO-moveit-servo \
        ros-$ROS_DISTRO-gazebo-ros \
        ros-$ROS_DISTRO-gazebo-ros-control \
        ros-$ROS_DISTRO-gazebo-ros-pkgs \
        ros-$ROS_DISTRO-gazebo-plugins \
        ros-$ROS_DISTRO-tf2-geometry-msgs \
        ros-$ROS_DISTRO-realsense2-camera \
        ros-$ROS_DISTRO-realsense2-description ; \
    fi

# Audio / speech-recognition dependencies (PulseAudio + Vosk)
RUN apt-get update && apt-get install -y --no-install-recommends \
    pulseaudio-utils \
    unzip \
    xterm \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --default-timeout=1000 --no-cache-dir vosk

# Download Vosk speech models (English + Italian)
RUN mkdir -p /opt/vosk-models && cd /opt/vosk-models \
    && wget -q https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip \
    && unzip vosk-model-small-en-us-0.15.zip \
    && mv vosk-model-small-en-us-0.15 vosk-model-small-en-us \
    && rm vosk-model-small-en-us-0.15.zip \
    && wget -q https://alphacephei.com/vosk/models/vosk-model-small-it-0.22.zip \
    && unzip vosk-model-small-it-0.22.zip \
    && mv vosk-model-small-it-0.22 vosk-model-small-it \
    && rm vosk-model-small-it-0.22.zip

# PlotJuggler for real-time ROS topic visualization
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-plotjuggler-ros \
    && rm -rf /var/lib/apt/lists/*

ARG MYUID=1000
ARG MYGID=1000
ARG USER=ros
ARG PWDR=/

RUN OLD_USER=$(awk -v uid="$MYUID" -F ':' '$3==uid {print $1}' /etc/passwd) \
    && if [ -z "${OLD_USER}" ]; then \
        # There is no existing UID equal to MYUID.
        # Create the same user as the host itself. (By default Docker creates the container as root, which is not recommended.)
        addgroup --gid ${MYGID} ${USER} \
        && adduser --gecos "ROS User" --disabled-password --uid ${MYUID} --gid ${MYGID} ${USER} \
        && usermod -a -G dialout ${USER} \
        && echo ${USER}" ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/99_aptget \
        && chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget ; \
    else \
        usermod -l ${USER} ${OLD_USER} \
        && sudo usermod -d /home/${USER} -m ${USER} \
        && echo ${USER}" ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/99_aptget \
        && chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget ; \
    fi

# Set environment variables for the user
ENV USER=${USER}
USER ${USER}
ENV HOME=/home/${USER}

# Install the required dependencies with rosdep.
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt --mount=type=cache,sharing=locked,target=/var/lib/apt \
    --mount=type=bind,source=./src,target=${PWDR}/src,rw \
    cd ${PWDR} \
    && sudo apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y

# Set up environment
COPY ./docker/.config/update_bashrc_ros_1 /sbin/update_bashrc_ros_1
COPY ./docker/.config/update_bashrc_ros_2 /sbin/update_bashrc_ros_2

RUN if [ "${ROS_NUMBER}" = "1" ] ; then \
        sudo chmod +x /sbin/update_bashrc_ros_1 ; sudo chown ${USER} /sbin/update_bashrc_ros_1 \
        && echo 'echo "source '${PWDR}'/devel/setup.bash" >> ~/.bashrc' >> /sbin/update_bashrc_ros_1 \
        && cat /sbin/update_bashrc_ros_1 \
        && sync ; /bin/bash -c /sbin/update_bashrc_ros_1 ; sudo rm /sbin/update_bashrc_ros_1 ; \
    elif [ "${ROS_NUMBER}" = "2" ] ; then \
        sudo chmod +x /sbin/update_bashrc_ros_2 ; sudo chown ${USER} /sbin/update_bashrc_ros_2 \
        && echo 'echo "source '${PWDR}'/install/setup.bash" >> ~/.bashrc' >> /sbin/update_bashrc_ros_2 \
        && cat /sbin/update_bashrc_ros_2 \
        && sync ; /bin/bash -c /sbin/update_bashrc_ros_2 ; sudo rm /sbin/update_bashrc_ros_2 ; \
    fi

# Change entrypoint to source ~/.bashrc and start in ~
COPY ./docker/.config/entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh ; sudo chown ${USER} /ros_entrypoint.sh \
 && echo "cd "${PWDR} >> /ros_entrypoint.sh \
 && echo 'exec bash -i -c $@' >> /ros_entrypoint.sh \
 && cat /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
