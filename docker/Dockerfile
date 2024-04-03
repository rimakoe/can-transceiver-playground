FROM ros:iron

RUN apt update && apt install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-vcstool \
    libpython3-dev \
    python3-pip \
    tmux \
    btop \
    net-tools \
    && python3 -m pip install --no-cache-dir setuptools==58.2.0 \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip

ENV SHELL /bin/bash
ENV GZ_VERSION harmonic 


ARG _USERNAME=user
ENV USERNAME=${_USERNAME}

ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $_USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $_USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo wget \
    && echo $_USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$_USERNAME \
    && chmod 0440 /etc/sudoers.d/$_USERNAME \
    && rm -rf /var/lib/apt/lists/*

USER $_USERNAME

RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc \
    && echo "source /home/user/can-transceiver-playground/ros2_ws/install/local_setup.bash" >> ~/.bashrc