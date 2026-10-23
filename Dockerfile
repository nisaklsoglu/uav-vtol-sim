FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV GZ_VERSION=harmonic

# 1. Temel kütüphaneler, sudo ve derleme araçları
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    git \
    python3-pip \
    lsb-release \
    gnupg \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    x11-apps \
    tmux \
    cmake \
    build-essential \
    sudo \
    tzdata \
    && rm -rf /var/lib/apt/lists/*

# 2. Gazebo Harmonic kurulumu ve ROS 2 köprüsü
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y \
    gz-harmonic \
    ros-humble-ros-gz \
    libgz-sim8-dev \
    rapidjson-dev \
    && rm -rf /var/lib/apt/lists/*

# 3. ArduPilot gereksinimi: root olmayan yetkili kullanıcı
ARG USER_NAME=uav
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USER_NAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USER_NAME && \
    echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USER_NAME && \
    chmod 0440 /etc/sudoers.d/$USER_NAME

USER $USER_NAME
WORKDIR /home/$USER_NAME

# 4. ArduPilot ve SITL altyapısı (uav kullanıcısı ile)
RUN git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git && \
    cd ardupilot && \
    git checkout ArduPlane-stable && \
    git submodule update --init --recursive

# Bağımlılıkları uav kullanıcısı ile yükle
RUN USER=$USER_NAME /home/$USER_NAME/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y && \
    sudo rm -rf /var/lib/apt/lists/*

# 5. Gazebo ArduPilot Eklentisi (ardupilot_gz)
RUN git clone https://github.com/ArduPilot/ardupilot_gz.git /home/$USER_NAME/ardupilot_gz && \
    cd /home/$USER_NAME/ardupilot_gz && \
    mkdir build && cd build && \
    cmake .. && make -j$(nproc)

# Ortam değişkenleri ve arama yolları
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/home/$USER_NAME/ardupilot_gz/build
ENV GZ_SIM_RESOURCE_PATH=/home/$USER_NAME/ardupilot_gz/models
ENV PATH="/home/$USER_NAME/ardupilot/Tools/autotest:/home/$USER_NAME/.local/bin:${PATH}"

# ROS 2 ortamını bashrc'ye bağla
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USER_NAME/.bashrc

CMD ["/bin/bash"]
