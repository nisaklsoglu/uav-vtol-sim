# Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic + ArduPilot SITL (ArduPlane / QuadPlane VTOL)
# Apple Silicon Mac ve Jetson Orin NX (JetPack 6.2) için ARM64 / aarch64 native mimari
FROM ros:humble-ros-base-jammy

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=uav
ARG USER_UID=1000
ARG USER_GID=1000
ARG ARDUPILOT_REF=ArduPlane-stable

ENV TZ=Europe/Istanbul \
    GZ_VERSION=harmonic \
    LANG=C.UTF-8

SHELL ["/bin/bash", "-c"]

# 1. Temel araçlar ve Gazebo resmi deposu
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl wget git sudo lsb-release gnupg ca-certificates \
        build-essential cmake ninja-build pkg-config \
        python3-pip python3-venv python3-dev \
        nano less tmux net-tools iputils-ping \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# 2. Gazebo Harmonic, GStreamer ve ardupilot_gazebo derleme kütüphaneleri
RUN apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        libgz-sim8-dev rapidjson-dev \
        libopencv-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    && rm -rf /var/lib/apt/lists/*

# 3. ROS 2 Humble köprüsü, MAVROS ve GeographicLib veri setleri
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
        ros-humble-mavros ros-humble-mavros-extras \
        ros-humble-rviz2 \
        python3-colcon-common-extensions python3-rosdep python3-vcstool \
    && wget -qO- https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | bash \
    && rm -rf /var/lib/apt/lists/*

# 4. Root olmayan yetkili kullanıcı oluşturma (ArduPilot şartı)
RUN groupadd --gid ${USER_GID} ${USER_NAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USER_NAME} \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    && chmod 0440 /etc/sudoers.d/${USER_NAME}

USER ${USER_NAME}
WORKDIR /home/${USER_NAME}

# 5. ArduPilot kaynak kodları (HTTP/1.1 ve sığ alt modüller)
RUN git config --global http.version HTTP/1.1 \
    && git config --global http.postBuffer 1048576000 \
    && git config --global http.lowSpeedLimit 1000 \
    && git config --global http.lowSpeedTime 600 \
    && git clone --depth 1 --branch ${ARDUPILOT_REF} https://github.com/ArduPilot/ardupilot.git \
    && cd ardupilot \
    && ./Tools/gittools/submodule-sync.sh \
    && git submodule update --init --depth 1 modules/mavlink modules/waf modules/DroneCAN

# 6. ArduPilot ortam gereksinimleri (uav kullanıcısı ile)
RUN cd ardupilot \
    && USER=${USER_NAME} ./Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && sudo rm -rf /var/lib/apt/lists/*

# 7. SITL ön derlemesi (QuadPlane / ArduPlane çalıştırılabilir dosyaları)
RUN cd ardupilot \
    && export PATH="/home/${USER_NAME}/.local/bin:${PATH}" \
    && ./waf configure --board sitl \
    && ./waf plane

# 8. ardupilot_gazebo (Harmonic eklentisi - VTOL modelleri hazır gelir)
RUN git clone --depth 1 https://github.com/ArduPilot/ardupilot_gazebo.git \
    && cd ardupilot_gazebo \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && make -j$(nproc)

# 9. Ortam değişkenleri ve ROS 2 kaynak bağlaması
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/home/${USER_NAME}/ardupilot_gazebo/build \
    GZ_SIM_RESOURCE_PATH=/home/${USER_NAME}/ardupilot_gazebo/models:/home/${USER_NAME}/ardupilot_gazebo/worlds \
    PATH=/home/${USER_NAME}/ardupilot/Tools/autotest:/home/${USER_NAME}/.local/bin:${PATH}

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && mkdir -p /home/${USER_NAME}/ros2_ws/src

CMD ["/bin/bash"]