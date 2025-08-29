# syntax=docker/dockerfile:1.7
#FROM --platform=linux/arm64 osrf/ros:humble-desktop
# syntax=docker/dockerfile:1.7
#FROM arm64v8/ros:humble-desktop

# syntax=docker/dockerfile:1.7
#FROM --platform=linux/arm64 ros:humble-desktop
# syntax=docker/dockerfile:1.7
ARG BASE_IMAGE=ros:humble-ros-base
FROM --platform=linux/arm64 ${BASE_IMAGE}



ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Taipei \
    ROS_DOMAIN_ID=87 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 所有 apt 先在 root 完成
RUN apt-get update && apt-get install -y --no-install-recommends \
      sudo curl wget git nano lsb-release gnupg2 locales \
      build-essential python3-pip python3-colcon-common-extensions \
      python3-vcstool python3-rosdep python3-argcomplete \
      net-tools iputils-ping tmux terminator htop psmisc \
      dbus-x11 libgl1 libxext6 libx11-6 libnss3 libgbm1 \
      g++ cmake pkg-config \
      libboost-serialization-dev libboost-filesystem-dev libboost-system-dev \
      libboost-program-options-dev libboost-test-dev \
      libeigen3-dev libode-dev libyaml-cpp-dev \
      castxml libboost-python-dev libboost-numpy-dev \
      python3-numpy python3-pyqt5.qtopengl freeglut3-dev libassimp-dev \
      python3-opengl python3-flask python3-celery libccd-dev libfcl-dev \
      libgpiod-dev \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-slam-toolbox \
      ros-humble-nav2-bringup ros-humble-nav2-msgs \
      ros-humble-turtlebot4-bringup ros-humble-turtlebot4-navigation ros-humble-turtlebot4-viz ros-humble-turtlebot4-msgs \
      ros-humble-diagnostic-updater \
      ros-humble-rqt-reconfigure \
      ros-humble-moveit ros-humble-geometric-shapes ros-humble-srdfdom \
    && rm -rf /var/lib/apt/lists/*

# rosdep
RUN rosdep init || true && rosdep update

# 建立使用者
ARG USER=rosuser
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} ${USER} \
 && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USER} \
 && echo "${USER} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 使用者環境
USER ${USER}
WORKDIR /home/${USER}
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
 && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc \
 && mkdir -p /home/${USER}/work
WORKDIR /home/${USER}/work

EXPOSE 22
ENTRYPOINT ["/entrypoint.sh", "terminator"]

