FROM osrf/ros:foxy-desktop
LABEL maintainer="Siddharth Saha <sisaha@ucsd.edu>"

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update --fix-missing \
 && apt-get install -y \
    wget \
    lsb-release \
    sudo \
    mesa-utils \
    python3-pip \
    git \
    vim

RUN python3 -m pip install --upgrade pip

RUN pip3 install --no-cache-dir numpy \
                                pandas \
                                matplotlib \
                                sklearn \
                                pyyaml \
                                rospkg

RUN apt install -y libeigen3-dev
RUN dbus-uuidgen > /etc/machine-id

RUN apt-get install -y \
    ros-foxy-robot-localization \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-robot-state-publisher \
    ros-foxy-pointcloud-to-laserscan \
    ros-foxy-image-transport \
    ros-foxy-lgsvl-bridge \
    ros-foxy-lgsvl-msgs \
    ros-foxy-image-transport-plugins \
    ros-foxy-slam-toolbox \
    ros-foxy-xacro \
    ros-foxy-ros2bag \
    ros-foxy-rosbag2-converter-default-plugins \
    ros-foxy-rosbag2-storage-default-plugins \
    ros-foxy-ackermann-msgs \
 && apt-get clean

RUN mkdir -p /ros2_ws/src/tritonkart_bringup
RUN mkdir -p /ros2_ws/src/tritonkart_description
RUN mkdir -p /ros2_ws/src/tritonkart_mapping
RUN mkdir -p /ros2_ws/src/tritonkart_utilities
RUN mkdir -p /ros2_ws/src/tritonkart_localization
RUN mkdir -p /ros2_ws/src/tritonkart_logging
RUN mkdir -p /ros2_ws/src/tritonkart_navigation
COPY ./tritonkart_bringup /ros2_ws/src/tritonkart_bringup
COPY ./tritonkart_description /ros2_ws/src/tritonkart_description
COPY ./tritonkart_mapping /ros2_ws/src/tritonkart_mapping
COPY ./tritonkart_utilities /ros2_ws/src/tritonkart_utilities
COPY ./tritonkart_localization /ros2_ws/src/tritonkart_localization
COPY ./tritonkart_logging /ros2_ws/src/tritonkart_logging
COPY ./tritonkart_navigation /ros2_ws/src/tritonkart_navigation


RUN /bin/bash -c "echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc "
RUN /bin/bash -c "echo 'source /ros2_ws/install/setup.bash' >> ~/.bashrc "
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash; cd /ros2_ws; colcon build; source install/setup.bash"
