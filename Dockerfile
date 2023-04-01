ARG IMAGE=osrf/ros:noetic-desktop-full

FROM ${IMAGE} AS ros-img

FROM ros-img AS env-setup

ARG INTERNAL_WORKSPACE=/ros_ws

WORKDIR ${INTERNAL_WORKSPACE}


ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

SHELL [ "/bin/bash" , "-c" ]

COPY repair_config_file.sh .

FROM env-setup AS gen-dep-install


RUN apt-get update && DEBIAN_FRONTEND=noninteractive && \
    apt install -y python3-colcon-common-extensions && \ 
    apt-get install -y make && \
    apt-get install -y g++ && \ 
    apt-get install psmisc && \
    apt-get install -y alsa-utils && \
    apt-get install -y python3-pip && \
    apt-get install -y git 

FROM gen-dep-install AS ros-dep-install

ARG ROS_DISTRO=noetic

RUN apt-get install -y ros-${ROS_DISTRO}-turtlebot3-msgs && \
    apt-get install -y ros-${ROS_DISTRO}-turtlebot3 && \
    apt-get install -y ros-${ROS_DISTRO}-usb-cam && \
    apt-get install -y ros-${ROS_DISTRO}-rviz && \
    apt-get install -y ros-${ROS_DISTRO}-rqt && \
    apt-get install -y ros-${ROS_DISTRO}-rqt-common-plugins && \
    apt-get install -y ros-${ROS_DISTRO}-joint-state-publisher && \
    apt-get install -y ros-${ROS_DISTRO}-ros-control ros-${ROS_DISTRO}-ros-controllers ros-${ROS_DISTRO}-gazebo-msgs && \
    apt-get install -y ros-${ROS_DISTRO}-gazebo-ros ros-${ROS_DISTRO}-gazebo-ros-control && \ 
    apt-get install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-laser-geometry && \ 
    apt-get install -y ros-${ROS_DISTRO}-tf-conversions ros-${ROS_DISTRO}-tf2-geometry-msgs && \ 
    apt-get install -y ros-${ROS_DISTRO}-joint-state-controller ros-${ROS_DISTRO}-effort-controllers && \ 
    apt-get install -y ros-${ROS_DISTRO}-position-controllers ros-${ROS_DISTRO}-velocity-controllers && \ 
    apt-get install -y ros-${ROS_DISTRO}-robot-state-publisher ros-${ROS_DISTRO}-joint-state-publisher && \
    apt-get install -y ros-${ROS_DISTRO}-joint-state-publisher-gui && \
    apt-get install -y ros-${ROS_DISTRO}-teleop-twist-keyboard


FROM ros-dep-install  AS set_entrypoint

COPY ros_entrypoint.sh /root/.

ENTRYPOINT ["/root/ros_entrypoint.sh"]

CMD ["bash"]