FROM osrf/ros:melodic-desktop-full

ENV ROS_DISTRO melodic

# nvidia-docker hooks
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

# Install packages
RUN apt-get update && \
    apt-get install -y \
    wget && \
    rm -rf /var/lib/apt/lists/*

RUN wget https://www.baslerweb.com/fp-1615275617/media/downloads/software/pylon_software/pylon_6.2.0.21487-deb0_amd64.deb && \
    wget https://bugs.launchpad.net/~ubuntu-security-proposed/+archive/ubuntu/ppa/+build/18845128/+files/libicu55_55.1-7ubuntu0.5_amd64.deb && \
    wget http://security.ubuntu.com/ubuntu/pool/universe/x/xerces-c/libxerces-c3.1_3.1.3+debian-1_amd64.deb && \
    wget https://launchpad.net/~ubuntu-security/+archive/ubuntu/ppa/+build/15108504/+files/libpng12-0_1.2.54-1ubuntu1.1_amd64.deb && \
    wget https://github.com/i3drobotics/phobosIntegration/releases/download/v1.0.54/Phobos-1.0.54-x86_64_reducedTemplates.deb

RUN dpkg -i pylon_6.2.0.21487-deb0_amd64.deb && \
    dpkg -i libicu55_55.1-7ubuntu0.5_amd64.deb && \
    dpkg -i libxerces-c3.1_3.1.3+debian-1_amd64.deb && \
    dpkg -i libpng12-0_1.2.54-1ubuntu1.1_amd64.deb && \
    dpkg -i Phobos-1.0.54-x86_64_reducedTemplates.deb

RUN apt-get update && \
    apt-get install -y \
    python-catkin-tools python-catkin-pkg python-rosdep python-wstool \
    ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-roslint \
    ros-$ROS_DISTRO-nodelet-core ros-$ROS_DISTRO-ddynamic-reconfigure && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p ~/catkin/src

WORKDIR /root/catkin
# Install wstool workspace packages
ADD ./install/*.rosinstall /tmp/
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash" && \
    wstool init src /tmp/i3dr_stereo_camera_https.rosinstall
# Replace i3dr_stereo_cam-ros with current repository
RUN rm -rf ~/catkin/src/i3dr_stereo_camera-ros && \
    mkdir -p ~/catkin/src/i3dr_stereo_camera-ros/
ADD . /root/catkin/src/i3dr_stereo_camera-ros/
# Install ROS catkin workspace dependencies
RUN apt-get update && \
    rosdep install --from-paths ~/catkin/src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash" && \
#     # /bin/bash -c "source /devel/setup.bash" && \
#     catkin build