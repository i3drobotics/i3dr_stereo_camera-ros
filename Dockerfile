FROM osrf/ros:melodic-desktop-full

# nvidia-docker hooks
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

# Install packages
RUN apt-get update
RUN apt-get install wget -y
RUN apt-get install ros-melodic-rviz ros-melodic-rqt -y

RUN mkdir ~/catkin
RUN mkdir ~/catkin/src

WORKDIR ~/catkin

RUN wstool init src https://raw.githubusercontent.com/i3drobotics/i3dr_titania-ros/master/install/i3dr_titania_https.rosinstall
RUN sudo sh -c 'echo "yaml https://raw.githubusercontent.com/i3drobotics/pylon_camera/master/rosdep/pylon_sdk.yaml " > /etc/ros/rosdep/sources.list.d/15-plyon_camera.list'
RUN rosdep update -y

RUN wget https://www.baslerweb.com/fp-1615275617/media/downloads/software/pylon_software/pylon_6.2.0.21487-deb0_amd64.deb
RUN wget https://bugs.launchpad.net/~ubuntu-security-proposed/+archive/ubuntu/ppa/+build/18845128/+files/libicu55_55.1-7ubuntu0.5_amd64.deb
RUN wget http://security.ubuntu.com/ubuntu/pool/universe/x/xerces-c/libxerces-c3.1_3.1.3+debian-1_amd64.deb
RUN wget https://launchpad.net/~ubuntu-security/+archive/ubuntu/ppa/+build/15108504/+files/libpng12-0_1.2.54-1ubuntu1.1_amd64.deb
RUN wget https://github.com/i3drobotics/phobosIntegration/releases/download/v1.0.54/Phobos-1.0.54-x86_64_reducedTemplates.deb

RUN dpkg -i pylon_6.2.0.21487-deb0_amd64.deb
RUN dpkg -i libicu55_55.1-7ubuntu0.5_amd64.deb
RUN dpkg -i libxerces-c3.1_3.1.3+debian-1_amd64.deb
RUN dpkg -i libpng12-0_1.2.54-1ubuntu1.1_amd64.deb
RUN dpkg -i Phobos-1.0.54-x86_64_reducedTemplates.deb

RUN rosdep install --from-paths src --ignore-src -r -y
