FROM osrf/ros:noetic-desktop-full

RUN sudo apt-get update && \
    sudo apt-get install -y \
    wget && \
    sudo rm -rf /var/lib/apt/lists/*

RUN wget https://www.baslerweb.com/fp-1615275617/media/downloads/software/pylon_software/pylon_6.2.0.21487-deb0_amd64.deb && \
    wget https://bugs.launchpad.net/~ubuntu-security-proposed/+archive/ubuntu/ppa/+build/18845128/+files/libicu55_55.1-7ubuntu0.5_amd64.deb && \
    wget http://security.ubuntu.com/ubuntu/pool/universe/x/xerces-c/libxerces-c3.1_3.1.3+debian-1_amd64.deb && \
    # wget https://launchpad.net/~ubuntu-security/+archive/ubuntu/ppa/+build/15108504/+files/libpng12-0_1.2.54-1ubuntu1.1_amd64.deb && \
    wget https://github.com/i3drobotics/phobosIntegration/releases/download/v1.0.54/Phobos-1.0.54-x86_64_reducedTemplates.deb
    
RUN dpkg -i pylon_6.2.0.21487-deb0_amd64.deb && \
    dpkg -i libicu55_55.1-7ubuntu0.5_amd64.deb && \
    dpkg -i libxerces-c3.1_3.1.3+debian-1_amd64.deb && \
    # dpkg -i libpng12-0_1.2.54-1ubuntu1.1_amd64.deb && \
    dpkg -i Phobos-1.0.54-x86_64_reducedTemplates.deb

RUN mkdir ~/catkin && \
    mkdir ~/catkin/src