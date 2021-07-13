# I3DR Stereo Camera ROS package

This is the generic stereo camera package for use with Industial 3D Robotics cameras. To avoid repeated code this package contains launchers and the like that would be the same across all i3dr stereo camera packages (e.g. stereo_matcher).

This package also includes all the nodes needed for stereo matching with the I3D Stereo Matcher.

## Status
[![ROS Build](https://github.com/i3drobotics/i3dr_stereo_camera-ros/actions/workflows/ros-build.yml/badge.svg)](https://github.com/i3drobotics/i3dr_stereo_camera-ros/actions/workflows/ros-build.yml)

## Build

### Catkin Build

``` bash
cd /path/to/repo
catkin_make
```
Note: Will build with I3DRSGM support by default. To disable this, pass -DWITH_I3DRSGM=OFF to catkin_make.

### CUDA
There is an option to build the stereo matcher with CUDA. This will unlock some extra matchers and run the block matcher on the GPU.
However, to use CUDA openCV features, a different openCV version than is default for ROS needs to be installed. (Requires OpenCV_contrib modules)

Follow these instructions to install OpenCV on linux with cuda support [here](https://www.pyimagesearch.com/2016/07/11/compiling-opencv-with-cuda-support/)

To build with CUDA use the following build option:
```bash
catkin_make -DWITH_CUDA=ON
```

### I3DR Stereo Matcher
Install I3DRSGM support package:
```
curl https://github.com/i3drobotics/phobosIntegration/releases/download/v1.0.54/Phobos-1.0.54-x86_64_reducedTemplates.deb > i3drsgm.deb
sudo dpkg -i i3drsgm.deb
```
To build with the I3DR Stereo Matcher use the following command:
``` bash
catkin_make -DWITH_I3DRSGM=ON
```
As I3DR Stereo Matcher uses CUDA the following enviromental varaiables should be set to avoid JIT recompiling:
```
export CUDA_CACHE_MAXSIZE=2147483648
export CUDA_CACHE_DISABLE=0
```
Contact info@i3drobotics.com for a license to use this matcher. 
We will provide you with a license file that should be placed in the following folder after building the workspace:
```
~/catkin_ws/devel/lib/i3dr_stereo_camera/yourlicense.lic
```

## Calibration

Calibration is done using the launch file 'stereo_calibration', or using the argument 'calibrate:=true' when launching of the i3dr camera launchers.

```bash
roslaunch i3dr_phobos_nuclear phobos_nuclear.launch calibrate:=true
```

This uses the cameracalibrator node in image_pipeline and needs to be set to executable to able to run using the following command:

```bash
sudo chmod +x src/image_pipeline/camera_calibration/nodes/cameracalibrator.py
sudo chmod +x src/image_pipeline/camera_calibration/nodes/cameracheck.py
```

## Basler

I3DR stereo cameras use basler cameras as a stereo pair. When using GigE cameras in a multi camera setup the netword switch must be setup for jumbo frames.

The ROS launcher 'stereo_capture.launch' uses a default value of 3000 MTU.

List current mtu size:

```bash
ip link show | grep mtu
```

Set mtu size

```bash
ip link set eth0 mtu 3000
```

Add this to /etc/rc.local to run at startup
