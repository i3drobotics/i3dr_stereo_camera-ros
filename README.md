# I3DR Stereo Camera ROS package

This is the generic stereo camera package for use with Industial 3D Robotics cameras. To avoid repeated code this package contains launchers and the like that would be the same across all i3dr stereo camera packages (e.g. stereo_matcher).

This package also includes all the nodes needed for stereo matching with the I3D Stereo Matcher.

## Build

### Catkin Build

``` bash
cd /path/to/repo
catkin_make
```

### I3DR Stereo Algorithm

To build with the I3DR stereo algorithm use the following command:

``` bash
catkin_make -DENABLE_I3DR_ALG=ON
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

### Testing

Phobos Nuclear uses a laser is used to assist generating 3D data to give a patten for the matching. However, this can interfere with feature matching when mapping a room (using rtabmap). Line 3 on the basler cameras is used to indicate weather the lasers are on/off. This method can be tested using the python script pypylon_laser_test.py

```bash
python pypylon_laser_test.py
```

**TO USE THIS SCRIPT WITH ROS KINETIC** pypylon is not natively compiled for python 2.7 on x86_64 linux machines

Must be compiled from source (see pypylon readme for details)
https://github.com/basler/pypylon#installation-from-source

### IMPORTANT

When using basler cameras in a multi camera setup the netword switch must be setup for jumbo frames.

The ROS launcher 'stereo_capture.launch' uses a default value of 3000 MTU.

**Linux:**

List current mtu size:

```bash
ip link show | grep mtu
```

Set mtu size

```bash
ip link set eth0 mtu 3000
```

Add this to /etc/rc.local to run at startup

**Windows:**

Open the Network and Sharing Center.

Click Change adapter settings.

Right-click the NIC for which you want to enable jumbo frames and select Properties.

Under the Networking tab, click the Configure button for the network adapter.

Select the Advanced tab.

Select Jumbo Frame and change the value from disabled to the desired value, such as 3kB MTU, depending on the NIC.
