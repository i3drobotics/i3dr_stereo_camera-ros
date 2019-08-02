# I3DR Stereo Camera ROS package

This is the generic stereo camera package for use with Industial 3D Robotics cameras. To avoid repeated code this package contains launchers and the like that would be the same across all i3dr stereo camera packages (e.g. stereo_matcher).

This package also includes all the nodes needed for stereo matching with the I3D Stereo Matcher.

## Basler

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
