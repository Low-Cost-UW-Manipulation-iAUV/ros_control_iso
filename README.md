ros_control_iso
===============

Implementing the Identification through Self-Oscillation as a ros_control controller.

The controller relies on the EffortJointInterface.

This controller is based on: 
https://github.com/ros-controls/ros_control/wiki/controller_interface
and
https://github.com/labust/labust-ros-pkg/tree/master/ident_so


All parameters required for setting up the relay with hysteresis are to be set in the I-SO.yaml file.
This file needs to be loaded and then the identification server loaded by:
```ros
rosrun ros_control_iso identification_server
```

It then needs to be started via a ros service call
```ros
rosservice call ros_control_iso/start
```

It can be stopped in between by doing
```ros
rosservice call ros_control_iso/stop
```
