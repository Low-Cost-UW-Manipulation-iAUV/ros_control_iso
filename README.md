ros_control_iso
===============

Implementing the Identification through Self-Oscillation as a ros_control controller.

The controller relies on the EffortJointInterface.

This controller is based on: 
https://github.com/ros-controls/ros_control/wiki/controller_interface
and
https://github.com/labust/labust-ros-pkg/tree/master/ident_so

It has been tested in simulation and works for linear as well as angular DOFs. 
For angular DOFs please note that it expects the position in deg and will automatically wrap them to +-pi. Therefore the relay width cannot be bigger than +-pi respectively. Otherwise it will never trigger.

All parameters required for setting up the relay with hysteresis are to be set in the I-SO.yaml file.
This file needs to be loaded:
```ros
rosparam load ...src/ros_control_iso/I-SO.yaml
```
and then the identification server loaded by:
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

The identified parameters are stored in the parameter server under their respective axis name (see I-SO.yaml) and can be displayed i.e.:
```ros
rosparam get ros_control_iso/x/solution/
```
