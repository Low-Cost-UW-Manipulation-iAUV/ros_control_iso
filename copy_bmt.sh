#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync -avzh ./src/*.cpp 						bmt:/home/devel/catkin_ws/src/ros_control_iso/src/
rsync -avzh ./srv/*.srv							bmt:/home/devel/catkin_ws/src/ros_control_iso/srv/
rsync -avzh ./include/ros_control_iso/*.hpp 	bmt:/home/devel/catkin_ws/src/ros_control_iso/include/ros_control_iso/
rsync -avzh CMakeLists.txt 						bmt:/home/devel/catkin_ws/src/ros_control_iso/
rsync -avzh *.xml 								bmt:/home/devel/catkin_ws/src/ros_control_iso/
rsync -avzh ./launch/*.launch					bmt:/home/devel/catkin_ws/src/ros_control_iso/launch/
rsync -avzh *.yaml		 						bmt:/home/devel/catkin_ws/src/ros_control_iso/
rsync -avzh *.md								bmt:/home/devel/catkin_ws/src/ros_control_iso/

echo "All done, Good Success!"
