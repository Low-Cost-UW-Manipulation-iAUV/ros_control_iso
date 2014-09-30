#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync -avzh ./src/*.cpp 						BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/ros_control_iso/src/
rsync -avzh ./srv/*.srv							BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/ros_control_iso/srv/
rsync -avzh ./include/ros_control_iso/*.hpp 	BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/ros_control_iso/include/ros_control_iso/
rsync -avzh CMakeLists.txt 						BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/ros_control_iso/
rsync -avzh *.xml 								BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/ros_control_iso/
rsync -avzh *.launch	 						BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/ros_control_iso/
rsync -avzh *.yaml		 						BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/ros_control_iso/
rsync -avzh *.md								BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/ros_control_iso/

echo "All done, Good Success!"