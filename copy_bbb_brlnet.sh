#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync -avzh ./src/*.cpp 						BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/ros_control_iso/src/
rsync -avzh ./srv/*.srv							BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/ros_control_iso/srv/
rsync -avzh ./include/ros_control_iso/*.hpp 	BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/ros_control_iso/include/ros_control_iso/
rsync -avzh CMakeLists.txt 						BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/ros_control_iso/
rsync -avzh *.xml 								BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/ros_control_iso/
rsync -avzh ./launch/*.launch					BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/ros_control_iso/launch/
rsync -avzh *.yaml		 						BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/ros_control_iso/
rsync -avzh *.md								BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/ros_control_iso/

echo "All done, Good Success!"
