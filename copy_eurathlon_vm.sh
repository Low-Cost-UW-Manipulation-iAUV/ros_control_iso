#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync -avzh ./src/*.cpp 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/ros_control_iso/src/
rsync -avzh ./srv/*.srv							eurathlon_vm:/home/euratlhon/uwesub_msc/src/ros_control_iso/srv/
rsync -avzh ./include/ros_control_iso/*.hpp 	eurathlon_vm:/home/euratlhon/uwesub_msc/src/ros_control_iso/include/ros_control_iso/
rsync -avzh CMakeLists.txt 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/ros_control_iso/
rsync -avzh *.xml 								eurathlon_vm:/home/euratlhon/uwesub_msc/src/ros_control_iso/
rsync -avzh *.launch	 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/ros_control_iso/
rsync -avzh *.yaml		 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/ros_control_iso/
rsync -avzh *.md								eurathlon_vm:/home/euratlhon/uwesub_msc/src/ros_control_iso/

echo "All done, Good Success!"
