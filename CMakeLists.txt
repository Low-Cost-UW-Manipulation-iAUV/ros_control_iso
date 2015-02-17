cmake_minimum_required(VERSION 2.8.3)
project(ros_control_iso)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
	controller_interface
	hardware_interface
	pluginlib
	control_msgs
	control_toolbox
	realtime_tools
	urdf
	message_generation
	roscpp
	std_srvs
	std_msgs
)


include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_service_files(
	DIRECTORY srv
	FILES string_ok.srv)


generate_messages(
	DEPENDENCIES
	std_msgs
	std_srvs
	)


# Declare catkin package
catkin_package(
	CATKIN_DEPENDS
		message_runtime
		controller_interface
		control_msgs
		control_toolbox
		realtime_tools
		urdf
		roscpp
		pluginlib
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME} src/relay_with_hysteresis.cpp )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(identification_server src/identification_server.cpp)
add_executable(tune_controller src/tune_controller.cpp)

target_link_libraries(identification_server ${catkin_LIBRARIES})
target_link_libraries(tune_controller ${catkin_LIBRARIES})

add_dependencies(identification_server ros_control_iso_generate_messages_cpp std_srvs_generate_messages_cpp)


# Install the pluginlib part of this node
install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS ${PROJECT_NAME}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES 	relay_with_hysteresis.xml
				I-SO.yaml
				relay_with_hysteresis.launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
