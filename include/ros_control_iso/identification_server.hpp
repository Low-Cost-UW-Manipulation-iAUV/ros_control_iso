//  *********************************************************************
//   This node controls the 5 DOF Identification by self-oscillation is a relay with hysterisis.
//    It is used to identify system parameters by following the identification by self oscillation
//   described by Miskovic DOI: 10.1002/rob.20374
//
//
//   further modified by Raphael Nagel
//   raphael.nagel (#) posteo de
//  Copyright 2014 - GNU license
//  29/Sept/2014
//  //////////////////////////////////////////////////////////////////////
#ifndef __ROS_CONTROL_ISO_IDENT_SERVER__
#define __ROS_CONTROL_ISO_IDENT_SERVER__

#include <cstdlib>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <ros/node_handle.h>

#include "ros_control_iso/nextDOF.h"
#include "std_srvs/Empty.h"

#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/LoadController.h"
#include "controller_manager_msgs/UnloadController.h"


#define STRICT 2
#define BEST_EFFORT 1


namespace ros_control_iso {

	//  system states
	#define DOF_FINISHED 1
	#define ALL_FINISHED 2
	#define RUNNING 3
	#define ERROR 4

	class identification_server {
    public:
		identification_server(const ros::NodeHandle &);
		~identification_server();
		bool next_DOF(ros_control_iso::nextDOF::Request&, ros_control_iso::nextDOF::Response&);
		bool start(std_srvs::Empty::Request& , std_srvs::Empty::Response&);
		bool stop(std_srvs::Empty::Request& , std_srvs::Empty::Response&);
		bool pause(std_srvs::Empty::Request& , std_srvs::Empty::Response&);

    private:
		void update(const ros::TimerEvent&);
		bool restart(void);
		bool stop(void);
		bool pause(void);
		unsigned int system_state;

		ros::Timer non_realtime_loop_;

		ros::ServiceServer service1;
		ros::ServiceServer service2;
		ros::ServiceServer service3;
		ros::ServiceServer service4;


		int num_of_DOF;
		int current_DOF;
		ros::NodeHandle nh_;
		ros::ServiceClient service_client_switcher;
		ros::ServiceClient service_client_loader;
		ros::ServiceClient service_client_unloader;


		std::vector<std::string> list_to_ident;
	};
};  // end of namespace

#endif
