#ifndef __ROS_CONTROL_ISO_IDENT_SERVER__
#define __ROS_CONTROL_ISO_IDENT_SERVER__

#include "ros/ros.h"
#include <vector>
#include "ros_control_iso/nextDOF.h"
#include "std_srvs/Empty.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/LoadController.h"
#include "controller_manager_msgs/UnloadController.h"





#define STRICT 2
#define BEST_EFFORT 1



namespace ros_control_iso{


	class identification_server {
	public:
		identification_server(ros::NodeHandle &);
		~identification_server();
		bool next_DOF(ros_control_iso::nextDOF::Request &, ros_control_iso::nextDOF::Response &);
		bool start(std_srvs::Empty::Request& , std_srvs::Empty::Response& );
		bool stop(std_srvs::Empty::Request& , std_srvs::Empty::Response& );
		bool pause(std_srvs::Empty::Request& , std_srvs::Empty::Response& );



	private:
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
}; // end of namespace

#endif