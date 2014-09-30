/**********************************************************************
*   This node controls the 5 DOF Identification by self-oscillation is a relay with hysterisis. 
*	It is used to identify
*   system parameters by following the identification by self oscillation
*   described by Miskovic DOI: 10.1002/rob.20374
*
*
*   further modified by Raphael Nagel
*   raphael.nagel (#) posteo de
*   29/Sept/2014
*///////////////////////////////////////////////////////////////////////

#include "ros_control_iso/identification_server.hpp"
#include <controller_interface/controller.h>
#include <cstdlib>


namespace ros_control_iso{
	/** identification_server() constructor
	*
	*
	* \author Raphael Nagel
	* \date 29/Sep/2014
	**************************************** */
	identification_server::identification_server(ros::NodeHandle &nh) {
		num_of_DOF = 0;
		current_DOF = 0;
		nh_ = nh;

		/// register as a client with the controller manager
		service_client_switcher = nh_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
		service_client_loader = nh_.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
		service_client_unloader = nh_.serviceClient<controller_manager_msgs::UnloadController>("/controller_manager/unload_controller");

		service1 = nh.advertiseService("/ros_control_iso/nextDOF", &identification_server::next_DOF, this);
		service2 = nh.advertiseService("/ros_control_iso/start", &identification_server::start, this);
		service3 = nh.advertiseService("/ros_control_iso/stop", &identification_server::stop, this);
		service4 = nh.advertiseService("/ros_control_iso/pause", &identification_server::pause, this);	
			
	}

	identification_server::~identification_server(){}


	/** next_DOF() sets the system up for the next DOF identification.
	*		It prepares the parameter server data so that the ros_controll controller
	*		can run again. 
	*
	* \author Raphael Nagel
	* \date 29/Sep/2014
	**************************************** */
	bool identification_server::next_DOF(ros_control_iso::nextDOF::Request &req, ros_control_iso::nextDOF::Response &res) {

		//stop the ros_controller
		controller_manager_msgs::SwitchController switcher;
		switcher.request.start_controllers.clear();
		switcher.request.stop_controllers.push_back("/ros_control_iso/relay_with_hysteresis");
		switcher.request.strictness = STRICT; //STRICT==2
		if(service_client_switcher.call(switcher)){
			ROS_INFO("ros_control_iso - identification_server: stopped the controller upon srv call to nextDOF");
		}else{
			ROS_ERROR("ros_control_iso - identification_server: failed to stop the controller upon srv call to nextDOF.");
		}

		/// check if the node and I talking about the same DOF
		if(list_to_ident[current_DOF] == req.now) {

			/// store the next DOF name in the parameter server
			current_DOF++;

			/// Have we gone through all DOFs to be identified?
			if(current_DOF >= num_of_DOF){
				ROS_INFO("ros_control_iso - Identification Process finished");
				return EXIT_SUCCESS;
			}

			// was storing successful?
			nh_.setParam("/ros_control_iso/joint", list_to_ident[current_DOF]);
			if (!nh_.getParam("/ros_control_iso/joint", list_to_ident[current_DOF])) {
			
				ROS_ERROR("ros_control_iso - could not set next DOF to be determined.");
				return EXIT_FAILURE;

			} else { //yes it was
				res.OK = EXIT_SUCCESS;

				// start the ros_controller again
				controller_manager_msgs::SwitchController switcher;
				switcher.request.start_controllers.push_back("/ros_control_iso/relay_with_hysteresis");
				switcher.request.stop_controllers.clear();
				switcher.request.strictness = STRICT; //STRICT==2
				if(service_client_switcher.call(switcher)){
					ROS_INFO("ros_control_iso - identification_server: started the next controller upon srv call to nextDOF");
				} else{
					ROS_ERROR("ros_control_iso - identification_server: failed to start the next controller upon srv call to nextDOF.");
				}

				return EXIT_SUCCESS;
			}			

		} else { // The controller is out of sync with me.
			ROS_ERROR("ros_control_iso - ros_controller has just finished a different identification than the server expected");
			return EXIT_FAILURE;
		}
	}



	/** start() sets the system up for the first DOF identification.
	*		It prepares the parameter server data so that the ros_controll controller
	*		can run again. 
	*
	* \author Raphael Nagel
	* \date 29/Sep/2014
	**************************************** */
	bool identification_server::start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
		num_of_DOF = 0;
		
		/// find out which DOF you want me to identify
		if(!nh_.getParam("/ros_control_iso/num_of_DOF", num_of_DOF)) {
			ROS_ERROR("ros_control_iso - couldnt get number of DOFs");
			return EXIT_FAILURE;

		}
		
		// fit the strings into the array
		list_to_ident.resize(num_of_DOF);

		if(!nh_.getParam("/ros_control_iso/list_to_ident", list_to_ident)) {
			ROS_ERROR("ros_control_iso - could not get list to ident");
			return EXIT_FAILURE;
		}


		/// set the first parameter to be identfied.
		nh_.setParam("/ros_control_iso/joint", list_to_ident[current_DOF]);		
		if (!nh_.getParam("/ros_control_iso/joint", list_to_ident[current_DOF])) {

			ROS_ERROR("ros_control_iso - could not set next DOF to be determined.");
			return EXIT_FAILURE;
			
		} else { // successfully set it, so now start the ros_control controller
		
			controller_manager_msgs::LoadController loader;
			loader.request.name = "/ros_control_iso/relay_with_hysteresis";
			if(service_client_loader.call(loader)){
				ROS_INFO("ros_control_iso - identification_server: loaded the controller upon start.");

			} else{
				ROS_ERROR("ros_control_iso - identification_server: could not load the controller upon start.");

			}
			/// start the ros_controller
			controller_manager_msgs::SwitchController switcher;
			switcher.request.start_controllers.push_back("/ros_control_iso/relay_with_hysteresis");
			switcher.request.stop_controllers.clear();
			switcher.request.strictness = STRICT; //STRICT==2
			if(service_client_switcher.call(switcher)){
				ROS_INFO("ros_control_iso - identification_server: started the controller upon start.");

			} else{
				ROS_ERROR("ros_control_iso - identification_server: could not start the controller upon start.");

			}
			return EXIT_SUCCESS;
		}
	}

	/** stop() stops the ident sequence
	*
	* \author Raphael Nagel
	* \date 29/Sep/2014
	**************************************** */
	bool identification_server::stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
		current_DOF = 0;

		/// start the ros_controller
		controller_manager_msgs::SwitchController switcher;
		switcher.request.start_controllers.clear();
		switcher.request.stop_controllers.push_back("/ros_control_iso/relay_with_hysteresis");
		switcher.request.strictness = STRICT; //STRICT==2
		if(service_client_switcher.call(switcher)){

		} else{
			ROS_ERROR("ros_control_iso - identification_server: could not stop the controller upon a srv stop request");
		}

		controller_manager_msgs::UnloadController unloader;
		unloader.request.name = "/ros_control_iso/relay_with_hysteresis";
		service_client_unloader.call(unloader);		

	}	

	/** pause() stops the current ident sequence, so that it can be resumed
	*
	*
	* \author Raphael Nagel
	* \date 29/Sep/2014
	**************************************** */
	bool identification_server::pause(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {

		/// start the ros_controller
		controller_manager_msgs::SwitchController switcher;
		switcher.request.start_controllers.clear();		
		switcher.request.stop_controllers.push_back("/ros_control_iso/relay_with_hysteresis");
		switcher.request.strictness = STRICT; //STRICT==2
		service_client_switcher.call(switcher);

	}		
} // end of namespace


int main(int argc, char **argv){
	ros::init(argc, argv, "identification_server");
	ros::NodeHandle nh;

	ros_control_iso::identification_server identer(nh);

	//service_client_switcher = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");


  	ROS_INFO("Services are running");
	
	ros::spin();

	ROS_INFO("identification_server: Shutting down ");

}