/**********************************************************************
*   Copyright 2014 GNU License
*	This node controls the 5 DOF Identification by self-oscillation. 
*	It will load and unload the relay_with_hysteresis controller 
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

namespace ros_control_iso{
	/** identification_server() constructor
	*
	*
	* \author Raphael Nagel
	* \date 29/Sep/2014
	**************************************** */
	identification_server::identification_server(const ros::NodeHandle &nh) {
		num_of_DOF = 0;
		current_DOF = 0;
		nh_ = nh;
		double identification_server_update_rate = 0;

		/// register as a client with the controller manager
		service_client_switcher = nh_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
		service_client_loader = nh_.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
		service_client_unloader = nh_.serviceClient<controller_manager_msgs::UnloadController>("/controller_manager/unload_controller");
		service_client_ISO_reference = nh_.serviceClient<ros_control_iso::string_ok>("/hw_loop/setIsoReference");
		/// register the services offered by this node
		service1 = nh_.advertiseService("/ros_control_iso/next_DOF", &identification_server::next_DOF, this);
		service2 = nh_.advertiseService("/ros_control_iso/start", &identification_server::start, this);
		service3 = nh_.advertiseService("/ros_control_iso/stop", &identification_server::stop, this);
		service4 = nh_.advertiseService("/ros_control_iso/pause", &identification_server::pause, this);

		/// set up the update loop with the update rate of identification_server_update_rate
		if (!nh_.getParam("/ros_control_iso/identification_server_update_rate", identification_server_update_rate)) {
			ROS_ERROR("ros_control_iso - identification_server: Could not find identification_server_update_rate, assuming 5Hz\n");
			identification_server_update_rate = 5;
			nh_.setParam("/ros_control_iso/identification_server_update_rate", identification_server_update_rate);
		}
		/// Set up the control loop by creating a timer and a connected callback
		ros::Duration update_freq = ros::Duration(1.0/identification_server_update_rate);
		non_realtime_loop_ = nh_.createTimer(update_freq, &identification_server::update, this);

	}

	identification_server::~identification_server() {}


	/** update() runs 1-10 times per second (see .yaml) to update the current state.
	*		It prepares the parameter server data so that the ros_controll controller
	*		can run againh_.
	*
	* \author Raphael Nagel
	* \date 29/Sep/2014
	**************************************** */
	void identification_server::update(const ros::TimerEvent& event) {
		switch(system_state) {
            case RUNNING:
				// everything is running, nothing to do.
				break;
            case DOF_FINISHED:
				system_state = RUNNING;
				pause();
				ROS_INFO("ros_control_iso - identification_server: stopped the controller, sleeping for 10s before starting next DOF ident.");
				sleep(10);
				restart();
				break;
            case ALL_FINISHED:
				system_state = RUNNING;
				ROS_INFO("ros_control_iso - identification_server: all DOFs have been identified. Unloading the controller. Job done.");
				stop();
				break;
            case ERROR:
				system_state = RUNNING;
				// stop the controller
				ROS_ERROR("ros_control_iso - identification_server: an error has occured, see above for more. Pausing the controller");
				pause();
				break;
            default:
            	break;
		}
	}

	/** next_DOF() sets the system up for the next DOF identificationh.
	*		It prepares the parameter server data so that the ros_controll controller
	*		can run again.
	*		To prevent a lock the acctual srv call for starting and stopping is run in the update() loop.
	*
	* \author Raphael Nagel
	* \date 29/Sep/2014
	**************************************** */
	bool identification_server::next_DOF(ros_control_iso::string_ok::Request &req, ros_control_iso::string_ok::Response &res) {

		/// check if the node and I talking about the same DOF
		if(list_to_ident[current_DOF] == req.data) {

			/// Take the next DOF...
			current_DOF++;

			/// Have we gone through all DOFs to be identified?
			if(current_DOF >= (num_of_DOF - 1) ) {
				ROS_INFO("ros_control_iso - identification_server: Identification Process finished");
				// tell the update() that we are finished
				system_state = ALL_FINISHED;
				res.OK = 1;
				return 1;  // the ros_controller hardware loop does not like EXIT_SUCCESS...
			}

			/// Line up the next DOF to be identified
			nh_.setParam("/ros_control_iso/joint", list_to_ident[current_DOF]);
			std::string pre_setParam_DOF = list_to_ident[current_DOF];
			nh_.getParam("/ros_control_iso/joint", list_to_ident[current_DOF]);
			if ( list_to_ident[current_DOF] != pre_setParam_DOF) {
				ROS_ERROR("ros_control_iso - identification_server: could not set next DOF to be determined.");
				system_state = ERROR;
				res.OK = 0;
				return EXIT_FAILURE;

			} else {  // yes it was loaded successfully
				ROS_INFO("ros_control_iso - identification_server: just set the next DOF to be identified to %s", list_to_ident[current_DOF].c_str());
				// start the ros_controller again
				system_state = DOF_FINISHED;
				res.OK = 1;
				return 1; // ros_control does not like EXIT_SUCCESS
			}

		} else { // The controller is out of sync with me.
			ROS_ERROR("ros_control_iso - identification_server: ros_controller has just finished a different identification than the server expected");
			// Error--> stop the controller
			system_state = ERROR;
			res.OK = 0;
			return EXIT_FAILURE;
		}
	}


	/** start() sets the system up for the first DOF identificationh_.
	*		It prepares the parameter server data so that the ros_controll controller
	*		can run againh_.
	*
	* \author Raphael Nagel
	* \date 29/Sep/2014
	**************************************** */
	bool identification_server::start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {


		/// fit the DOFs to ident into the list
		if(!nh_.getParam("/ros_control_iso/list_to_ident", list_to_ident)) {
			ROS_ERROR("ros_control_iso - identification_server: could not get list to ident");
			return EXIT_FAILURE;
		}
		num_of_DOF = list_to_ident.size();


		/// set the first parameter to be identfied.
		nh_.setParam("/ros_control_iso/joint", list_to_ident[current_DOF]);
		if (!nh_.getParam("/ros_control_iso/joint", list_to_ident[current_DOF])) {

			ROS_ERROR("ros_control_iso - identification_server: could not set next DOF to be determined.");
			return EXIT_FAILURE;

		} else { // successfully set it, so now start the ros_control controller

			// Set the reference point for the ISO
			ros_control_iso::string_ok ref_setter;
			ref_setter.request.data = "ISO";
			while(service_client_ISO_reference.call(ref_setter) != 1) {

			}

			// Load the relay
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
			return 1;  // ros_control does not like EXIT_SUCCESS
		}
	}

	// private version
	bool identification_server::restart(void) {

		// Set the reference point for the ISO
		ros_control_iso::string_ok ref_setter;
		ref_setter.request.data = "ISO";
		while(service_client_ISO_reference.call(ref_setter) != 1) {

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
		return 1;  // ros_control does not like EXIT_SUCCESS

	}

	/** stop() stops the ident sequence
	*
	* \author Raphael Nagel
	* \date 29/Sep/2014
	**************************************** */
	bool identification_server::stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
		current_DOF = 0;
		// Set the reference point for the ISO
		ros_control_iso::string_ok ref_setter;
		ref_setter.request.data = "pool";
		while(service_client_ISO_reference.call(ref_setter) != 1) {

		}		
		/// start the ros_controller
		controller_manager_msgs::SwitchController switcher;
		switcher.request.start_controllers.clear();
		switcher.request.stop_controllers.push_back("/ros_control_iso/relay_with_hysteresis");
		switcher.request.strictness = STRICT; //STRICT==2
		if(service_client_switcher.call(switcher)){
			ROS_INFO("ros_control_iso - identification_server: stopped the controller upon a srv stop request");

		} else{
			ROS_ERROR("ros_control_iso - identification_server: could not stop the controller upon a srv stop request");
		}

		controller_manager_msgs::UnloadController unloader;
		unloader.request.name = "/ros_control_iso/relay_with_hysteresis";
		if(service_client_unloader.call(unloader)) {
			ROS_INFO("ros_control_iso - identification_server: unloaded the controller upon srv stop request");
			return 1;  // ros_control does not like EXIT_SUCCESS
		} else {
			ROS_INFO("ros_control_iso - identification_server: could not unload the controller upon srv stop request");
			return EXIT_FAILURE;
		}
	}

	/// private version
	bool identification_server::stop(void) {
		current_DOF = 0;

		/// stop the ros_controller
		controller_manager_msgs::SwitchController switcher;
		switcher.request.start_controllers.clear();
		switcher.request.stop_controllers.push_back("/ros_control_iso/relay_with_hysteresis");
		switcher.request.strictness = STRICT; //STRICT==2
		if(service_client_switcher.call(switcher)){
			ROS_INFO("ros_control_iso - identification_server: stopped the controller");

		} else{
			ROS_ERROR("ros_control_iso - identification_server: could not stop the controller");
			return EXIT_FAILURE;
		}
		/// then unload the controller
		controller_manager_msgs::UnloadController unloader;
		unloader.request.name = "/ros_control_iso/relay_with_hysteresis";
		if(service_client_unloader.call(unloader)) {
			ROS_INFO("ros_control_iso - identification_server: unloaded the controller");
			return 1;  // ros_control does not like EXIT_SUCCESS
		} else {
			ROS_INFO("ros_control_iso - identification_server: could not unload the controller");
			return EXIT_FAILURE;
		}

	}

	/** pause() pauses the current ident sequence, so that it can be resumed
	*
	*	This stops the controller, which sets the output to 0. However, it preserves the current DOF
	*	 and does not unload the controller from the controller_manager.
	* \author Raphael Nagel
	* \date 29/Sep/2014
	**************************************** */
	bool identification_server::pause(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {

		/// start the ros_controller
		controller_manager_msgs::SwitchController switcher;
		switcher.request.start_controllers.clear();
		switcher.request.stop_controllers.push_back("/ros_control_iso/relay_with_hysteresis");
		switcher.request.strictness = STRICT; //STRICT==2

		if(service_client_switcher.call(switcher)){
			ROS_INFO("ros_control_iso - identification_server: paused the controller upon srv pause request");
			return 1;  // ros_control does not like EXIT_SUCCESS
		} else{
			ROS_ERROR("ros_control_iso - identification_server: could not pause the controller upon srv stop request");
			return EXIT_FAILURE;
		}

	}
	/// private version
	bool identification_server::pause(void) {

			/// start the ros_controller
			controller_manager_msgs::SwitchController switcher;
			switcher.request.start_controllers.clear();
			switcher.request.stop_controllers.push_back("/ros_control_iso/relay_with_hysteresis");
			switcher.request.strictness = STRICT; //STRICT==2

			if(service_client_switcher.call(switcher)){
				ROS_INFO("ros_control_iso - identification_server: paused the controller");
				return 1;  // ros_control does not like EXIT_SUCCESS
			} else{
				ROS_ERROR("ros_control_iso - identification_server: could not pause the controller");
				return EXIT_FAILURE;
			}
		}
} // end of namespace


int main(int argc, char **argv){
	ros::init(argc, argv, "identification_server");
	ros::NodeHandle nh;

	ros_control_iso::identification_server identer(nh);

	//service_client_switcher = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");


  	ROS_INFO("Services are running");

	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();
	ros::waitForShutdown();

	ROS_INFO("identification_server: Shutting down ");

}
