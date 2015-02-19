//  *********************************************************************
//   This node turns the hydrodynamics drag and inertia coeffs into PID controller values
//   as described by Miskovic DOI: 10.1002/rob.20374
//
//
//   Authohr:Raphael Nagel
//   raphael.nagel (#) posteo de
//  Copyright 2014 - GNU license
//  20/Oct/2014
//  //////////////////////////////////////////////////////////////////////

#include "ros_control_iso/tune_controller.hpp"

namespace ros_control_iso{

    tune_controller::tune_controller(const ros::NodeHandle  &nh) {
        nh_ = nh;
       
        desired_tf_coeffs.resize(4,0);

        hydro_coeffs["alpha"] = 0;
        hydro_coeffs["beta_r"] = 0;
        hydro_coeffs["beta_rr"] = 0;
        hydro_coeffs["delta"] = 0;
        hydro_coeffs["omega_n"] = 0;

        gains["p"] = 0;
        gains["i"] = 0;
        gains["d"] = 0;
    }
    
    tune_controller::~tune_controller() {}

    /** do_work(): do all the individual steps
    */
    int tune_controller::do_work(void) {

        /// fit the DOFs to ident into the list
        if(!nh_.getParam("/ros_control_iso/list_to_ident", list_to_ident)) {
            ROS_ERROR("ros_control_iso - identification_server: could not get list to ident");
            return EXIT_FAILURE;
        }

        // run the steps for each DOF
        for (int x = 0; x < list_to_ident.size(); x++) {
            get_hydro_coeffs(list_to_ident[x]);
            get_desired_tf_coeffs(list_to_ident[x]);

            calc_gains(list_to_ident[x]);

            store_on_paramserver(list_to_ident[x]);
        }
        store_in_yaml();
        ROS_INFO("ros_control_iso - tune_controller: finished. Stopping the node.");
    }


    /** tune_PID(): turns hydrodynamic coeffs into PID controller values
    */
    void tune_controller::calc_gains(const std::string current_DOF){
        if (desired_tf_coeffs[3] != 0) {
            gains["i"] = hydro_coeffs["alpha"] / desired_tf_coeffs[3];
            gains["d"] = ( desired_tf_coeffs[2] * hydro_coeffs["alpha"] / desired_tf_coeffs[3]) - hydro_coeffs["beta_r"]; 
            gains["p"] = -1 * ( desired_tf_coeffs[1] * hydro_coeffs["alpha"]) / desired_tf_coeffs[3];
        } else {
            ROS_ERROR("ros_control_iso - tune_controller: coefficients for DOF: %s aren't set properly...",current_DOF.c_str());
        }
    }

    /** get_hydro_coeffs(): Get all the hydroidynamics coefficients for this DOF from the param server (p.s.)
    */
    int tune_controller::get_hydro_coeffs(const std::string current_DOF) {
        std::ostringstream param_address;
        param_address.clear();
        param_address.str("");

        param_address << "/ros_control_iso/" << current_DOF << "/solution";
        if (!nh_.getParam(param_address.str(), hydro_coeffs)) {

            ROS_ERROR("ros_control_iso - tune_controller: Could not find I-SO solution for DOF: %s", current_DOF.c_str());
            return EXIT_FAILURE;
        }
    }

    /** get_desired_tf_coeffs(): get the coeffs of the desired transfer function (for this DOF) from the p.s.
    */
    int tune_controller::get_desired_tf_coeffs(const std::string current_DOF) {
        std::ostringstream param_address;
        param_address.clear();
        param_address.str("");

        param_address << "/ros_control_iso/" << current_DOF << "/desired_tf";
        if (!nh_.getParam(param_address.str(), desired_tf_coeffs)) {

            ROS_ERROR("ros_control_iso - tune_controller: Could not find desired_tf for DOF: %s", current_DOF.c_str());
            return EXIT_FAILURE;
        }
    }

    /** store_on_paramserver(): store the found solution on the p.s.
    */
    int tune_controller::store_on_paramserver(const std::string current_DOF) {
        std::ostringstream param_address;
        param_address.clear();
        param_address.str("");

        param_address << "/controller/gains/" << current_DOF;
        nh_.setParam(param_address.str(), gains);
    } 

    /** store_in_yaml(): I would like to be able to store directly to a yaml file. 
            Since I don't know how to do that and its not important enough to waste 
            time on you (the user) will have to do it through the command line.
    */
    int tune_controller::store_in_yaml(void) {
        // I dont know how to do this programmatically, but who cares, let the user do it:
        ROS_INFO("ros_control_iso - tune_controller: I'm finished, please save the parameters in the yaml file by running: \n rosparam dump dump.yaml /controller");
    }


}  //End of namespace

int main(int argc, char **argv) {
    ros::init(argc, argv, "tune_controller");
    ros::NodeHandle nh;

    // create the instance of the class
    ros_control_iso::tune_controller tuner(nh);
    tuner.do_work();
    ROS_INFO("ros_control_iso - controller_tuner: Shutting down");
    ros::shutdown();    
}
