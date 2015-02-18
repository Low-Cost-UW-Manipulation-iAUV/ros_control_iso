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
#ifndef __ROS_CONTROL_ISO_TUNE_PID__
#define __ROS_CONTROL_ISO_TUNE_PID__

#include <cstdlib>
#include <vector>
#include <map>
#include <string>

#include <ros/ros.h>
#include <ros/node_handle.h>

namespace ros_control_iso {

class tune_controller {

public:
    tune_controller(const ros::NodeHandle &);
    ~tune_controller();

    int do_work(void);

private:
    int get_hydro_coeffs(const std::string);
    int get_desired_tf_coeffs(const std::string);
    void calc_gains(const std::string);
    int store_on_paramserver(const std::string);
    int store_in_yaml(void);

    ros::NodeHandle nh_;

    std::vector<std::string> list_to_ident;    

    std::map<std::string,double> hydro_coeffs;

    std::vector<double> desired_tf_coeffs;

    std::map<std::string, double> gains;
};


}  // End of namespace





#endif