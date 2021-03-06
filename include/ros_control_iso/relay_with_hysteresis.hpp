// **********************************************************************
//  Copyright 2014 GNU license
//  This controller header file belongs to relay with hysterisis. It is used to identify
//   system parameters by following the identification by self oscillation
//   described by Miskovic DOI: 10.1002/rob.20374
//
//   This controller is based on:
//   https://github.com/ros-controls/ros_control/wiki/controller_interface
//   and
//   https://github.com/labust/labust-ros-pkg/tree/master/ident_so
//
//   further modified by Raphael Nagel
//   raphael.nagel (#) posteo de
//   18/Aug/2014
//
////////////////////////////////////////////////////////////////////////
#ifndef __RELAY_W_HYSTERESIS__
#define __RELAY_W_HYSTERESIS__


#include <ros/node_handle.h>
#include <std_msgs/Float64.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include <vector>

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

#include <hardware_interface/joint_command_interface.h>

#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>


#define LINEAR 0
#define ANGULAR 1

#define RISING_EDGE 0
#define FALLING_EDGE 1

#define TRUE 1
#define FALSE 0

#define ALPHA 0
#define KX 1
#define KXX 2
#define DELTA 3
#define OMEGA_N 4
#define NUMPARAMS 5

#define STRICT 2
#define BEST_EFFORT 1


namespace ros_control_iso{

	class relay_with_hysteresis : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
	public:
	 	bool init(hardware_interface::EffortJointInterface* , ros::NodeHandle& );
		void update(const ros::Time& , const ros::Duration& period);
		void starting(const ros::Time& );
		void stopping(const ros::Time& );
	private:
		void do_Identification_Step(const ros::Duration&);
		int do_Identification_Switched(int,  const ros::Time&);
		void do_Identification_Parameter_Calculation(void);
		int store_I_SO_Solution(void);
		int real_time_publish(const ros::Time&);
		int get_parameters(void);
		void reset_waveform_meas(void);

    	double mean(const std::vector<double>);
    	double std2(const std::vector<double>, const double );   
		double wrapRad(double );
		double wrapDeg(double );



		// Relay Parameters:
		unsigned int sequence;
		double current_position;
		ros::NodeHandle nh_;
		double position_error;

		double update_rate;

		double relay_upper_limit;
		double relay_lower_limit;
		double relay_amplitude_out;
		double position_reference;
		bool linear_or_angular;

		double maxPosition_encountered;
		double minPosition_encountered;

		double tSum;
		int identLen;
		int counterHigh;
		int counterLow;

		bool finished;
		bool notified_server;

		double minMaxError;
		double eMaxError;
		double eMinError;

		std::string my_joint;

		ros::ServiceClient next_client;

		std::vector<double> e_max, e_min, t_max, t_min, xa_high, xa_low;
		std::vector<double> solutions;
		hardware_interface::EffortJointInterface* hw_;
		hardware_interface::JointHandle joint_;
 		//realtime_tools::RealtimeBuffer<Commands> command_;
 		//Commands command_struct_; // pre-allocated memory that is re-used to set the realtime buffer
		boost::scoped_ptr <realtime_tools::RealtimePublisher <control_msgs::JointControllerState> > controller_state_publisher_ ;

	};
};

#endif
