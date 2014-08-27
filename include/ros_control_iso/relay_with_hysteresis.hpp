/**********************************************************************
*   This controller header file belongs to
*	 relay with hysterisis. It is used to identify
*   system parameters by following the identification by self oscillation
*   described by Miskovic DOI: 10.1002/rob.20374
*
*   This controller is based on: 
*   https://github.com/ros-controls/ros_control/wiki/controller_interface
*   and
*   https://github.com/labust/labust-ros-pkg/tree/master/ident_so
*
*   further modified by Raphael Nagel
*   raphael.nagel@posteo.de
*   18/Aug/2014
*///////////////////////////////////////////////////////////////////////
#ifndef __RELAY_W_HYSTERESIS__
#define __RELAY_W_HYSTERESIS__


#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

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


namespace controller_ns{

	class ISO_Relay : public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
	public:
	 	bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
		void update(const ros::Time& time, const ros::Duration& period);
		void starting(const ros::Time& time);
		void stopping(const ros::Time& time); 
	private:
		int do_Identification_Step(void);
		int do_Identification_Switched(int);
		int do_Identification_Parameter_Calculation(void);
		int store_I_SO_Solution(void);


		double current_position;

		double position_error;

		double sampling_rate;
		
		hardware_interface::JointHandle joint_;
		double relay_upper_limit;
		double relay_lower_limit;
		double relay_amplitude_out;
		double position_reference;
		bool linear_or_angular;

		double maxPosition;
		double minPosition;

		double tSum;
		double identLen;

		bool finished;
		double minMaxError;

		std::string my_joint;

		std::vector<double> e_max, e_min, t_max, t_min, xa_high, xa_low;
		std::vector<double> params;

	};
};

#endif