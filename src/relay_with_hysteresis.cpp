/**********************************************************************
*  Copyright 2014 GNU license
* This controller is a relay with hysterisis. It is used to identify
*   system parameters by following the identification by self oscillation
*   described by Miskovic DOI: 10.1002/rob.20374
*
*   This controller is based on:
*   https://github.com/ros-controls/ros_control/wiki/controller_interface
*   and
*   https://github.com/labust/labust-ros-pkg/tree/master/ident_so
*
*   further modified by Raphael Nagel
*   raphael.nagel (#) posteo de
*   18/Aug/2014
*///////////////////////////////////////////////////////////////////////

#include <sstream>
#include <pluginlib/class_list_macros.h>

#include <controller_interface/controller.h>
#include <controller_manager_msgs/SwitchController.h>

#include <hardware_interface/joint_command_interface.h>

#include "ros_control_iso/NumberManipulation.hpp""

#include "ros_control_iso/relay_with_hysteresis.hpp"
#include "ros_control_iso/nextDOF.h"



namespace ros_control_iso{

  /** init() gets called when the relay ros_controller is being loaded
  *
  *
  * \author Raphael Nagel
  * \date 18/Aug/2014
  **************************************** */
  bool relay_with_hysteresis::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n) {
    nh_ = n;
    hw_ = hw;

    reset_waveform_meas();
    /// get the relay parameters
    if(get_parameters()){
      ROS_INFO("ros_control_iso - relay_with_hysteresis: Loaded all parameters, starting the realtime publisher.\n");
    } else {
      ROS_ERROR("ros_control_iso - relay_with_hysteresis: Could not load relay parameters.");
      return EXIT_FAILURE;
    }

     // Start realtime state publisher
    controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1) );
    ROS_INFO("ros_control_iso - relay_with_hysteresis: RealtimePublisher started\n");

    next_client = nh_.serviceClient<ros_control_iso::nextDOF>("/ros_control_iso/nextDOF");

    //return EXIT_SUCCESS;
    return 1;
  }

  void relay_with_hysteresis::reset_waveform_meas(void){
    current_position = 0;
    position_error = 0;
    sequence = 0;
    // reset the maximum position encountered
    maxPosition_encountered = -std::numeric_limits<double>::max();
    minPosition_encountered = std::numeric_limits<double>::max();
    tSum = 0;

    counterHigh = 0;
    counterLow = 0;

    finished = FALSE;
    notified_server = FALSE;
    minMaxError = std::numeric_limits<double>::max();
  }
    /** get_parameters(...) gets the parameters from the parameter server and stores them
  *   This gets the identification and relay parameters from the parameter server.
  *   List:
  *     Joint Name
  *       from it then get the joint handle
  *       check for linear or angular DOF
  *       Identification length for this DOF
  *         resize the measurements vectors
  *         resize the solution vector
  *       Relay Parameter for this DOF
  *
  *
  * \author Raphael Nagel
  * \date 18/Aug/2014
  **************************************** */
  int relay_with_hysteresis::get_parameters(void){
    std::ostringstream param_address;

    // get joint name from the parameter server
    if (!nh_.getParam("/ros_control_iso/joint", my_joint)) {
      ROS_ERROR("ros_control - ros_control_iso: Could not find joint name\n");
      return EXIT_FAILURE;
    } else {
      ROS_INFO("ros_control - ros_control_iso: Current joint is: %s", my_joint.c_str());
    }

    // get the joint object to use in the realtime loop
    try {
    joint_ = hw_->getHandle(my_joint);  // throws on failure

    }
    catch(...) {
      ROS_ERROR("ros_control - ros_control_iso: Exception happened - Could not get handle of the joint");
    }

    // Determining wether linear or angular procedures should be used
    if ( (my_joint == "x") || (my_joint == "y") || (my_joint == "z") ) {
      linear_or_angular = LINEAR;

    } else if ( (my_joint == "yaw") || (my_joint == "pitch") || (my_joint == "roll") ) {
      linear_or_angular = ANGULAR;

    } else {
      ROS_ERROR("ros_control - ros_control_iso: No valid joint referenced, ending\n");
      return EXIT_FAILURE;
    }

    /// Get the identification algorithm parameters: - Update Rate of the hardware loop and controller...
    if (!nh_.getParam("/ros_control_iso/relay_with_hysteresis/publish_rate", update_rate)) {
      ROS_ERROR("ros_control - ros_control_iso: Could not find update_rate\n");
      return EXIT_FAILURE;
    }

    param_address.clear();
    param_address.str("");
    param_address << "/ros_control_iso/"<< my_joint << "/parameters/identification_length";

  /// ... and how many cycles (full waveforms) do we keep track of for the purpose of the identification.
    if (!nh_.getParam(param_address.str(), identLen)) {
      ROS_ERROR("ros_control - ros_control_iso: Could not find identification length, assuming 10. \n");
      identLen = 10;
      nh_.setParam(param_address.str(), identLen);
    }

    // now that we know the identification length resize all the vectors...
    e_max.clear();
    e_max.resize(identLen,0);

    e_min.clear();
    e_min.resize(identLen,0);

    t_max.clear();
    t_max.resize(identLen);

    t_min.clear();
    t_min.resize(identLen);

    xa_high.clear();
    xa_high.resize(identLen,0);

    xa_low.clear();
    xa_low.resize(identLen,0);

    // 5 for number of parameters: alpha, kx, kxx, delta, omega_n
    solutions.clear();
    solutions.resize(5,0);

    ///Get the relay Parameters
    param_address.clear();
    param_address.str("");
    param_address << "/ros_control_iso/"<< my_joint << "/parameters/relay_upper_limit";
    if (!nh_.getParam(param_address.str(), relay_upper_limit)) {
      ROS_ERROR("ros_control - ros_control_iso: Could not find upper relay switching threshold\n");
      return EXIT_FAILURE;
    }


    param_address.clear();
    param_address.str("");
    param_address << "/ros_control_iso/"<< my_joint << "/parameters/relay_lower_limit";
    if (!nh_.getParam(param_address.str(), relay_lower_limit)) {
      ROS_ERROR("ros_control - ros_control_iso: Could not find lower relay switching threshold\n");
      return EXIT_FAILURE;
    }


    param_address.clear();
    param_address.str("");
    param_address << "/ros_control_iso/"<< my_joint << "/parameters/relay_amplitude_out_inNewtons";
    if (!nh_.getParam(param_address.str(), relay_amplitude_out)){
      ROS_ERROR("ros_control - ros_control_iso: Could not find relay amplitude out value\n");
      return EXIT_FAILURE;
    }


    param_address.clear();
    param_address.str("");
    param_address << "/ros_control_iso/"<< my_joint << "/parameters/position_reference";
    if (!nh_.getParam(param_address.str(), position_reference)){
      ROS_ERROR("ros_control - ros_control_iso: Could not find position_reference, assuming 0\n");
      position_reference = 0;
    }


    param_address.clear();
    param_address.str("");
    param_address << "/ros_control_iso/"<< my_joint << "/parameters/e_max_error";
    if (!nh_.getParam(param_address.str(), eMaxError)){
      ROS_ERROR("ros_control - ros_control_iso: Could not find e_max_error, assuming 0.1\n");
      eMaxError = 0.1;
      nh_.setParam(param_address.str(), eMaxError);
    }


    param_address.clear();
    param_address.str("");
    param_address << "/ros_control_iso/"<< my_joint << "/parameters/e_min_error";
    if (!nh_.getParam(param_address.str(), eMinError)){
      ROS_ERROR("ros_control - ros_control_iso: Could not find e_min_error, assuming 0.1\n");
      eMinError = 0.1;
      nh_.setParam(param_address.str(), eMinError);
    }
    return 1;  // The controller_manager does not like the EXIT_SUCCESS
  }


  /** update(...) implements the asynchronous relay update step
  *
  *
  * \author Raphael Nagel
  * \date 18/Aug/2014
  **************************************** */
  void relay_with_hysteresis::update(const ros::Time& time, const ros::Duration& period){
    //ROS_INFO("ros_control - ros_control_iso: Updating the controller output.\n");
    static double command_out = relay_amplitude_out;
    current_position = joint_.getPosition();

    do_Identification_Step();

    //Do the relay's job

    //If we have crossed the threshold rising edge and we are still driving upwards-->drive downwards
    if( (current_position > relay_upper_limit) && ( joint_.getCommand() == ( relay_amplitude_out) ) ){


      command_out = (-1) * relay_amplitude_out;
      joint_.setCommand(command_out);

      /// The control output is falling
      do_Identification_Switched(FALLING_EDGE, time);

      do_Identification_Parameter_Calculation();
    }else

    //same for going down-->start driving upwards
    if( ( current_position < relay_lower_limit) && ( joint_.getCommand() == ( (-1) * relay_amplitude_out) ) ){

      command_out = relay_amplitude_out;
      joint_.setCommand(command_out);

      /// The control output is rising
      do_Identification_Switched(RISING_EDGE , time);
      do_Identification_Parameter_Calculation();

    }


    if(finished == TRUE){

      // run the stuff below only once.
      if(notified_server == FALSE){
        joint_.setCommand(0);
        /// set the thruster output to
        ROS_INFO("ros_control - ros_control_iso: Identified the following parameters for axis %s: alpha: %f, k_x: %f, k_xx: %f, delta: %f, omega_n: %f. \n", my_joint.c_str(), solutions[ALPHA], solutions[KX], solutions[KXX], solutions[DELTA], solutions[OMEGA_N]);
        /// store the ISO solution in the parameter server
        store_I_SO_Solution();

        //Publish the state using the realtime safe way.
        real_time_publish(time);

        /// Tell the identification server that we want the next DOF to identify
        ros_control_iso::nextDOF do_next;
        do_next.request.now = my_joint;
        if(next_client.call(do_next)){
          ROS_INFO("ros_control_iso - relay_with_hysteresis: identification server called");
        } else{
          ROS_ERROR("ros_control_iso - relay_with_hysteresis: could not call identification server");
        }
        notified_server = TRUE;
      }
    }
  }


  /** starting() is called when the controller starts, it resets the variables to be identified
  *
  * \author Raphael Nagel
  * \date 18/Aug/2014
  **************************************** */
  void relay_with_hysteresis::starting(const ros::Time& time) {

    //reset the current waveform measurements
    reset_waveform_meas();
    ROS_INFO("ros_control - ros_control_iso: starting the controller. \n");

    /// get the parameters to maybe change the relay settings.
    if (get_parameters()) {
      ROS_INFO("ros_control_iso - relay_with_hysteresis: got parameters");
      ///Start the procedure off
      joint_.setCommand(relay_amplitude_out);

      ROS_INFO("ros_control - ros_control_iso: Relay has been reset, running now \n");
    } else {
      ROS_ERROR("ros_control_iso - relay_with_hysteresis: could not get parameters.");
      joint_.setCommand(0);
    }
  }

  /** stopping() gets called when the controller is being stopped, it sets output to 0
  *
  *
  * \author Raphael Nagel
  * \date 18/Aug/2014
  **************************************** */
  void relay_with_hysteresis::stopping(const ros::Time& time) {
    joint_.setCommand(0);
    ROS_INFO("ros_control - ros_control_iso: Relay is stopping, output set to 0\n");
  }


  /** do_Identification() runs a data collection step necessary for the ros_control_iso parameter calculation
  * It should be run at every relay update step
  *
  *
  * \author Raphael Nagel
  * \date 18/Aug/2014
  **************************************** */
  void relay_with_hysteresis::do_Identification_Step(void){
    //Update the variables for the identification
    position_error = position_reference - current_position;
    tSum += 1/update_rate; //measure the time during this half waveform


    //Handle angular values and angular wrapping, maps it into +- pi range
    if (linear_or_angular == ANGULAR) {
      position_error = labust::math::wrapRad(
          labust::math::wrapRad(position_reference) - labust::math::wrapRad(current_position)
        );
    }

    //Store the largest (+ or -) position values encountered
    if (position_error > maxPosition_encountered) {
      maxPosition_encountered = position_error;
    }
    if (position_error < minPosition_encountered) {
      minPosition_encountered = position_error;
    }

  }

  /** do_Identification_Switched() is the ros_control_iso data collection step
  * It is run at every relay switch.
  * This means it executes once per half waveform of the position waveform plot caused by the self oscillationh_.
  *
  * \param rising_falling Indicates a rising edge or falling edge relay limit trigger causing a relay output switch.
  *
  * \author Raphael Nagel
  * \date 18/Aug/2014
  **************************************** */
  int relay_with_hysteresis::do_Identification_Switched(int rising_falling, const ros::Time& time) {
    if ( rising_falling == RISING_EDGE ) {
      xa_high[counterHigh] = position_error;  // the position value when the switch acctually happened (this might differ from when we wanted it to happen due to delay in the system)
      e_min[counterLow] = minPosition_encountered;   //The minimum position value ever encountered during this half waveform
      t_min[counterLow] = tSum;     //the time taken for this half waveform
      tSum = 0;
      counterLow=(counterLow+1)%identLen;

    }else if (rising_falling == FALLING_EDGE){
      xa_low[counterLow] = position_error;
      e_max[counterHigh] = maxPosition_encountered;
      t_max[counterHigh] = tSum;
      tSum = 0;
      counterHigh=(counterHigh+1)%identLen;

    }else{
      ROS_ERROR("ros_control - ros_control_iso: Relay switched neither up nor down\n");
      return EXIT_FAILURE;
    }

    real_time_publish(time);

    //reset the maximum position encountered
    maxPosition_encountered = -std::numeric_limits<double>::max();
    minPosition_encountered  = std::numeric_limits<double>::max();

    return EXIT_SUCCESS;
  }

  int relay_with_hysteresis::real_time_publish(const ros::Time& time){

    //Publish the state using the realtime safe way.
   if(controller_state_publisher_ && controller_state_publisher_->trylock()){
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.header.seq = sequence;
      controller_state_publisher_->msg_.set_point = position_reference; //
      controller_state_publisher_->msg_.error = position_error;
      controller_state_publisher_->msg_.process_value = current_position;
      controller_state_publisher_->msg_.process_value_dot = minMaxError;
      controller_state_publisher_->msg_.command = joint_.getCommand();
      controller_state_publisher_->unlockAndPublish();
      sequence++;
      return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
  }
  /** do_Identification_Parameter_Calculation() calculates the Systems characteristic parameters
  *
  *
  * \author Raphael Nagel
  * \date 18/Aug/2014
  **************************************** */
  void relay_with_hysteresis::do_Identification_Parameter_Calculation(void) {

    double meanEMax = labust::math::mean(e_max);  // Mean of the maximum position encountered
    double stdEMax = labust::math::std2(e_max, meanEMax);  // calculate the standard deviation for that measurement

    double meanEMin = labust::math::mean(e_min);  // Mean of the minimuim (negative) encountered
    double stdEMin = labust::math::std2(e_min, meanEMin);  // calculate the standard deviation for that measurement

    double meanTMax = labust::math::mean(t_max);  // Mean of the time taken for each positive halfcycle
    double meanTMin = labust::math::mean(t_min);  // Mean of the time taken for each negative halfcycle

    double meanXaLow = labust::math::mean(xa_low);  // acctual position when the switch happened during negative halfcycle
    double meanXaHigh = labust::math::mean(xa_high);

    double T = (meanTMax + meanTMin);     // Average time a complete cycle takes
    double Xm = (0.5 * (meanEMax - meanEMin) );  // maximum Amplitude of the waveform
    double X0 = (0.5 * (meanEMax + meanEMin) );  // Find the zero point of the waveform, which is also its offset from 0.
    double xa_star = (0.5 * (meanXaHigh - meanXaLow) );  // Find the mean amplitude of the waveform at switching time. This is basically a standard mean calculationh_. It uses minus because the Xa_low is negative itself--> mean([Xa_high, abs(Xa_low)])

    double omega = 2 * M_PI / T;  //
  //  double C = relay_amplitude_out;

    double sq1 = (xa_star + X0) / Xm;
    sq1 = sqrt(1 - sq1 * sq1);

    double sq2 = (xa_star-X0) / Xm;
    sq2 = sqrt(1 - sq2 * sq2);

    /// ros_control_iso Parameter calculation
    solutions[ALPHA] = (2 * relay_amplitude_out * (sq1 + sq2) ) / (M_PI * omega * omega * Xm);
    solutions[KX]= (4.0 * relay_amplitude_out * xa_star) / (omega * M_PI * Xm * Xm);
    solutions[KXX]= (3.0 * relay_amplitude_out * xa_star) / (2 * omega * omega * Xm * Xm * Xm);
    solutions[DELTA] = relay_amplitude_out * (meanTMax - meanTMin) / (meanTMax + meanTMin);
    solutions[OMEGA_N] = omega;

    // std::cout<<"Xa_star"<<xa_star<<", X0"<<X0<<std::endl;
    // std::cout<<"Alpha = "<<solutions[ALPHA]<<", Kx = "<<solutions[KX]<<", Kxx = "<<solutions[KXX]<<", Delta:"<<solutions[DELTA]<<", w:"<<omega<<std::endl;
    // ROS_INFO("ros_control - ros_control_iso: rawish stuff: relay_amplitude_out: %f, sq1: %f, sq2: %f, omega: %f, Xm: %f, M_PI: %F \n", relay_amplitude_out, sq1, sq2, omega, Xm, M_PI );
    // ROS_INFO("ros_control - ros_control_iso: intmed. solutions: alpha: %f, k_x: %f, k_xx: %f, delta: %f, omega_n: %f. \n", solutions[ALPHA], solutions[KX], solutions[KXX], solutions[DELTA], solutions[OMEGA_N]);

    /// Test eMAX / eMIN standard deviation of this axis identification
    finished = ( (std::fabs(stdEMax / meanEMax) < eMaxError)  &&  (std::fabs(stdEMin / meanEMin) < eMinError) );
    // ROS_ERROR("eMaxError is currently: %f, eMinError is currently: %f",(stdEMax / meanEMax), (stdEMin / meanEMin));
    minMaxError = ( std::fabs(stdEMax / meanEMax) + std::fabs(stdEMin / meanEMin)) / 2;
  }

  /** store_I_SO_Solution() writes the system characteristic parameters found through the ros_control_iso to the parameter server
  *
  *
  * \author Raphael Nagel
  * \date 26/Aug/2014
  **************************************** */
  int relay_with_hysteresis::store_I_SO_Solution(void) {
    double test = 0;
    std::ostringstream param_address;

    param_address.clear();
    param_address.str("");

    param_address << "/ros_control_iso/" << my_joint << "/solution/alpha";
    ros::param::set(param_address.str(), solutions[ALPHA]);
    if ( ros::param::get(param_address.str(), test) ) {
      if (test !=solutions[ALPHA]) {
        ROS_ERROR("ros_control - ros_control_iso: Could not store ros_control_iso alpha solution on parameter server");
        return EXIT_FAILURE;
      }
    }
    param_address.clear();
    param_address.str("");

    param_address << "/ros_control_iso/" << my_joint << "/solution/k_x";
    ros::param::set(param_address.str(), solutions[KX]);
    if ( ros::param::get(param_address.str(), test) ) {
      if (test !=solutions[KX]) {
        ROS_ERROR("ros_control - ros_control_iso: Could not store ros_control_iso k_x solution on parameter server");
        return EXIT_FAILURE;
      }
    }
    param_address.clear();
    param_address.str("");

    param_address << "/ros_control_iso/" << my_joint << "/solution/k_xx";
    ros::param::set(param_address.str(), solutions[KXX]);
    if ( ros::param::get(param_address.str(), test) ) {
      if (test !=solutions[KXX]) {
        ROS_ERROR("ros_control - ros_control_iso: Could not store ros_control_iso k_xx solution on parameter server");
        return EXIT_FAILURE;
      }
    }
    param_address.clear();
    param_address.str("");

    param_address << "/ros_control_iso/" << my_joint << "/solution/delta";
    ros::param::set(param_address.str(), solutions[DELTA]);
    if ( ros::param::get(param_address.str(), test) ) {
      if (test !=solutions[DELTA]) {
        ROS_ERROR("ros_control - ros_control_iso: Could not store ros_control_iso delta solution on parameter server");
        return EXIT_FAILURE;
      }
    }
    param_address.clear();
    param_address.str("");

    param_address << "/ros_control_iso/" << my_joint << "/solution/omega_n";
    ros::param::set(param_address.str(), solutions[OMEGA_N]);
    if ( ros::param::get(param_address.str(), test) ) {
      if (test !=solutions[OMEGA_N]) {
        ROS_ERROR("ros_control - ros_control_iso: Could not store ros_control_iso omega_n solution on parameter server");
        return EXIT_FAILURE;
      }
    }
    return EXIT_SUCCESS;
  }
}

PLUGINLIB_EXPORT_CLASS(ros_control_iso::relay_with_hysteresis, controller_interface::ControllerBase)
