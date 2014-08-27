/**********************************************************************
*   This controller is a relay with hysterisis. It is used to identify
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


#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <labust/math/NumberManipulation.hpp>
#include "include/ros_control_iso/relay_with_hysteresis.hpp"
using ros_control_iso;

/** init() gets called when the relay ros_controller is being loaded
*
*
* \author Raphael Nagel
* \date 18/Aug/2014
**************************************** */
bool ISO_Relay::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
{

  position_reference = 0;
  current_position= 0;
  position_error = 0;

  //reset the maximum position encountered
  maxPosition_encountered = -std::numeric_limits<double>::max();
  minPosition_encountered  = std::numeric_limits<double>::max();

  tSum = 0;

  counterHigh = 0;
  counterLow = 0;
  identLen = 0;

  finished = FALSE;
  minMaxError = std::numeric_limits<double>::max();


  // get joint name from the parameter server
  if (!n.getParam("/ros_control_iso/joint", my_joint)){
    ROS_ERROR("ros_control - ros_control_iso: Could not find joint name\n");
    return EXIT_FAILURE;
  }

  //Determining wether linear or angular procedures should be used
  switch(my_joint){
    case x:
    case y:
    case z:
      linear_or_angular = LINEAR;
      break;

    case yaw:
    case pitch:
    case roll:
      linear_or_angular = ANGULAR;
      break;

    default:
      ROS_ERROR("ros_control - ros_control_iso: No valid joint referenced, ending\n");
      return EXIT_FAILURE;
  }

  if (!n.getParam("/ros_control_iso/parameters/sampling_rate", sampling_rate)){
    ROS_ERROR("ros_control - ros_control_iso: Could not find sampling_rate\n");
    return EXIT_FAILURE;
  }

//How many cycles (full waveforms) do we keep track of for the purpose of the identification:
  if (!n.getParam("/ros_control_iso/parameters/identification_length", identlen)){
    ROS_ERROR("ros_control - ros_control_iso: Could not find identification length, assuming 10. \n");
    identLen = 0;
  }
  if (!n.getParam("/ros_control_iso/parameters/relay_upper_limit_limit_inMeters", relay_upper_limit)){
    ROS_ERROR("ros_control - ros_control_iso: Could not find upper relay switching threshold\n");
    return EXIT_FAILURE;
  }
  if (!n.getParam("/ros_control_iso/parameters/relay_lower_limit_limit_inMeters", relay_lower_limit)){
    ROS_ERROR("ros_control - ros_control_iso: Could not find lower relay switching threshold\n");
    return EXIT_FAILURE;
  }
  if (!n.getParam("/ros_control_iso/parameters/relay_amplitude_out_inNewtons", relay_amplitude_out)){
    ROS_ERROR("ros_control - ros_control_iso: Could not find relay amplitude out value\n");
    return EXIT_FAILURE;
  }            
  if (!n.getParam("/ros_control_iso/parameters/position_reference", position_reference)){
    ROS_ERROR("ros_control - ros_control_iso: Could not find position_referenc, assuming 0\n");

  } 
  // get the joint object to use in the realtime loop
  try{
  joint_ = hw->getHandle(my_joint);  // throws on failure

  }
  catch (...) {
    ROS_ERROR("ros_control - ros_control_iso: Exception happened - Could not get handle of the joint");
  }

   // Start realtime state publisher
  controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1));

  return EXIT_SUCCESS;
}

/** update(...) implements the asynchronous relay update step
*
*
* \author Raphael Nagel
* \date 18/Aug/2014
**************************************** */
void ISO_Relay_linear::update(const ros::Time& time, const ros::Duration& period)
{
  current_position = joint_.getPosition();
  
  do_Identification_Step();
 
  //Do the relay's job

  //If we have crossed the threshold rising edge and we are still powering up.
  if( (current_position > relay_upper_limit) && ( joint_.getCommand() == ( relay_amplitude_out) ) ){         
 
    joint_.setCommand((-1) * relay_amplitude_out);
    do_Identification_Switched(RISING_EDGE);
    do_Identification_Parameter_Calculation();
  }

  //same for going down
  if( ( current_position < relay_lower_limit) && ( joint_.getCommand() == ( (-1) * relay_amplitude_out) ) ){    
 
    joint_.setCommand(relay_amplitude_out);
    do_Identification_Switched(FALLING_EDGE);
    do_Identification_Parameter_Calculation();
 
  }
  if(current_position <= relay_upper_limit && current_position >= relay_lower_limit){   
    //if we are within the relay limits
  }else{
    ROS_ERROR("ros_control - ros_control_iso: Relay update failed.\n");
  }

  if(finished == TRUE){
    ROS_INFO("ros_control - ros_control_iso: Identified the following paramters for axis %s: alpha: %f, k_x: %f, k_xx: %f, delta: %f, omega_n: %f. \n", my_joint, params[ALPHA], params[KX], params[KXX], params[DELTA], params[OMEGA_N]);

    //Unload the relay
    controller_manager::SwitchController switcher;
    switcher.stop_controllers="relay_with_hysteresis";
    switcher.strictness = STRICT; //STRICT==2
    ros::service::call("/controller_manager/switch_controller", switcher);    
  }


}


/** starting() is called when the controller starts, it resets the variables to be identified
*
* \author Raphael Nagel
* \date 18/Aug/2014
**************************************** */
void ISO_Relay_linear::starting(const ros::Time& time) { 
  position_reference = 0;
  current_position= 0;
  position_error = 0;

  //reset the maximum position encountered
  maxPosition_encountered = -std::numeric_limits<double>::max();
  minPosition_encountered  = std::numeric_limits<double>::max();

  tSum = 0;

  counterHigh = 0;
  counterLow = 0;
  identLen = 0;

  finished = FALSE;
  minMaxError = std::numeric_limits<double>::max();

  ROS_INFO("ros_control - ros_control_iso: Relay has been reset, running now \n");
}

/** stopping() gets called when the controller is being stopped, it sets output to 0
* 
*
* \author Raphael Nagel
* \date 18/Aug/2014
**************************************** */
void ISO_Relay_linear::stopping(const ros::Time& time) { 
  joint_setCommand(0);
  ROS_INFO("ros_control - ros_control_iso: Relay is stopping, output set to 0\n");
}


/** do_Identification() runs a data collection step necessary for the ros_control_iso parameter calculation
* It should be run at every relay update step
*
*
* \author Raphael Nagel
* \date 18/Aug/2014
**************************************** */
void do_Identification_Step(void){
  //Update the variables for the identification
  position_error = position_reference - current_position;
  tSum += 1/sampling_rate; //measure the time during this half waveform


  //Handle angular values and angular wrapping
  if (linear_or_angular == ANGLULAR){
    position_error = labust::math::wrapRad(
        labust::math::wrapRad(position_reference) - labust::math::wrapRad(current_position)
      );
  }

  //Store the largest (+ or -) position values encountered
  if (position_error > maxPosition_encountered ){
    maxPosition_encountered = position_error;
  }
  if (position_error < minPosition_encountered ){
    minPosition_encountered = position_error;
  }

}

/** do_Identification_Switched() is the ros_control_iso data collection step
* It is run at every relay switch. 
* This means it executes once per half waveform of the position waveform plot caused by the self oscillation.
*
* \param rising_falling Indicates a rising edge or falling edge relay limit trigger causing a relay output switch.
*
* \author Raphael Nagel
* \date 18/Aug/2014
**************************************** */
int do_Identification_Switched(int rising_falling){
  if(rising_falling == RISING_EDGE){
    xa_high[counterHigh] = error; //the position value when the switch acctually happened (this might differ from when we wanted it to happen due to delay in the system)
    e_min[counterLow] = minPosition_encountered;   //The minimum position value ever encountered during this half waveform
    t_min[counterLow] = tSum;     //the time taken for this half waveform
    tSum = 0;
    counterLow=(counterLow+1)%identLen;

  }else if (rising_falling == FALLING_EDGE){
    xa_low[counterLow] = error;
    e_max[counterHigh] = maxPosition_encountered;
    t_max[counterHigh] = tSum;
    tSum = 0;
    counterHigh=(counterHigh+1)%identLen;      

  }else{
    ROS_ERROR("ros_control - ros_control_iso: Relay switched neither up nor down\n");
    return EXIT_FAILURE;
  }
  //reset the maximum position encountered
  maxPosition_encountered = -std::numeric_limits<double>::max();
  minPosition_encountered  = std::numeric_limits<double>::max(); 

//Publish the state using the realtime safe way.
  if(controller_state_publisher_ && controller_state_publisher_->trylock()){
    controller_state_publisher_->msg.header.stamp = time;
    controller_state_publisher_->msg.set_point = joint_.getCommand();
    controller_state_publisher_->msg.process_value = error;
    controller_state_publisher_->msg.command = relay_amplitude_out;
  }   

  return EXIT_SUCCESS;
}
/** do_Identification_Parameter_Calculation() calculates the Systems characteristic parameters
*
*
* \author Raphael Nagel
* \date 18/Aug/2014
**************************************** */
void do_Identification_Parameter_Calculation(void){

  double meanEMax = labust::math::mean(e_max);
  double stdEMax(labust::math::std2(e_max, meanEMax);

  double meanEMin = labust::math::mean(e_min);
  double stdEMin = labust::math::std2(e_min, meanEMin);

  double meanTMax = labust::math::mean(t_max);
  double meanTMin = labust::math::mean(t_min);

  double meanXaLow = labust::math::mean(xa_low);
  double meanXaHigh = labust::math::mean(xa_high);

  double T = (meanTMax + meanTMin);
  double Xm = (0.5 * (meanEMax - meanEMin) );
  double X0 = (0.5 * (meanEMax + meanEMin) );
  double xa_star = (0.5 * (meanXaHigh - meanXaLow) );

  double omega = 2 * M_PI / T;
//  double C = relay_amplitude_out;

  double sq1 = ( xa_star + X0) / Xm;
  sq1 = sqrt( 1 - sq1 * sq1);

  double sq2 = (xa_star-X0) / Xm;
  sq2 = sqrt( 1 - sq2 * sq2);

  /// ros_control_iso Parameter calculation
  params[ALPHA] = 2 * relay_amplitude_out * (sq1 + sq2) / (M_PI * omega * omega * Xm);
  params[KX]=( 4.0 * relay_amplitude_out * xa_star ) / ( omega * M_PI * Xm * Xm) ;
  params[KXX]=( 3.0 * relay_amplitude_out * xa_star) / ( 2 * omega * omega * Xm * Xm * Xm );
  params[DELTA] = relay_amplitude_out * ( meanTMax - meanTMin) / (meanTMax + meanTMin);
  params[OMEGA_N] = omega;

  //std::cout<<"Xa_star"<<xa_star<<", X0"<<X0<<std::endl;
  //std::cout<<"Alpha = "<<params[alpha]<<", Kx = "<<params[kx]<<", Kxx = "<<params[kxx]<<", Delta:"<<params[delta]<<", w:"<<omega<<std::endl;

  /// Test eMAX / eMIN standard deviation of this axis identification
  finished = ( (std::fabs(stdEMax / meanEMax) < eMaxError )  &&  (std::fabs(stdEMin / meanEMin) < eMinError) );
  minMaxError = ( std::fabs(stdEMax / meanEMax) + std::fabs(stdEMin / meanEMin) ) / 2;

}

/** store_I_SO_Solution() writes the system characteristic parameters found through the ros_control_iso to the parameter server
*
*
* \author Raphael Nagel
* \date 26/Aug/2014
**************************************** */
int store_I_SO_Solution(void){
  double test = 0;

  ros::param::set("/ros_control_iso/solution/alpha", params[ALPHA]);
  if(ros::param::get("/ros_control_iso/solution/alpha", test) ){
    if(test !=params[ALPHA]){
      ROS_ERROR("ros_control - ros_control_iso: Could not store ros_control_iso alpha solution on parameter server");
      return EXIT_FAILURE;
    }
  }
  ros::param::set("/ros_control_iso/solution/k_x", params[KX]);
  if(ros::param::get("/ros_control_iso/solution/k_x", test) ){
    if(test !=params[KX]){
      ROS_ERROR("ros_control - ros_control_iso: Could not store ros_control_iso k_x solution on parameter server");
      return EXIT_FAILURE;      
    }
  }  
  ros::param::set("/ros_control_iso/solution/k_xx", params[KXX]);
  if(ros::param::get("/ros_control_iso/solution/k_xx", test) ){
    if(test !=params[KXX]){
      ROS_ERROR("ros_control - ros_control_iso: Could not store ros_control_iso k_xx solution on parameter server");
      return EXIT_FAILURE;      
    }
  }
  ros::param::set("/ros_control_iso/solution/delta", params[DELTA]);
  if(ros::param::get("/ros_control_iso/solution/delta", test) ){
    if(test !=params[DELTA]){
      ROS_ERROR("ros_control - ros_control_iso: Could not store ros_control_iso delta solution on parameter server");
      return EXIT_FAILURE;      
    }
  }
  ros::param::set("/ros_control_iso/solution/omega_n", params[OMEGA_N]);
  if(ros::param::get("/ros_control_iso/solution/omega_n", test) ){
    if(test !=params[OMEGA_N]){
      ROS_ERROR("ros_control - ros_control_iso: Could not store ros_control_iso omega_n solution on parameter server");
      return EXIT_FAILURE;      
    }
  }
  return EXIT_SUCCESS;

}

PLUGINLIB_DECLARE_CLASS( ros_control_iso::relay_with_hysteresis, controller_interface::ControllerBase);
