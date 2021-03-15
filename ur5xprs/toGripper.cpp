// Moves the robot to a given configuration. The configuration is imported from a .txt file (a keyframe file).
// only the first line of the file is taken to consideration.

#include <iostream>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>

#include "functions.h"

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLE DECLARATIONS


// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SUBSCRIBER CALLBACKS
void setValuetoConfig(double &value, std::vector<double> &pose){
  pose.clear();
  pose.push_back(value);
  pose.push_back(value);
  pose.push_back(-value);
  pose.push_back(-value);
  pose.push_back(-value);
  pose.push_back(value);

}

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if( argc != 2 ){
    std::cout<<"Wrong number of arguments, please enter a value between 0.0 and 0.7 for gripper to go\n";
    return 0;
  }
  else{
    // INITIALIZATIONS-----------------------------------------------------------------------------------------------------------
    ros::init(argc, argv, "go_to_config");
    ros::NodeHandle nodeHandle;
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::Rate rate(100);

    // SUBSCRIBER/PUBLISHER DECLARATIONS------------------------------------------------------------------------------------------
    

    // VARIABLE DECLARATIONS------------------------------------------------------------------------------------------------------
    moveit::planning_interface::MoveGroupInterface gripper("gripper");           // setup a MoveGroup class to control and plan for
    
    std::vector<double>  gripperConf;
    
    char    menu_1    {'0'};  // menu selection characters
    bool    success   {0};    // boolean success tracker

    double  vel       {0.3};    // velocity scaling factor
    double  acc       {0.3};    // acceleration scaling factor
    double  value     {0.0};
    std::string fileName;     // name of the keyframe file to be imported
    
    // DEBUG: DELETE THESE

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // THE WHOLE BLOODY AFFAIR-----------------------------------------------------------------------------------------------------

    while(ros::ok())
    {
      gripperConf.clear(); // Clear current keyframe register

      value = atof(argv[1]);

      setValuetoConfig(value, gripperConf);

      gripper.setMaxVelocityScalingFactor(vel);
      gripper.setMaxAccelerationScalingFactor(acc);
      gripper.setJointValueTarget(gripperConf);

      gripper.move();
      ros::shutdown();
      return 0;

    } 
    ros::shutdown();
    return 0;
  }
}
