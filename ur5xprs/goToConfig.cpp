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


// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // INITIALIZATIONS-----------------------------------------------------------------------------------------------------------
  ros::init(argc, argv, "go_to_config");
  ros::NodeHandle nodeHandle;
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::Rate rate(100);

  // SUBSCRIBER/PUBLISHER DECLARATIONS------------------------------------------------------------------------------------------
  

  // VARIABLE DECLARATIONS------------------------------------------------------------------------------------------------------
  moveit::planning_interface::MoveGroupInterface group("manipulator");           // setup a MoveGroup class to control and plan for
  
  std::vector<double>  robotConf;
  
  moveit::planning_interface::MoveGroupInterface::Plan planCurrent;    // Time parametrized trajectory is then taken from rt and inserted into plan

  char    menu_1    {'0'};  // menu selection characters
  bool    success   {0};    // boolean success tracker

  double  vel       {0.3};    // velocity scaling factor
  double  acc       {0.3};    // acceleration scaling factor
  
  std::string fileName;     // name of the keyframe file to be imported
  

  // DEBUG: DELETE THESE


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // THE WHOLE BLOODY AFFAIR-----------------------------------------------------------------------------------------------------

  std::cout<<"\n\n\n"<<"Plan to a target configuration imported from a file ('x' to shut down)"<<'\n';
  std::cout<<"Plan is computed by moveIt! using moveGroup.setJointValueTarget() and"<<'\n';
  std::cout<<"moveGroup.setJointValueTarget() methods."<<'\n';
  std::cout<<"------------------------------------------------------"<<'\n';
  while(ros::ok())
  {
    robotConf.clear(); // Clear current keyframe register

    std::cout<<"Enter configuration file name TO GO (without '.txt' extension)..."<<'\n';
    std::cin>>fileName;       //
    
    if(fileName == "x")
    {
      ros::shutdown();
      return 0;
    }

    fileName.append(".txt"); 

    if(importSingleFrame(robotConf, fileName))
    {
      std::cout<<"Configuration imported"<<'\n';
      
      group.setMaxVelocityScalingFactor(vel);
      group.setMaxAccelerationScalingFactor(acc);
      group.setJointValueTarget(robotConf);

      success = (group.plan(planCurrent) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Planning for joint space goal) %s", success ? "SUCCEDED" : "FAILED");
      
      std::cout<<"Enter any character to execute plan, 'x' to exit..."<<'\n';
        std::cin >> menu_1;

      if(menu_1 == 'x')
      {
        ros::shutdown();
        return 0;
      }
      else
        group.execute(planCurrent);
    }

  } 
  ros::shutdown();
  return 0;
}
