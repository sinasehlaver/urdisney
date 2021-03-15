// Moves the robot to a given configuration. The configuration is imported from a .txt file (a keyframe file).
// only the first line of the file is taken to consideration.

#include <iostream>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>

#include "functions.h"
#include "ur5xprs/gripper.h"
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLE DECLARATIONS


// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SUBSCRIBER CALLBACKS


// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // INITIALIZATIONS-----------------------------------------------------------------------------------------------------------
  
  if( argc != 5 ){
    std::cout<<"Wrong number of arguments, please enter the file name for gripper config and robot config with the extension in order and velocity and acceleration factors\n";
    return 0;
  }
  else{
    ros::init(argc, argv, "go_to_config");
    ros::NodeHandle nodeHandle;
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::Rate rate(100);

    // SUBSCRIBER/PUBLISHER DECLARATIONS------------------------------------------------------------------------------------------
    

    // VARIABLE DECLARATIONS------------------------------------------------------------------------------------------------------
    moveit::planning_interface::MoveGroupInterface group("manipulator");           // setup a MoveGroup class to control and plan for
    moveit::planning_interface::MoveGroupInterface gripper("gripper");
    std::vector<double>  robotConf;
    std::vector<double>  gripperConf;
    
    moveit::planning_interface::MoveGroupInterface::Plan planCurrent;    // Time parametrized trajectory is then taken from rt and inserted into plan

    char    menu_1    {'0'};  // menu selection characters
    bool    success   {0};    // boolean success tracker
    double  vel       {0.9};    // velocity scaling factor
    double  acc       {0.9};    // acceleration scaling factor
    double  gripper_value {0.7};
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
      gripperConf.clear();

      fileName = argv[1];

      importSingleFrame(gripperConf, fileName);
      gripper.setMaxVelocityScalingFactor(vel);
      gripper.setMaxAccelerationScalingFactor(acc);
      gripper.setJointValueTarget(gripperConf);

      for(auto i:gripper.getCurrentJointValues() ){
        std::cout<<i<<"\n";
      }

      gripper.move();

      for(auto i:gripper.getCurrentJointValues() ){
        std::cout<<i<<"\n";
      }

      fileName = argv[2];

      if(importSingleFrame(robotConf, fileName))
      {
        std::cout<<"Configuration imported"<<'\n';

        vel = atof(argv[3]);
        acc = atof(argv[4]);
        
        group.setMaxVelocityScalingFactor(vel);
        group.setMaxAccelerationScalingFactor(acc);
        group.setJointValueTarget(robotConf);

        success = (group.plan(planCurrent) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Planning for joint space goal) %s", success ? "SUCCEDED" : "FAILED");
        group.execute(planCurrent);
        ros::shutdown();
        return 0;
      }

    } 
    ros::shutdown();
    return 0;
  }
}
