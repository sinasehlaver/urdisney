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
  std::string saveFileName;
  int force;
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
      
      std::vector<int>::size_type size1 = planCurrent.trajectory_.joint_trajectory.points.size();

      force = 2;

      if(size1 + 1 <= force){
        force = 1;
      }

      std::cout<<"Enter configuration file name TO SAVE (without '.txt' extension)..."<<'\n';
      std::cin>>saveFileName;       //
      saveFileName.append(".txt");

      std::ofstream outputFile;

      outputFile.open (saveFileName);

      for(auto j: planCurrent.trajectory_.joint_trajectory.points[size1 - force - 1].positions)outputFile  << j << '\t';
      outputFile << '\n';

      outputFile.close();



    }

  }
  ros::shutdown();
  return 0;
}
