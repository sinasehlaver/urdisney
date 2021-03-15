// Calculates the cubic spline based joint space trajectory and generates the trajectory message based on IPTP
// TODO:  instead of using IPTP consider Parabolic Parameterization for generation of time stamps velocities etc.
//        IPTP is known to have bugs/glitches.

#include <iostream>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

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
  ros::init(argc, argv, "cubic_spline_interpolator");
  ros::NodeHandle nodeHandle;
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::Rate rate(100);

  // SUBSCRIBER/PUBLISHER DECLARATIONS------------------------------------------------------------------------------------------
  

  // VARIABLE DECLARATIONS------------------------------------------------------------------------------------------------------
  moveit::planning_interface::MoveGroupInterface group("manipulator");           // setup a MoveGroup class to control and plan for
  robot_trajectory::RobotTrajectory robotTrajectory(group.getCurrentState()->getRobotModel(), "manipulator");   // rt is used for IPTP
  
  std::vector<std::vector<double>>  keyFrames;
  std::vector<std::vector<double>>  interpolatedTrajectory;
  moveit_msgs::RobotTrajectory      trajectoryMsg;       // Interpolated joint trajectory is inserted into this
                                    // trajectoryMsg is then inserted into robotTrajectory class using setRobotTrajectoryMsg(trajectoryMsg).
  
  trajectory_processing::IterativeParabolicTimeParameterization iptp;   // create a IterativeParabolicTimeParameterization object

  moveit::planning_interface::MoveGroupInterface::Plan planCurrent;    // Time parametrized trajectory is then taken from rt and inserted into plan

  char    menu_1    {'0'};  // menu selection characters
  int     i         {0};    // counter
  bool    success   {0};    // boolean success tracker
  
  std::string fileName;     // name of the keyframe file to be imported

  double vel  {0.3};
  double acc  {0.3};
  double dummy {0};
  

  // DEBUG: DELETE THESE


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // THE WHOLE BLOODY AFFAIR-----------------------------------------------------------------------------------------------------

  // ----------------------------
  // init: if this section is not used for initialization of the trajectoryMsg and the robotTrajectory struct,
  // the first generated plan happens to only include a single configuration for all trajectory points, and zero
  // velocities. On the second run of the main while loop, everything turns back to normal even without this
  // rock and stone initialization. but still..
  // assign 2 dummy configurations to the trajectory
  trajectoryMsg.joint_trajectory.points.push_back({});      
  trajectoryMsg.joint_trajectory.points[i].positions        = {0,0,0,0,0,0};
  trajectoryMsg.joint_trajectory.points.push_back({});
  trajectoryMsg.joint_trajectory.points[i].positions        = {0.1,0.1,0.1,0.1,0.1,0.1};
  // assign the two-poin to the struct
  robotTrajectory.setRobotTrajectoryMsg(*group.getCurrentState(), trajectoryMsg);
  // get the time parametrization done
  success = iptp.computeTimeStamps(robotTrajectory, vel, acc);
  // get the parametrizet trajectory into trajectoryMsg
  robotTrajectory.getRobotTrajectoryMsg(trajectoryMsg);
  // and eventually clear the trajectoryMsg and get the initialization finalized... bulls. 
  trajectoryMsg.joint_trajectory.points.clear();
  // ---------------------------- end of init:

  std::cout<<"\n\n\n"<<"Keyframe-based Cubic Spline Interpolation node for UR5 ('x' to shut down)"<<'\n';
  std::cout<<"------------------------------------------------------"<<'\n';

  std::cout<<"Enter velocity scaling factor... (enter 0 to use default value)"<<'\n';
  std::cin >> dummy;
  if( dummy != 0 ) vel = dummy;
  std::cout<<"Velocity scaling factor is set to: "<<vel<<'\n';
  
  std::cout<<"Enter acceleration scaling factor...(enter 0 to use default value)"<<'\n';
  std::cin >> dummy;
  if( dummy != 0 ) acc = dummy;
  std::cout<<"Acceleration scaling factor is set to: "<<acc<<'\n';

  while(ros::ok())
  {
    keyFrames.clear(); // Clear current keyframe register
    trajectoryMsg.joint_trajectory.points.clear();
    
    std::cout<<"Enter keyframe file name TO EXECUTE (without '.txt' extension)..."<<'\n';
    std::cout<<"Enter 's' to stop current animation"<<'\n';
    std::cin>>fileName;       //
    
    if(fileName == "x")
    {
      ros::shutdown();
      return 0;
    }
    else if(fileName == "s")
    {
      group.stop();
    }

    fileName.append(".txt"); 

    if(importKeyframes(keyFrames, fileName))
    {
      if(keyFrames.size() == 0) ROS_WARN_STREAM("Keyframe file has only one frame.");
      std::cout<<"Keyframes imported"<<'\n';

      interpolatedTrajectory = interpolateCubic(keyFrames);
      std::cout<<"Cubic interpolation complete"<<'\n';     

      for(i=0; i < interpolatedTrajectory.size(); i++)
      { 
        trajectoryMsg.joint_trajectory.points.push_back({});
        trajectoryMsg.joint_trajectory.points[i].positions        = interpolatedTrajectory[i];
      }

      robotTrajectory.setRobotTrajectoryMsg(*group.getCurrentState(), trajectoryMsg);
      success = iptp.computeTimeStamps(robotTrajectory, vel, acc);
      ROS_INFO("IPTP %s",success?"SUCCEDED":"FAILED");


   
      robotTrajectory.getRobotTrajectoryMsg(trajectoryMsg); 
      planCurrent.trajectory_ = trajectoryMsg;

      std::cout<<"Enter any character to execute plan, 's' to stop, 'x' to exit..."<<'\n';
        std::cin >> menu_1;

      if(menu_1 == 'x')
      {
        ros::shutdown();
        return 0;
      }
      else
        group.asyncExecute(planCurrent);
    }

  } 
  ros::shutdown();
  return 0;
}
