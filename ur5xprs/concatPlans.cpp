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
  ros::init(argc, argv, "plan_concatenator");
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

  moveit::planning_interface::MoveGroupInterface::Plan planOverall;    // Time parametrized trajectory is then taken from rt and inserted into plan
  moveit::planning_interface::MoveGroupInterface::Plan planTemp;

  char    menu_1    {'0'};  // menu selection characters
  int     i         {0};    // counter
  int     j         {0};    // counter #2
  int     nof       {0};    // number of frames
  bool    success   {0};    // boolean success tracker
  bool    execFlag  {0};    // controls the loop execution after the first commitment of group.execute()
  
  std::string fileName;     // name of the keyframe file to be imported
  std::vector<std::string> fileNames;     // array to store the keyframe files

  std::vector<std::vector <double> > scales;

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

  std::cout<<"\n\n\n"<<"Plan concatenator for Cubic Spline Interpolation node ('x' to shut down)"<<'\n';
  std::cout<<"------------------------------------------------------"<<'\n';

  while(ros::ok())
  {
  
    // FIRST GET THE KEYFRAME FILE SEQUENCE
    while(fileName != "x.txt" && !execFlag)
    {
      std::cout<<"Enter "<<nof+1<<"th keyframe filename to CONCATENATE  (without '.txt' extension)"<<'\n';
      std::cin>>fileName;       //
      fileName.append(".txt"); 

      if(fileName != "x.txt" && importKeyframes(keyFrames, fileName))
      {
        nof++;
        fileNames.push_back(fileName);
      }
    }

    std::cout<<"Keyframe file addition for concatenation complete. "<<'\n';
    std::cout<<fileNames.size()<<" files in total."<<'\n';
    fileName = "";
    
    // nof = i;
    i = 0;

    // NEXT GET THE VEL/ACC SCALING FACTORS FOR EACH KEYFRAME FILE
    if(!execFlag)
    {
      for(auto file:fileNames)
      {
        scales.push_back({});
        std::cout<<"Enter vel scale factor for "<<file<<'\n';
        std::cin >> dummy;
        scales.back().push_back(dummy);
        std::cout<<"Vel scale factor set to "<<scales.back()[0]<<'\n';

        std::cout<<"Enter acc scale factor for "<<file<<'\n';
        std::cin >> dummy;
        scales.back().push_back(dummy);
        std::cout<<"Acc scale factor set to "<<scales.back()[1]<<'\n';      
      }
    }
    

    // initialize the overall plan, planOverall, with the first keyframe file
    keyFrames.clear(); // Clear current keyframe register
    trajectoryMsg.joint_trajectory.points.clear();
    
    importKeyframes(keyFrames, fileNames[0]);
    interpolatedTrajectory = interpolateCubic(keyFrames);

    for(j=0; j < interpolatedTrajectory.size(); j++)
    { 
      trajectoryMsg.joint_trajectory.points.push_back({});
      trajectoryMsg.joint_trajectory.points[j].positions= interpolatedTrajectory[j];
    }
    robotTrajectory.setRobotTrajectoryMsg(*group.getCurrentState(), trajectoryMsg);
    success = iptp.computeTimeStamps(robotTrajectory, scales[0][0], scales[0][1]);
    ROS_INFO("IPTP %s ",success?"SUCCEDED":"FAILED");

    robotTrajectory.getRobotTrajectoryMsg(trajectoryMsg); 
    planOverall.trajectory_ = trajectoryMsg;

    // then concatenate onto planOverall with the remaining keyframe files
    for(i = 1; i<nof; i++)
    {
      std::cout<<"hi!\n";
      
      keyFrames.clear(); // Clear current keyframe register
      trajectoryMsg.joint_trajectory.points.clear();
      
      std::cout<<"processing "<<fileNames[i]<<'\n';
      importKeyframes(keyFrames, fileNames[i]);
      std::cout<<"hi2!\n";
      interpolatedTrajectory = interpolateCubic(keyFrames);

      for(j=0; j < interpolatedTrajectory.size(); j++)
      { 
        trajectoryMsg.joint_trajectory.points.push_back({});
        trajectoryMsg.joint_trajectory.points[j].positions        = interpolatedTrajectory[j];
      }
      robotTrajectory.setRobotTrajectoryMsg(*group.getCurrentState(), trajectoryMsg);
      success = iptp.computeTimeStamps(robotTrajectory, scales[i][0], scales[i][1]);
      ROS_INFO("IPTP %s ",success?"SUCCEDED":"FAILED");

      robotTrajectory.getRobotTrajectoryMsg(trajectoryMsg); 
      planTemp.trajectory_ = trajectoryMsg;

      planOverall = concatenatePlans(planOverall, planTemp);

    }

    std::cout<<"Enter any character to execute plan, 'x' to exit..."<<'\n';
      std::cin >> menu_1;

    if(menu_1 == 'x')
    {
      ros::shutdown();
      return 0;
    }
    else
      group.execute(planOverall);
      execFlag = 1;
    
  } 
  ros::shutdown();
  return 0;
}
