// Register JOINT SPACE keyframes and record them into the specified .txt file

#include <iostream>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/move_group_interface/move_group.h>
#include "functions.h"


// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLE DECLARATIONS


// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS


// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SUBSCRIBER CALLBACKS


// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // INITIALIZATIONS-----------------------------------------------------------------------------------------------------------
  ros::init(argc, argv, "ee_pose_based_keyframe_registry");
  ros::NodeHandle nodeHandle;
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::Rate rate(100);

  // SUBSCRIBER/PUBLISHER DECLARATIONS------------------------------------------------------------------------------------------
  // ros::Subscriber subJS           = nodeHandle.subscribe("/joint_states", 1, &JSTracker);
  
  // VARIABLE DECLARATIONS------------------------------------------------------------------------------------------------------
  moveit::planning_interface::MoveGroupInterface group("manipulator");           // setup a MoveGroup class to control and plan for
  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  
  double  fraction  {0};    // what fraction of the cartesian path is computed?
  bool    success   {0};    // boolean success tracker
  char    menu_1    {'0'};  // menu selection characters
  char    menu_2    {'0'};  // menu selection characters
  double  vel       {1};    // velocity scaling factor
  double  acc       {1};    // acceleration scaling factor
  std::string fileName;     // file name to export the keyframes into

   std::vector<geometry_msgs::Pose>  poses;
  std::vector<double>               keyFrame;
  std::vector<std::vector<double>>  keyFrames;
  std::vector<double>  robotConf;

  geometry_msgs::Pose         robotPose;
  geometry_msgs::PoseStamped  robotPoseStamped;
  moveit_msgs::RobotTrajectory trajectoryMsg;
  moveit::planning_interface::MoveGroupInterface::Plan planCurrent; 

  // DEBUG: DELETE THESE


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // THE WHOLE BLOODY AFFAIR-----------------------------------------------------------------------------------------------------

  std::cout<<"\n\n\n"<<"End-effector space keyframe registry"<<'\n';
  std::cout<<"------------------------------------"<<'\n';

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // TODO: IMPLEMENT THIS USING THE getConfirmedRobotState();
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  //INVERSE KINEMATICS

  while(ros::ok() && menu_1 != 'x')
  {
    
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    //kinematic_state->setToRandomPositions(joint_model_group);

    /* Print end-effector pose. Remember that this is in the model frame */


    std::cout<<"\n\n\n"<<"Enter filename to get robot pose"<<'\n';
    std::cout<<"Enter 'x' to finish"<<'\n';
    
    std::cin>>fileName;

    fileName.append(".txt");

    if( fileName == "x.txt" ){
      break;
    }


    if(importSinglePose(robotPose, fileName)){
      ROS_INFO_STREAM("Imported robot pose");
    }
    else{
      ROS_WARN_STREAM("Invalid file content");
      continue;
    }

    const std::vector<std::string> variables = kinematic_state->getVariableNames();

    //group.setPositionTarget(robotPose.pose.x, robotPose.pose.y, robotPose.pose.z, kinematic_state->getEndEffectorLink());
    kinematic_state->enforceBounds();

    for (int i = 0; i < 2; ++i)
    {
        /* code */
      bool found_ik = kinematic_state->setFromIK(joint_model_group, robotPose, 10, 0.5);

      if(found_ik){

        if( kinematic_state->satisfiesBounds() ){
          ROS_INFO_STREAM("Current state is valid");
          kinematic_state->copyJointGroupPositions(joint_model_group, keyFrame);
          for (int i = 0; i < joint_names.size(); ++i)
          {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), keyFrame[i]);
          }
          break;
        }
       
      }
      else{
        ROS_WARN_STREAM("Did not find IK solution");
      }
    }

    /*
    continue;

    poses.clear();
    poses.push_back(robotPose);


    fraction = group.computeCartesianPath(poses,
                                          0.1,  // eef_step
                                          0.0,   // jump_threshold
                                          trajectoryMsg, false);
    ROS_INFO("Visualizing plan for cartesian path (%.2f%% acheived)",fraction * 100.0);

    if( fraction != 1.0 ){
      throw 19;
    }

    //sleep(2);

    rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectoryMsg);
    success = iptp.computeTimeStamps(rt, vel, acc);
    ROS_INFO("Timestamp computation %s",success?"SUCCEDED":"FAILED");

    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectoryMsg);

    std::vector<int>::size_type size1 = trajectoryMsg.joint_trajectory.points.size();


    //std::cout<<"MSG: "<<trajectoryMsg<<"\n";

    for(auto j: trajectoryMsg.joint_trajectory.points[size1-1].positions){
      std::cout<<j<<'\t';
      keyFrame.push_back(j);
    }
    std::cout<<"\n";

    //trajectoryMsg.getLastWayPointPtr()->printStateInfo(std::cout);
    */
    std::cout<<"\n\n\n"<<"Enter a character to add the joint angle to the keyframes"<<'\n';
    std::cout<<"Enter 'x' to finish"<<'\n';
    
    std::cin>>menu_2;
    
    if(menu_2 != 'x')
    {
      keyFrames.clear();
      keyFrames.push_back(keyFrame);

      std::cout<<"Keyframe added"<<'\n';
    }

    if(keyFrames.size() != 0)
    {
      std::cout<<"Keyframe addition completed"<<'\n';
      std::cout<<"Enter file name to export..."<<'\n';
      std::cin >> fileName;
      fileName.append(".txt");

      exportFrames(keyFrames, fileName);
      std::cout<<"Keyframes exported"<<'\n';
    }

    std::cout<<"Enter any character to register new keyframes, 'x' to exit..."<<'\n';
        std::cin>>menu_1;
  } 
  ros::shutdown();
  return 0;
}
