// Register JOINT SPACE keyframes and record them into the specified .txt file

#include <iostream>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
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
 
  char    menu_1    {'0'};  // menu selection characters
  char    menu_2    {'0'};  // menu selection characters
  std::string fileName;     // file name to export the keyframes into

  // std::vector<geometry_msgs::Pose>  keyFrames;
  std::vector<double>               pose;
  std::vector<std::vector<double>>  keyFrames;

  // geometry_msgs::Pose         robotPose;
  geometry_msgs::PoseStamped  robotPoseStamped;
  

  // DEBUG: DELETE THESE


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // THE WHOLE BLOODY AFFAIR-----------------------------------------------------------------------------------------------------

  std::cout<<"\n\n\n"<<"End-effector space keyframe registry"<<'\n';
  std::cout<<"------------------------------------"<<'\n';

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // TODO: IMPLEMENT THIS USING THE getConfirmedRobotState();
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  while(ros::ok() && menu_1 != 'x')
  {
    keyFrames.clear(); // Clear current keyframe register
    pose.clear();

    std::cout<<"\n\n\n"<<"Enter any character TO REGISTER the current POSE "<<'\n';
    std::cout<<"Enter 'x' to finish"<<'\n';
    
    std::cin>>menu_2;
    
    while(ros::ok() && menu_2 != 'x')
    {
      pose.clear();
      ros::spinOnce();
      sleep(0.5);
      
      robotPoseStamped = group.getCurrentPose();
      
      pose.push_back(robotPoseStamped.pose.position.x);
      pose.push_back(robotPoseStamped.pose.position.y);
      pose.push_back(robotPoseStamped.pose.position.z);

      pose.push_back(robotPoseStamped.pose.orientation.x);
      pose.push_back(robotPoseStamped.pose.orientation.y);
      pose.push_back(robotPoseStamped.pose.orientation.z);
      pose.push_back(robotPoseStamped.pose.orientation.w);
      
      keyFrames.push_back(pose);

      std::cout<<"Keyframe added"<<'\n';
      std::cin>>menu_2;
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
