// Register JOINT SPACE keyframes and record them into the specified .txt file

#include <iostream>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include "functions.h"

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLE DECLARATIONS
sensor_msgs::JointState   robotJointState;

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS

void setVector(std::vector<double> &position, std::vector<double> &pose){
  pose.clear();
  for (int i = 0; i < position.size(); ++i)
  {
    pose.push_back(position[i]);
  }
}
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SUBSCRIBER CALLBACKS

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // INITIALIZATIONS-----------------------------------------------------------------------------------------------------------
  ros::init(argc, argv, "keyframe_registry");
  ros::NodeHandle nodeHandle;
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::Rate rate(100);

  // VARIABLE DECLARATIONS------------------------------------------------------------------------------------------------------
  moveit::planning_interface::MoveGroupInterface gripper("gripper");           // setup a MoveGroup class to control and plan for
 
  char    menu_1    {'0'};  // menu selection characters
  char    menu_2    {'0'};  // menu selection characters
  std::string fileName;     // file name to export the keyframes into

  std::vector<double>  keyFrame;
  std::vector<double>  currentState;
  std::vector<std::vector<double>>  keyFrames;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // THE WHOLE BLOODY AFFAIR-----------------------------------------------------------------------------------------------------

  std::cout<<"\n\n\n"<<"Joint space keyframe registry"<<'\n';
  std::cout<<"-----------------------------"<<'\n';

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // TODO: IMPLEMENT THIS USING THE getConfirmedRobotState();
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  while(ros::ok() && menu_1 != 'x')
  {
    keyFrames.clear(); // Clear current keyframe register
    std::cout<<"\n\n\n"<<"Enter any character TO REGISTER the current config. as a keyframe"<<'\n';
    std::cout<<"Enter 'x' to finish"<<'\n';
      std::cin>>menu_2;
    
    while(ros::ok() && menu_2 != 'x')
    {
      ros::spinOnce();
      sleep(0.2);
      currentState = gripper.getCurrentJointValues();
      setVector(currentState, keyFrame);
      std::cout<<"Keyframe added"<<'\n';
      keyFrames.push_back(keyFrame);
      break;
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
