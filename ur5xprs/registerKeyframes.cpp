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


// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SUBSCRIBER CALLBACKS
void JSTracker(const sensor_msgs::JointState& confMsg) 
{
  std::string dummyNam("");
  double dummyPos {0};
  double dummyVel {0};
  double dummyEff {0};

  robotJointState = confMsg;    // NORMALLY this line is all this callback is supposed to do but
  // simulation and actual robots publish the joint states in different orders
  // in simulation it is    0 1 2 3 4 5
  // in real it is          2 1 0 3 4 5
  // the if sentence below is meant to fix this only
  // TODO: assign the states of each joint by comparing each one's names. warn if any twists.

  if(robotJointState.name[0].compare("elbow_joint") == 0)
  {
    dummyNam = robotJointState.name[0];
    dummyPos = robotJointState.position[0];
    dummyVel = robotJointState.velocity[0];
    dummyEff = robotJointState.effort[0];

    robotJointState.name[0]   = robotJointState.name[2];
    robotJointState.position[0] = robotJointState.position[2];
    robotJointState.velocity[0] = robotJointState.velocity[2];
    robotJointState.effort[0]   = robotJointState.effort[2];

    robotJointState.name[2]   = dummyNam;
    robotJointState.position[2] = dummyPos;
    robotJointState.velocity[2] = dummyVel;
    robotJointState.effort[2]   = dummyEff;
  }
}

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

  // SUBSCRIBER/PUBLISHER DECLARATIONS------------------------------------------------------------------------------------------
  ros::Subscriber subJS           = nodeHandle.subscribe("/joint_states", 1, &JSTracker);
  
  // VARIABLE DECLARATIONS------------------------------------------------------------------------------------------------------
  moveit::planning_interface::MoveGroupInterface group("manipulator");           // setup a MoveGroup class to control and plan for
 
  char    menu_1    {'0'};  // menu selection characters
  char    menu_2    {'0'};  // menu selection characters
  std::string fileName;     // file name to export the keyframes into

  std::vector<std::vector<double>>  keyFrames;
  

  // DEBUG: DELETE THESE


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
      keyFrames.push_back(robotJointState.position);  // 
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
