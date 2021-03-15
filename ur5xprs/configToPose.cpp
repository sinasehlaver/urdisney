// Register JOINT SPACE keyframes and record them into the specified .txt file

#include <iostream>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "functions.h"

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLE DECLARATIONS

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS


geometry_msgs::Quaternion toQuaternion( const Eigen::Matrix<double, 3, 3> a ) {
  geometry_msgs::Quaternion q;

  if ( 1 + a(0,0) + a(1,1) + a(2,2) > 0 )
    q.w = sqrt( 1 + a(0,0) + a(1,1) + a(2,2)) / 2;

  if ( 1 + a(0,0) - a(1,1) - a(2,2) > 0 )
    q.x = sqrt( 1 + a(0,0) - a(1,1) - a(2,2)) / 2;

  if ( 1 - a(0,0) + a(1,1) - a(2,2) > 0 )
    q.y = sqrt( 1 - a(0,0) + a(1,1) - a(2,2)) / 2;

  if ( 1 - a(0,0) - a(1,1) + a(2,2) > 0 )
    q.z = sqrt( 1 - a(0,0) - a(1,1) + a(2,2)) / 2;

  if( a(2,1) - a(1,2) < 0 )
    q.x = -q.x;

  if( a(0,2) - a(2,0) < 0 )
    q.y = -q.y;

  if( a(1,0) - a(0,1) < 0 )
    q.z = -q.z;

  return q;
}

geometry_msgs::Quaternion CalculateRotation( const Eigen::Matrix<double, 3, 3> a ) {
  geometry_msgs::Quaternion q;
  double trace = a(0,0) + a(1,1) + a(2,2);
  if( trace > 0 ) {// I changed M_EPSILON to 0
    double s = 0.5 / sqrt(trace+ 1.0);
    q.w = 0.25 / s;
    q.x = ( a(2,1) - a(1,2) ) * s;
    q.y = ( a(0,2) - a(2,0) ) * s;
    q.z = ( a(1,0) - a(0,1) ) * s;
  } else {
    if ( a(0,0) > a(1,1) && a(0,0) > a(2,2) ) {
      double s = 2.0 * sqrt( 1.0 + a(0,0) - a(1,1) - a(2,2));
      q.w = (a(2,1) - a(1,2) ) / s;
      q.x = 0.25 * s;
      q.y = (a(0,1) + a(1,0) ) / s;
      q.z = (a(0,2) + a(2,0) ) / s;
    } else if (a(1,1) > a(2,2)) {
      double s = 2.0 * sqrt( 1.0f + a(1,1) - a(0,0) - a(2,2));
      q.w = (a(0,2) - a(2,0) ) / s;
      q.x = (a(0,1) + a(1,0) ) / s;
      q.y = 0.25 * s;
      q.z = (a(1,2) + a(2,1) ) / s;
    } else {
      double s = 2.0 * sqrt( 1.0 + a(2,2) - a(0,0) - a(1,1) );
      q.w = (a(1,0) - a(0,1) ) / s;
      q.x = (a(0,2) + a(2,0) ) / s;
      q.y = (a(1,2) + a(2,1) ) / s;
      q.z = 0.25 * s;
    }
  }

  if ( q.w < 0 ){
    q.x = -q.x;
    q.y = -q.y;
    q.z = -q.z;
    q.w = -q.w;
  }

  return q;
}

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
  std::vector<double>  robotConf;

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

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  

  while(ros::ok() && menu_1 != 'x')
  {
    
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    //kinematic_state->setToRandomPositions(joint_model_group);

    /* Print end-effector pose. Remember that this is in the model frame */


    std::cout<<"\n\n\n"<<"Enter filename to get coordinate"<<'\n';
    std::cout<<"Enter 'x' to finish"<<'\n';
    
    std::cin>>fileName;

    fileName.append(".txt");

    if( fileName == "x.txt" ){
      break;
    }


    if(importSingleFrame(robotConf, fileName)){
      ROS_INFO_STREAM("Imported joint state");
    }
    else{
      ROS_WARN_STREAM("Invalid file content");
      continue;
    }

    const std::vector<std::string> variables = kinematic_state->getVariableNames() ;


    for (int i = 0; i < variables.size(); ++i)
    { 
      kinematic_state->setVariablePosition(variables[i], robotConf[i]);
      //ROS_INFO_STREAM("variable: "<<i<<" "<<variables[i]<<" value: "<< kinematic_state->getVariablePosition(variables[i]) <<"\n");
    }

    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform(group.getEndEffectorLink());

    //Eigen::Vector3f ea = end_effector_state.eulerAngles(0,1,2);
    //Eigen::Quaterniond q(end_effector_state.rotation());

    //ROS_INFO_STREAM("Quaternion x: \n" << ea << "\n");
    //ROS_INFO_STREAM("Quaternion y: \n" << q.y << "\n");
    //ROS_INFO_STREAM("Quaternion z: \n" << q.z << "\n");
    //ROS_INFO_STREAM("Quaternion w: \n" << q.w << "\n");




    //ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation().x() << "\n");
    //ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

    keyFrames.clear(); // Clear current keyframe register
    pose.clear();

    std::cout<<"\n\n\n"<<"Enter any character TO REGISTER the current POSE "<<'\n';
    std::cout<<"Enter 'x' to finish"<<'\n';
    
    std::cin>>menu_2;
    
    if(menu_2 != 'x')
    {
      pose.clear();
      
      robotPoseStamped = group.getCurrentPose();
      
      pose.push_back(end_effector_state.translation().x());
      pose.push_back(end_effector_state.translation().y());
      pose.push_back(end_effector_state.translation().z());

      

      if(false){
        geometry_msgs::Quaternion q = toQuaternion(end_effector_state.rotation());
        std::cout<<"QUATERNION "<<q.x<<" "<<q.y<<" "<<q.z<<" "<<q.w<<"\n";

        pose.push_back(q.x);
        pose.push_back(q.y);
        pose.push_back(q.z);
        pose.push_back(q.w);
      }
      else{
        Eigen::Quaterniond qx = Eigen::Quaterniond(end_effector_state.rotation());

        std::cout<<"QUATERNION "<<qx.x()<<" "<<qx.y()<<" "<<qx.z()<<" "<<qx.w()<<"\n";

        pose.push_back(qx.x());
        pose.push_back(qx.y());
        pose.push_back(qx.z());
        pose.push_back(qx.w());
      }
      
      keyFrames.push_back(pose);

      std::cout<<"Keyframe added"<<'\n';
    }

    if(keyFrames.size() != 0)
    {
      std::cout<<"Keyframe addition completed"<<'\n';
      std::cout<<"Enter file name to export..."<<'\n';
      std::cin >> fileName;

      if(fileName != "x"){
        fileName.append(".txt");

        exportFrames(keyFrames, fileName);
        std::cout<<"Keyframes exported"<<'\n';
      }
    }

    std::cout<<"Enter any character to register new keyframes, 'x' to exit..."<<'\n';
        std::cin>>menu_1;
  } 
  ros::shutdown();
  return 0;
}
