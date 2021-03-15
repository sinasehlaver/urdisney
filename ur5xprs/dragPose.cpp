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

void getShootout(geometry_msgs::Pose &a, geometry_msgs::Pose &b, geometry_msgs::Pose &c){

  c.position.x = 2*b.position.x - a.position.x;
  c.position.y = 2*b.position.y - a.position.y;
  c.position.z = 2*b.position.z - a.position.z;

  c.orientation = b.orientation;

  /*

  c.orientation.x = 2*b.orientation.x - a.orientation.x;
  c.orientation.y = 2*b.orientation.y - a.orientation.y;
  c.orientation.z = 2*b.orientation.z - a.orientation.z;
  c.orientation.w = 2*b.orientation.w - a.orientation.w;

  /*
  for (int i = 0; i < a.size(); ++i)
  {
    c[i] = 2*b[i] - a[i];
  }
  */
}

void convertVectortoPose(std::vector<double> &poseV, geometry_msgs::Pose &pose){
  if( poseV.size() == 7 ){
    pose.position.x = poseV[0];
    pose.position.y = poseV[1];
    pose.position.z = poseV[2];
    pose.orientation.x = poseV[3];
    pose.orientation.y = poseV[4];
    pose.orientation.z = poseV[5];
    pose.orientation.w = poseV[6];
  }
  else{
    ROS_WARN_STREAM("Not a valid vector");
  }

}

void setVector(std::vector<double> &position, std::vector<double> &pose){
  pose.clear();
  for (int i = 0; i < position.size(); ++i)
  {
    pose.push_back(position[i]);
  }
}

bool checkPose(robot_state::RobotStatePtr kinematic_state, const robot_state::JointModelGroup* joint_model_group, geometry_msgs::Pose &pose){
  kinematic_state->enforceBounds();
  for (int i = 0; i < 2; ++i)
  {
    bool found_ik = kinematic_state->setFromIK(joint_model_group, pose, 10, 0.5);
    if(found_ik){

      if( kinematic_state->satisfiesBounds() ){
        ROS_INFO_STREAM("Current state is valid");
        return true;
        break;
      }
    }
    else{
      ROS_WARN_STREAM("Did not find IK solution");
    }
  }
  return false;
}

void convertPosetoKeyframe(robot_state::RobotStatePtr kinematic_state, const robot_state::JointModelGroup* joint_model_group, geometry_msgs::Pose &pose,std::vector<double> &keyFrame){
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  kinematic_state->enforceBounds();
  for (int i = 0; i < 2; ++i)
  {
    bool found_ik = kinematic_state->setFromIK(joint_model_group, pose, 10, 0.5);
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
}

void convertStatetoPose(const Eigen::Affine3d &state, geometry_msgs::Pose &pose){
    pose.position.x = state.translation().x();
    pose.position.y = state.translation().y();
    pose.position.z = state.translation().z();
    Eigen::Quaterniond qx = Eigen::Quaterniond(state.rotation());
    std::cout<<"QUATERNION "<<qx.x()<<" "<<qx.y()<<" "<<qx.z()<<" "<<qx.w()<<"\n";
    pose.orientation.x = qx.x();
    pose.orientation.y = qx.y();
    pose.orientation.z = qx.z();
    pose.orientation.w = qx.w();
}

void convertKeyframetoRobotState(robot_state::RobotStatePtr kinematic_state, std::vector<double> &keyFrame){
    const std::vector<std::string> variables = kinematic_state->getVariableNames() ;

    for (int i = 0; i < variables.size(); ++i)
    { 
      kinematic_state->setVariablePosition(variables[i], keyFrame[i]);
    }

}

void convertKeyframetoPose(robot_state::RobotStatePtr kinematic_state, const std::string &ee_link, std::vector<double> &keyFrame, geometry_msgs::Pose &pose ) {
    
    convertKeyframetoRobotState(kinematic_state, keyFrame);

    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform(ee_link);

    convertStatetoPose(end_effector_state, pose);
}



void convertStatetoVector(const Eigen::Affine3d &state, std::vector<double> &poseV ){
    poseV.clear();
    poseV.push_back(state.translation().x());
    poseV.push_back(state.translation().y());
    poseV.push_back(state.translation().z());
    Eigen::Quaterniond qx = Eigen::Quaterniond(state.rotation());
    std::cout<<"QUATERNION "<<qx.x()<<" "<<qx.y()<<" "<<qx.z()<<" "<<qx.w()<<"\n";
    poseV.push_back(qx.x());
    poseV.push_back(qx.y());
    poseV.push_back(qx.z());
    poseV.push_back(qx.w());
}

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
  moveit::planning_interface::MoveGroupInterface::Plan planCurrent;
  char    menu_1    {'0'};  // menu selection characters
  char    menu_2    {'0'};  // menu selection characters
  int     dragPower {2};
  double  vel       {0.3};    // velocity scaling factor
  double  acc       {0.3};    // acceleration scaling factor
  std::string fileName;     // file name to export the keyframes into

  // std::vector<geometry_msgs::Pose>  keyFrames;
  geometry_msgs::Pose  inPose;
  geometry_msgs::Pose  endPose;
  geometry_msgs::Pose  outPose;

  std::vector<std::vector<double>>  keyFrames;
  std::vector<double>  initKeyframe;
  std::vector<double>  endKeyframe;
  std::vector<double>  inKeyframe;
  std::vector<double>  outKeyframe;

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


    std::cout<<"\n\n\n"<<"Enter initial state filename"<<'\n';
    std::cout<<"Enter 'x' to finish"<<'\n';
    
    std::cin>>fileName;

    fileName.append(".txt");

    if( fileName == "x.txt" ){
      break;
    }


    if(importSingleFrame(initKeyframe, fileName)){
      ROS_INFO_STREAM("Imported initial state");
    }
    else{
      ROS_WARN_STREAM("Invalid file content");
      throw 19;
    }

    keyFrames.push_back(initKeyframe);

    std::cout<<"\n\n\n"<<"Enter end state filename"<<'\n';
    std::cout<<"Enter 'x' to finish"<<'\n';
    
    std::cin>>fileName;

    fileName.append(".txt");

    if( fileName == "x.txt" ){
      break;
    }

    if(importSingleFrame(endKeyframe, fileName)){
      ROS_INFO_STREAM("Imported end state");
    }
    else{
      ROS_WARN_STREAM("Invalid file content");
      throw 19;
    }

    keyFrames.push_back(endKeyframe);

    convertKeyframetoPose(kinematic_state, group.getEndEffectorLink(), endKeyframe, endPose);

    if(!checkPose(kinematic_state, joint_model_group, endPose)){//CHECK IF END IS A VALID POSE
      ROS_WARN_STREAM("END STATE IS NOT A REACHABLE POSE");      
      throw 19;
    } 

    convertKeyframetoRobotState(kinematic_state, initKeyframe);
    group.setMaxVelocityScalingFactor(vel);
    group.setMaxAccelerationScalingFactor(acc);
    group.setStartState(*kinematic_state);
    group.setJointValueTarget(endKeyframe);

    bool success = (group.plan(planCurrent) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Planning for joint space goal) %s", success ? "SUCCEDED" : "FAILED");

    std::vector<int>::size_type size1 = planCurrent.trajectory_.joint_trajectory.points.size();

    

    /*

    std::vector<std::vector<double>> interpolatedTrajectory = interpolateCubic(keyFrames);
    */

    std::cout<<"Enter drag intensity"<<'\n';
    std::cin>>dragPower;

    if(size1 + 1 <= dragPower){
      dragPower = 1;
    }

    setVector(planCurrent.trajectory_.joint_trajectory.points[size1-dragPower-1].positions, inKeyframe);

    convertKeyframetoPose(kinematic_state, group.getEndEffectorLink(), inKeyframe, inPose);

    if(!checkPose(kinematic_state, joint_model_group, inPose)){//CHECK IF SHOOT IN IS A VALID POSE
      ROS_WARN_STREAM("SHOOT IN STATE IS NOT A REACHABLE POSE");      
      throw 19;
    }

    keyFrames.clear(); // Clear current keyframe register
    keyFrames.push_back(inKeyframe);
    exportFrames(keyFrames, "shootin_pose.txt");

    getShootout(inPose, endPose, outPose);// task space shootout calculation

    if(!checkPose(kinematic_state, joint_model_group, outPose)){//CHECK IF SHOOT OUT IS A VALID POSE
      ROS_WARN_STREAM("SHOOT OUT STATE IS NOT A REACHABLE POSE");      
      throw 19;
    } 

    convertPosetoKeyframe(kinematic_state, joint_model_group, outPose, outKeyframe);

    keyFrames.clear(); // Clear current keyframe register
    keyFrames.push_back(outKeyframe);

    std::cout<<"Keyframe added"<<'\n';

    if(keyFrames.size() != 0)
    {
      std::cout<<"Enter file name to export shootout ..."<<'\n';
      std::cin >> fileName;

      if(fileName != "x"){
        fileName.append(".txt");

        exportFrames(keyFrames, fileName);
        std::cout<<"Keyframes exported"<<'\n';
      }
    }

  } 
  ros::shutdown();
  return 0;
}
