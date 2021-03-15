#include "animation_functions.h"


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

std::vector<std::string> splitString(std::string s, char delimiter){
  size_t last = 0; 
  size_t next = 0; 
  std::vector<std::string> splitted;
  while ((next = s.find(delimiter, last)) != std::string::npos) { 
    splitted.push_back(s.substr(last, next-last));  
    //std::cout << s.substr(last, next-last) << std::endl;
    last = next + 1; 
  } 
  splitted.push_back(s.substr(last));
  //std::cout << s.substr(last) << std::endl;
  return splitted;
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


void configToPose(moveit::planning_interface::MoveGroupInterface &group, robot_model::RobotModelPtr &kinematic_model, std::vector<double> &robotConf, geometry_msgs::Pose &pose ){
  std::vector<std::vector<double>>  keyFrames;
  geometry_msgs::PoseStamped  robotPoseStamped;

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  const std::vector<std::string> variables = kinematic_state->getVariableNames() ;

  for (int i = 0; i < variables.size(); ++i)
  { 
    kinematic_state->setVariablePosition(variables[i], robotConf[i]);
  }

  const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform(group.getEndEffectorLink());
  keyFrames.clear(); // Clear current keyframe register
  robotPoseStamped = group.getCurrentPose();
  convertStatetoPose(end_effector_state, pose);

}


void poseToConfig(moveit::planning_interface::MoveGroupInterface &group, robot_model::RobotModelPtr &kinematic_model, geometry_msgs::Pose &pose, std::vector<double> &robotConf){
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  const std::vector<std::string> variables = kinematic_state->getVariableNames();
  kinematic_state->enforceBounds();

  for (int i = 0; i < 2; ++i)
  {
      /* code */
    bool found_ik = kinematic_state->setFromIK(joint_model_group, pose, 10, 0.5);

    if(found_ik){

      if( kinematic_state->satisfiesBounds() ){
        ROS_INFO_STREAM("Current state is valid");
        kinematic_state->copyJointGroupPositions(joint_model_group, robotConf);
        for (int i = 0; i < joint_names.size(); ++i)
        {
          ROS_INFO("Joint %s: %f", joint_names[i].c_str(), robotConf[i]);
        }
        break;
      }
     
    }
    else{
      ROS_WARN_STREAM("Did not find IK solution");
    }
  }

}
