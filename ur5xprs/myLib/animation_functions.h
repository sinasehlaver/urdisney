#include <fstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/move_group_interface/move_group.h>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

bool checkPose(robot_state::RobotStatePtr kinematic_state, const robot_state::JointModelGroup* joint_model_group, geometry_msgs::Pose &pose);

std::vector<std::string> splitString(std::string s, char delimiter);

void convertVectortoPose(std::vector<double> &poseV, geometry_msgs::Pose &pose);

void setVector(std::vector<double> &position, std::vector<double> &pose);

void convertPosetoKeyframe(robot_state::RobotStatePtr kinematic_state, const robot_state::JointModelGroup* joint_model_group, geometry_msgs::Pose &pose,std::vector<double> &keyFrame);

void convertStatetoPose(const Eigen::Affine3d &state, geometry_msgs::Pose &pose);

void convertKeyframetoRobotState(robot_state::RobotStatePtr kinematic_state, std::vector<double> &keyFrame);

void convertKeyframetoPose(robot_state::RobotStatePtr kinematic_state, const std::string &ee_link, std::vector<double> &keyFrame, geometry_msgs::Pose &pose ) ;

void convertStatetoVector(const Eigen::Affine3d &state, std::vector<double> &poseV );

void configToPose(moveit::planning_interface::MoveGroupInterface &group, robot_model::RobotModelPtr &kinematic_model, std::vector<double> &robotConf, geometry_msgs::Pose &pose );

void poseToConfig(moveit::planning_interface::MoveGroupInterface &group, robot_model::RobotModelPtr &kinematic_model, geometry_msgs::Pose &pose, std::vector<double> &robotConf);
