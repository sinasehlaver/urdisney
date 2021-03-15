// Provides a similar functionality with goToConfig.cpp. However, instead of using group.setJointValueTarget(),
// and group.plan(), this node utilizes group.computeCartesianPath() to generate a linearly interpolated trajectory 
// in the end-effector space. This way I hope to end up with 'more robot-like' motions.

// Moves the robot to a given configuration. The configuration is imported from a .txt file (a keyframe file).
// only the first line of the file is taken to consideration.

#include <iostream>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

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
  ros::init(argc, argv, "go_from_pose_to_pose");
  ros::NodeHandle nodeHandle;
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::Rate rate(100);

  // SUBSCRIBER/PUBLISHER DECLARATIONS------------------------------------------------------------------------------------------
  

  // VARIABLE DECLARATIONS------------------------------------------------------------------------------------------------------
  moveit::planning_interface::MoveGroupInterface group("manipulator");           // setup a MoveGroup class to control and plan for
  

  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");


  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  // std::vector<double>  robotConf;

  moveit_msgs::RobotTrajectory trajectoryMsg;

  moveit::planning_interface::MoveGroup::Plan myMotionPlan;



  moveit_msgs::Constraints test_constraints;
  moveit_msgs::OrientationConstraint ocm;

  // geometry_msgs::PoseStamped  stampedPose;
  geometry_msgs::Pose         targetPose;
  // geometry_msgs::Pose         currentPose;
  
  std::vector< geometry_msgs::Pose > keyFrames;
  
  moveit::planning_interface::MoveGroupInterface::Plan planCurrent;    // Time parametrized trajectory is then taken from rt and inserted into plan

  char    menu_1    {'0'};  // menu selection characters
  bool    success   {0};    // boolean success tracker

  double  fraction    {0};    // what fraction of the cartesian path is computed?
  double  constraint_x  {0};
  double  constraint_y  {0};
  double  constraint_z  {0};

  double  vel         {1};    // velocity scaling factor
  double  acc         {1};    // acceleration scaling factor
  
  std::string fileName;     // name of the keyframe file to be imported
  

  // DEBUG: DELETE THESE


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // THE WHOLE BLOODY AFFAIR-----------------------------------------------------------------------------------------------------

  std::cout<<"\n\n\n"<<"Plan to a target pose imported from a file ('x' to shut down)"<<'\n';
  std::cout<<"Plan is computed by moveIt! using moveGroup.computeCartesianPath() method."<<'\n';
  std::cout<<"--------------------------------------------------------------------------"<<'\n';

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  while(ros::ok())
  {
    // robotConf.clear(); // Clear current robot configuration register

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    

    keyFrames.clear(); // Clear current keyframe register

    std::cout<<"Enter pose file name TO GO (without '.txt' extension)..."<<'\n';
    std::cin>>fileName;       //
    
    if(fileName == "x")
    {
      ros::shutdown();
      return 0;
    }

    fileName.append(".txt"); 

    if(importSinglePose(targetPose, fileName))
    {
      std::cout<<"Robot pose imported"<<'\n';
      
      // sleep(0.3);
      // ros::spinOnce();
      
      // stampedPose = group.getCurrentPose();

      // currentPose.position.x = stampedPose.pose.position.x;
      // currentPose.position.y = stampedPose.pose.position.y;
      // currentPose.position.z = stampedPose.pose.position.z;

      // currentPose.orientation.x = stampedPose.pose.orientation.x;
      // currentPose.orientation.y = stampedPose.pose.orientation.y;
      // currentPose.orientation.z = stampedPose.pose.orientation.z;
      // currentPose.orientation.w = stampedPose.pose.orientation.w;

      // keyFrames.push_back(currentPose); // THIS CAUSES MOVEIT TO RETURN THE ERROR:Trajectory message contains waypoints that are not strictly increasing in time.
      
      /*
      std::cout<<"Enter orientation constraint factor for x axis for "<<fileName<<'\n';
      std::cin >> constraint_x;

      std::cout<<"Enter orientation constraint factor for y axis for "<<fileName<<'\n';
      std::cin >> constraint_y;

      std::cout<<"Enter orientation constraint factor for z axis for "<<fileName<<'\n';
      std::cin >> constraint_z;
      */
      //moveit_msgs::OrientationConstraint ocm;
      /*
      for (int i = 0; i < joint_names.size(); ++i)
      {
        std::cout<<i<<" "<<joint_names[i]<<std::endl;
      }
      */
      /*
      ocm.link_name = joint_names[1];
      ocm.header.frame_id = "base_link";
      ocm.orientation.w = 1.0;
      ocm.absolute_x_axis_tolerance = 3.6;
      ocm.absolute_y_axis_tolerance = 1.8;
      ocm.absolute_z_axis_tolerance = 3.6;
      ocm.weight = 1.0;

      moveit_msgs::Constraints test_constraints;
      test_constraints.orientation_constraints.push_back(ocm);
      group.setPathConstraints(test_constraints);
      */

      //group.setPositionTarget( targetPose.position.x, targetPose.position.y, targetPose.position.z, group.getEndEffectorLink());
      //group.plan(planCurrent);
      
      //continue;

      keyFrames.push_back(targetPose);


      fraction = group.computeCartesianPath(keyFrames,
                                            0.1,   // eef_step
                                            0.0,   // jump_threshold
                                            trajectoryMsg, false);
      ROS_INFO("Visualizing plan for cartesian path (%.2f%% acheived)",fraction * 100.0);

      // sleep(2);

      std::cout<<"initializing robotTrajectory based on the keyframe path.."<<"\n";
      // Second get a RobotTrajectory from trajectory
      rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectoryMsg);

      std::cout<<"computing time stamps"<<"\n";
      // Fourth compute computeTimeStamps

      success = iptp.computeTimeStamps(rt, vel, acc);
      // success = iptp.computeTimeStamps(rt, 0.25, 0.25);

      ROS_INFO("Timestamp computation %s",success?"SUCCEDED":"FAILED");

      // Get RobotTrajectory_msg from RobotTrajectory
      rt.getRobotTrajectoryMsg(trajectoryMsg);

      // Finally plan and execute the trajectory
      myMotionPlan.trajectory_ = trajectoryMsg;

      
      std::cout<<"Enter any character to execute plan, 'x' to exit..."<<'\n';
        std::cin >> menu_1;

      if(menu_1 == 'x')
      {
        ros::shutdown();
        return 0;
      }
      else
        group.execute(myMotionPlan);
    }

  } 
  ros::shutdown();
  return 0;
}
