#include <iostream>
#include <fstream>
#include <mutex>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/move_group_interface/move_group.h>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "functions.h"
#include "animation_functions.h"
#include "std_msgs/String.h"

std::mutex lock;

std::string command;
std::string fileName;
std::string initialFileName;     
std::string endFileName;     
std::string concatFileName;     
double      vel       {0.3}; 
double      acc       {0.3}; 
char        menu_1    {'0'}; 
char        menu_2    {'0'};  
int         presence  {0};
bool        success   {0};    
sensor_msgs::JointState   robotJointState;


void getShootout(geometry_msgs::Pose &a, geometry_msgs::Pose &b, geometry_msgs::Pose &c){
  c.position.x = 2*b.position.x - a.position.x;
  c.position.y = 2*b.position.y - a.position.y;
  c.position.z = 2*b.position.z - a.position.z;
  c.orientation = b.orientation;
}


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

int extractPlan( std::vector<std::vector<std::string>> &complexPlan, std::string &fileName){
  std::string buffer;

  std::ifstream inputFile;

  inputFile.open(fileName);

  std::vector<std::string> row;
  complexPlan.clear();
  if (inputFile.is_open()) 
  {
    ROS_INFO_STREAM("complex file opened.");
    
    while (!inputFile.eof())
    {
      for(int i=0; i<6; i++)
      {    
        inputFile >> buffer ;
        if(inputFile.eof())
        {
          inputFile.close();
          if(complexPlan.size() == 1) ROS_WARN_STREAM("Plan file has only one frame.");
          return 1;   // for success
        }
        row.push_back(buffer);
      }
      complexPlan.push_back(row);
      row.clear();
    }

    ROS_INFO_STREAM("complex file closed.");

  }
  else
  {
    ROS_WARN_STREAM("Cannot open complex file... Check file name.");
    inputFile.close();
    ROS_INFO_STREAM("complex file closed.");
    return 0; // for failure
  }
}

int extractFileNames(std::string &fileName, std::vector<std::string> &fileNames, std::vector<std::vector <double> > &scales){//TODO
  int nof=0;
  std::vector<std::vector<double>>  keyFrames;
  if(fileName != "x.txt" && importKeyframes(keyFrames, fileName))
  {
    nof++;
    fileNames.push_back(fileName);
  }

  for(auto file:fileNames)
  {
    scales.push_back({});
    scales.back().push_back(0.3);
    std::cout<<"Vel scale factor set to "<<scales.back()[0]<<'\n';

    scales.back().push_back(0.3);
    std::cout<<"Acc scale factor set to "<<scales.back()[1]<<'\n';      
  }
  return nof;
}

void goToConfig(moveit::planning_interface::MoveGroupInterface &group, robot_model::RobotModelPtr &kinematic_model, std::string &file){
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  std::vector<double>  robotConf;
  robotConf.clear();
  moveit::planning_interface::MoveGroupInterface::Plan planOverall;
  if(importSingleFrame(robotConf, file))
  {
    std::cout<<"Configuration imported"<<'\n';
    group.setMaxVelocityScalingFactor(vel);
    group.setMaxAccelerationScalingFactor(acc);
    group.setJointValueTarget(robotConf);

    success = (group.plan(planOverall) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Planning for joint space goal) %s", success ? "SUCCEDED" : "FAILED");
    
    group.execute(planOverall);
  }
}

void goToConfig(moveit::planning_interface::MoveGroupInterface &group, robot_model::RobotModelPtr &kinematic_model, std::vector<double> &robotConf){
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  moveit::planning_interface::MoveGroupInterface::Plan planOverall;
  group.setMaxVelocityScalingFactor(vel);
  group.setMaxAccelerationScalingFactor(acc);
  group.setJointValueTarget(robotConf);

  success = (group.plan(planOverall) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Planning for joint space goal) %s", success ? "SUCCEDED" : "FAILED");
  
  group.execute(planOverall);
}

void goToPose(moveit::planning_interface::MoveGroupInterface &group, robot_model::RobotModelPtr &kinematic_model, std::string &file){
  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  moveit_msgs::RobotTrajectory trajectoryMsg;
  moveit::planning_interface::MoveGroup::Plan myMotionPlan;
  geometry_msgs::Pose         targetPose;
  std::vector< geometry_msgs::Pose > keyFrames;
  double  fraction    {0};    // what fraction of the cartesian path is computed?

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  keyFrames.clear(); // Clear current keyframe register

  if(importSinglePose(targetPose, file))
  {
    std::cout<<"Robot pose imported"<<'\n';
    
    if( checkPose( kinematic_state, joint_model_group, targetPose) ){

      keyFrames.push_back(targetPose);

      fraction = group.computeCartesianPath(keyFrames,
                                            0.1,   // eef_step
                                            0.0,   // jump_threshold
                                            trajectoryMsg, false);
      ROS_INFO("Visualizing plan for cartesian path (%.2f%% acheived)",fraction * 100.0);

      std::cout<<"initializing robotTrajectory based on the keyframe path.."<<"\n";
      rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectoryMsg);

      std::cout<<"computing time stamps"<<"\n";
      success = iptp.computeTimeStamps(rt, vel, acc);

      ROS_INFO("Timestamp computation %s",success?"SUCCEDED":"FAILED");

      rt.getRobotTrajectoryMsg(trajectoryMsg);

      myMotionPlan.trajectory_ = trajectoryMsg;
      group.execute(myMotionPlan);
    }
  }
}

void goToPose(moveit::planning_interface::MoveGroupInterface &group, robot_model::RobotModelPtr &kinematic_model, geometry_msgs::Pose &targetPose){
  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  moveit_msgs::RobotTrajectory trajectoryMsg;
  moveit::planning_interface::MoveGroup::Plan myMotionPlan;
  std::vector< geometry_msgs::Pose > keyFrames;
  double  fraction    {0};    // what fraction of the cartesian path is computed?

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  if( checkPose( kinematic_state, joint_model_group, targetPose) ){

    keyFrames.push_back(targetPose);

    fraction = group.computeCartesianPath(keyFrames,
                                          0.1,   // eef_step
                                          0.0,   // jump_threshold
                                          trajectoryMsg, false);
    ROS_INFO("Visualizing plan for cartesian path (%.2f%% acheived)",fraction * 100.0);

    std::cout<<"initializing robotTrajectory based on the keyframe path.."<<"\n";
    rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectoryMsg);

    std::cout<<"computing time stamps"<<"\n";
    success = iptp.computeTimeStamps(rt, vel, acc);

    ROS_INFO("Timestamp computation %s",success?"SUCCEDED":"FAILED");

    rt.getRobotTrajectoryMsg(trajectoryMsg);

    myMotionPlan.trajectory_ = trajectoryMsg;
    group.execute(myMotionPlan);
  }
}

moveit::planning_interface::MoveGroupInterface::Plan planKeyFrames(moveit::planning_interface::MoveGroupInterface &group, robot_model::RobotModelPtr &kinematic_model, std::vector<std::vector<double>> &keyFrames){
  robot_trajectory::RobotTrajectory robotTrajectory(group.getCurrentState()->getRobotModel(), "manipulator");   // rt is used for IPTP
      
  std::vector<std::vector<double>>  interpolatedTrajectory;
  moveit_msgs::RobotTrajectory      trajectoryMsg;       // Interpolated joint trajectory is inserted into this
  trajectory_processing::IterativeParabolicTimeParameterization iptp;   // create a IterativeParabolicTimeParameterization object
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  moveit::planning_interface::MoveGroupInterface::Plan planOverall;    // Time parametrized trajectory is then taken from rt and inserted into plan

  if( keyFrames.size() == 2 ){
    convertKeyframetoRobotState(kinematic_state, keyFrames[0]);

    group.setMaxVelocityScalingFactor(vel);
    group.setMaxAccelerationScalingFactor(acc);
    group.setStartState(*kinematic_state);
    group.setJointValueTarget(keyFrames[1]);

    success = (group.plan(planOverall) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Planning for joint space goal) %s", success ? "SUCCEDED" : "FAILED");
  }
  else if( keyFrames.size() > 2) {
    int     i         {0};    // counter
    int     j         {0};    // counter #2
    int     nof       {0};    // number of frames
    
    std::vector<std::vector <double> > scales;

    double dummy {0};
    
    trajectoryMsg.joint_trajectory.points.push_back({});      
    trajectoryMsg.joint_trajectory.points[i].positions        = {0,0,0,0,0,0};
    trajectoryMsg.joint_trajectory.points.push_back({});
    trajectoryMsg.joint_trajectory.points[i].positions        = {0.1,0.1,0.1,0.1,0.1,0.1};
    robotTrajectory.setRobotTrajectoryMsg(*group.getCurrentState(), trajectoryMsg);
    success = iptp.computeTimeStamps(robotTrajectory, vel, acc);
    robotTrajectory.getRobotTrajectoryMsg(trajectoryMsg);
    trajectoryMsg.joint_trajectory.points.clear();

    i = 0;

    trajectoryMsg.joint_trajectory.points.clear();
    
    interpolatedTrajectory = interpolateCubic(keyFrames);

    for(j=0; j < interpolatedTrajectory.size(); j++)
    { 
      trajectoryMsg.joint_trajectory.points.push_back({});
      trajectoryMsg.joint_trajectory.points[j].positions= interpolatedTrajectory[j];
    }

    robotTrajectory.setRobotTrajectoryMsg(*group.getCurrentState(), trajectoryMsg);
    success = iptp.computeTimeStamps(robotTrajectory, vel, acc);
    ROS_INFO("IPTP %s ",success?"SUCCEDED":"FAILED");

    robotTrajectory.getRobotTrajectoryMsg(trajectoryMsg); 
    planOverall.trajectory_ = trajectoryMsg;
  }
  group.setStartStateToCurrentState();
  return planOverall;
}

moveit::planning_interface::MoveGroupInterface::Plan applyArcs( moveit::planning_interface::MoveGroupInterface &group, robot_model::RobotModelPtr &kinematic_model){
  if( presence == 1){
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    
    std::vector<double>  initKeyframe;
    std::vector<double>  endKeyframe;
    moveit::planning_interface::MoveGroupInterface::Plan planOverall;    // Time parametrized trajectory is then taken from rt and inserted into plan
    
    importSingleFrame(initKeyframe, initialFileName);
    importSingleFrame(endKeyframe, endFileName);

    convertKeyframetoRobotState(kinematic_state, initKeyframe);

    group.setMaxVelocityScalingFactor(vel);
    group.setMaxAccelerationScalingFactor(acc);
    group.setStartState(*kinematic_state);
    group.setJointValueTarget(endKeyframe);

    success = (group.plan(planOverall) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Planning for joint space goal) %s", success ? "SUCCEDED" : "FAILED");
    group.setStartStateToCurrentState();
    return planOverall;
  }
  else{
    robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    moveit_msgs::RobotTrajectory trajectoryMsg;
    moveit::planning_interface::MoveGroup::Plan myMotionPlan;

    geometry_msgs::Pose         initPose;
    geometry_msgs::Pose         endPose;

    std::vector<double>  initKeyframe;
    std::vector<double>  endKeyframe;

    std::vector< geometry_msgs::Pose > keyFrames;
    double  fraction    {0};    // what fraction of the cartesian path is computed?

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    keyFrames.clear(); // Clear current keyframe register

    if(importSingleFrame(initKeyframe, initialFileName))
    {
      std::cout<<"Robot init frame imported"<<'\n';

      configToPose(group, kinematic_model, initKeyframe, initPose);

      if( checkPose( kinematic_state, joint_model_group, initPose) ){
        keyFrames.push_back(initPose);

        if(importSingleFrame(endKeyframe, endFileName))
        {
          std::cout<<"Robot end frame imported"<<'\n';

          configToPose(group, kinematic_model, endKeyframe, endPose);

          if( checkPose( kinematic_state, joint_model_group, endPose) ){
            keyFrames.push_back(endPose);

            fraction = group.computeCartesianPath(keyFrames,
                                                  0.1,   // eef_step
                                                  0.0,   // jump_threshold
                                                  trajectoryMsg, false);
            ROS_INFO("Visualizing plan for cartesian path (%.2f%% acheived)",fraction * 100.0);

            std::cout<<"initializing robotTrajectory based on the keyframe path.."<<"\n";
            rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectoryMsg);

            std::cout<<"computing time stamps"<<"\n";
            success = iptp.computeTimeStamps(rt, vel, acc);

            ROS_INFO("Timestamp computation %s",success?"SUCCEDED":"FAILED");

            rt.getRobotTrajectoryMsg(trajectoryMsg);

            myMotionPlan.trajectory_ = trajectoryMsg;
            return myMotionPlan;
          }
        }
      }
    }
  }
}

moveit::planning_interface::MoveGroupInterface::Plan applySISO( moveit::planning_interface::MoveGroupInterface &group, robot_model::RobotModelPtr &kinematic_model){
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  
  std::vector<double>  initKeyframe;
  std::vector<double>  endKeyframe;
  moveit::planning_interface::MoveGroupInterface::Plan planOverall;    // Time parametrized trajectory is then taken from rt and inserted into plan
  
  importSingleFrame(initKeyframe, initialFileName);
  importSingleFrame(endKeyframe, endFileName);

  convertKeyframetoRobotState(kinematic_state, initKeyframe);

  group.setMaxVelocityScalingFactor(0.6);//TODO DELETE
  ROS_INFO("ACC set to %f", (11-presence)/11.0);
  group.setMaxAccelerationScalingFactor((11-presence)/11.0);
  group.setStartState(*kinematic_state);
  group.setJointValueTarget(endKeyframe);

  success = (group.plan(planOverall) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Planning for joint space goal) %s", success ? "SUCCEDED" : "FAILED");
  group.setStartStateToCurrentState();
  return planOverall;
}

moveit::planning_interface::MoveGroupInterface::Plan applyDrag( moveit::planning_interface::MoveGroupInterface &group, robot_model::RobotModelPtr &kinematic_model){
  geometry_msgs::Pose  inPose;
  geometry_msgs::Pose  endPose;
  geometry_msgs::Pose  outPose;

  int dragPower;

  std::vector<std::vector<double>>  keyFrames;
  std::vector<double>  initKeyframe;
  std::vector<double>  endKeyframe;
  std::vector<double>  inKeyframe;
  std::vector<double>  outKeyframe;

  moveit::planning_interface::MoveGroupInterface::Plan planCurrent;

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  if(importSingleFrame(initKeyframe, initialFileName)){
    ROS_INFO_STREAM("Imported initial state");
  }
  keyFrames.push_back(initKeyframe);

  if(importSingleFrame(endKeyframe, endFileName)){
    ROS_INFO_STREAM("Imported end state");
  }

  keyFrames.push_back(endKeyframe);

  if(presence != 0){

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

    dragPower = (presence/10.0)*(size1/2);
    ROS_INFO("drag power: %d", dragPower);

    setVector(planCurrent.trajectory_.joint_trajectory.points[size1-dragPower-1].positions, inKeyframe);

    convertKeyframetoPose(kinematic_state, group.getEndEffectorLink(), inKeyframe, inPose);

    if(!checkPose(kinematic_state, joint_model_group, inPose)){//CHECK IF SHOOT IN IS A VALID POSE
      ROS_WARN_STREAM("SHOOT IN STATE IS NOT A REACHABLE POSE");      
      throw 19;
    }

    getShootout(inPose, endPose, outPose);// task space shootout calculation

    if(!checkPose(kinematic_state, joint_model_group, outPose)){//CHECK IF SHOOT OUT IS A VALID POSE
      ROS_WARN_STREAM("SHOOT OUT STATE IS NOT A REACHABLE POSE");      
      throw 19;
    } 

    convertPosetoKeyframe(kinematic_state, joint_model_group, outPose, outKeyframe);

    keyFrames.push_back(outKeyframe);
    keyFrames.push_back(endKeyframe);
  }

  return planKeyFrames( group, kinematic_model, keyFrames);
}

void msgCallback(const std_msgs::String::ConstPtr& msg)//TODO parse the message and set the global variables accordingly
{
  lock.lock();
  std::string msgStr = msg->data;
  std::vector<std::string> args = splitString(msgStr, ' ');

  if (args[0] == "go_to_config" || args[0] == "go_to_pose"){
      command = args[0];
      fileName = args[1];
  }
  else if (args[0] == "save_config" || args[0] == "save_pose"){
      command = args[0];
      fileName = args[1];
  }
  else if ( args[0] == "plan_basic"){
    command = args[0];
    initialFileName = args[1];
    endFileName = args[2];
  }
  else if ( args[0] == "principle_arcs" || args[0] == "principle_drag" || args[0] == "principle_siso"){
    command = args[0];
    initialFileName = args[1];
    endFileName = args[2];
    presence = std::stoi(args[3]);
  }
  else if (args[0] == "concat_config_plans" || args[0] == "complex_plan" ){
      command = args[0];
      concatFileName = args[1];
  }
  else{
    command = "idle";
    fileName = "";
    initialFileName = "";
    endFileName = "";
    concatFileName = "";
  }
  lock.unlock();

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "animation_principle_planner_node");
  ros::NodeHandle nodeHandle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate rate(100);

  moveit::planning_interface::MoveGroupInterface group("manipulator");          
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  moveit::planning_interface::MoveGroupInterface::Plan planCurrent;
  ros::Subscriber sub = nodeHandle.subscribe("principle_planner_command", 1000, msgCallback);
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  ros::Subscriber subJS = nodeHandle.subscribe("/joint_states", 1, &JSTracker);

  std::cout<<"\n\n\n"<<"Animation Principle Planner Framework"<<'\n';
  std::cout<<"------------------------------------"<<'\n';

  while(ros::ok() && menu_1 != 'x')//MAIN LOOP
  {
    
    //std::cin>>menu_1;
    lock.lock();

    try{
      if( command == "a"){
        std::cout<<"Action done\n";
        command = 'b';
      }
      else if( command == "go_to_config" ){ // GO TO CONFIG
        goToConfig(group, kinematic_model, fileName);
      }
      else if( command == "go_to_pose" ){ // GO TO POSE
        goToPose(group, kinematic_model, fileName);
      }
      else if( command == "save_config"){ // SAVE CONFIG
        std::vector<std::vector<double>>  keyFrames;

        keyFrames.clear(); // Clear current keyframe register
        keyFrames.push_back(robotJointState.position);  // 
        std::cout<<"Keyframe added"<<'\n';

        if(keyFrames.size() != 0)
        {
          std::cout<<"Keyframe addition completed"<<'\n';
          exportFrames(keyFrames, fileName);
          std::cout<<"Keyframes exported"<<'\n';
        }
      }
      else if( command == "save_pose"){ // SAVE POSE
        std::vector<double>               pose;
        std::vector<std::vector<double>>  keyFrames;
        geometry_msgs::PoseStamped  robotPoseStamped;
        robotPoseStamped = group.getCurrentPose();
        
        pose.push_back(robotPoseStamped.pose.position.x);
        pose.push_back(robotPoseStamped.pose.position.y);
        pose.push_back(robotPoseStamped.pose.position.z);

        pose.push_back(robotPoseStamped.pose.orientation.x);
        pose.push_back(robotPoseStamped.pose.orientation.y);
        pose.push_back(robotPoseStamped.pose.orientation.z);
        pose.push_back(robotPoseStamped.pose.orientation.w);
        
        keyFrames.push_back(pose);

        if(keyFrames.size() != 0)
        {
          std::cout<<"Keyframe addition completed"<<'\n';
          exportFrames(keyFrames, fileName);
          std::cout<<"Keyframes exported"<<'\n';
        }
      }
      else if( command == "principle_arcs"){ // ARCS PRINCIPLE
        goToConfig(group, kinematic_model, initialFileName);
        ROS_INFO("READY FOR EXECUTING THE ORIGINAL PLAN");
        planCurrent = applyArcs(group, kinematic_model);
        sleep(2);
        group.execute(planCurrent);
      }
      else if( command == "principle_siso"){ // SISO PRINCIPLE
        goToConfig(group, kinematic_model, initialFileName);
        ROS_INFO("READY FOR EXECUTING THE ORIGINAL PLAN");
        planCurrent = applySISO(group, kinematic_model);
        sleep(2);
        group.execute(planCurrent);
      }
      else if( command == "principle_drag"){ // DRAG PRINCIPLE
        goToConfig(group, kinematic_model, initialFileName);
        planCurrent = applyDrag(group, kinematic_model);
        sleep(2);
        group.execute(planCurrent);
      }
      else if( command == "plan_basic" ){
        std::vector<double>  initKeyframe;
        std::vector<double>  endKeyframe;
        std::vector<std::vector<double>>  keyFrames;
        if(importSingleFrame(initKeyframe, initialFileName)){
          ROS_INFO_STREAM("Imported initial state");
        }
        keyFrames.push_back(initKeyframe);

        goToConfig(group, kinematic_model, initKeyframe);

        if(importSingleFrame(endKeyframe, endFileName)){
          ROS_INFO_STREAM("Imported end state");
        }

        keyFrames.push_back(endKeyframe);

        planCurrent = planKeyFrames(group, kinematic_model,keyFrames);
        sleep(2);
        group.execute(planCurrent);
      }
      else if( command == "concat_config_plans"){ // CONCAT CONFIG
        std::vector<std::vector<double>>  keyFrames;
        importKeyframes(keyFrames, concatFileName);
        goToConfig(group, kinematic_model, keyFrames[0]);
        planCurrent = planKeyFrames(group, kinematic_model, keyFrames);
        sleep(2);
        group.execute(planCurrent);
      }
      else if( command == "complex_plan" ){ // COMPLEX PLAN
        moveit::planning_interface::MoveGroupInterface::Plan planOverall;   
        std::vector<double>  initKeyframe;
        std::vector<double>  endKeyframe; 
        std::vector<double>  prevKeyframe; 
        std::vector<std::vector<std::string> > complexPlan;
        std::vector<std::vector<double>>  keyFrames;
        std::vector<std::vector<double>>  dummy;
        
        extractPlan(complexPlan, concatFileName);
        int flag = 1;
        for (int i = 0; i < complexPlan.size(); ++i)
        {
          if( complexPlan[i][0] == "1" ) {
            initialFileName = complexPlan[i][1];

            initKeyframe.clear();

            if(importSingleFrame(initKeyframe, initialFileName)){
              ROS_INFO_STREAM("Imported initial state");
            }

            endFileName = complexPlan[i][2];

            endKeyframe.clear();

            if(importSingleFrame(endKeyframe, endFileName)){
              ROS_INFO_STREAM("Imported end state");
            }

            if( prevKeyframe.size() != 0){
              ROS_INFO_STREAM("Connecting rows");

              for (int i = 0; i < prevKeyframe.size(); ++i)
              {
                std::cout<<prevKeyframe[i]<<" ";
              }
              std::cout<<"\n";

              keyFrames.clear();
              keyFrames.push_back(prevKeyframe);
              keyFrames.push_back(initKeyframe);
              planCurrent = planKeyFrames(group, kinematic_model, keyFrames);
              planOverall = concatenatePlans(planOverall,planCurrent);
            }

            prevKeyframe.clear();
            prevKeyframe = endKeyframe;

            for (int i = 0; i < prevKeyframe.size(); ++i)
            {
              std::cout<<prevKeyframe[i]<<" ";
            }
            std::cout<<"\n";

            ROS_INFO_STREAM("Saved pre");

            if( flag == 1 ){
              flag = 0;
              goToConfig(group, kinematic_model, initKeyframe);

              if( complexPlan[i][4] == "1" ){
                presence = std::stoi(complexPlan[i][5]);

                if( complexPlan[i][3] == "drag" ){
                  planOverall = applyDrag(group, kinematic_model);
                }
                else if ( complexPlan[i][3] == "siso" ){
                  planOverall = applySISO(group, kinematic_model);
                }
                else if ( complexPlan[i][3] == "arcs" ){
                  presence = 1;
                  planOverall = applyArcs(group, kinematic_model);
                }
              }
              else{
                if( complexPlan[i][3] == "arcs" ){
                  ROS_INFO_STREAM("no arcs");
                  presence = 0;
                  planOverall = applyArcs(group, kinematic_model);
                }
                else{
                  ROS_INFO_STREAM("arcs");

                  keyFrames.clear();
                  keyFrames.push_back(initKeyframe);
                  keyFrames.push_back(endKeyframe);
                  planOverall = planKeyFrames(group, kinematic_model, keyFrames);
                  ROS_INFO_STREAM("planned");
                  
                }
              }
              ROS_INFO_STREAM("principle done");
            }
            else{
              if( complexPlan[i][4] == "1" ){
                presence = std::stoi(complexPlan[i][5]);

                if( complexPlan[i][3] == "drag" ){
                  planCurrent = applyDrag(group, kinematic_model);
                }
                else if ( complexPlan[i][3] == "siso" ){
                  planCurrent = applySISO(group, kinematic_model);
                }
                else if ( complexPlan[i][3] == "arcs" ){
                  presence = 1;
                  planCurrent = applyArcs(group, kinematic_model);
                }
              }
              else{
                if( complexPlan[i][3] == "arcs" ){
                  ROS_INFO_STREAM("no arcs");
                  presence = 0;
                  planCurrent = applyArcs(group, kinematic_model);
                }
                else{
                  ROS_INFO_STREAM("arcs");

                  keyFrames.clear();
                  keyFrames.push_back(initKeyframe);
                  keyFrames.push_back(endKeyframe);
                  planCurrent = planKeyFrames(group, kinematic_model, keyFrames);
                  ROS_INFO_STREAM("planned");
                  
                }
              }
              ROS_INFO_STREAM("principle done");

              planOverall = concatenatePlans(planOverall,planCurrent);
              ROS_INFO_STREAM("principle added to overall");
            }
          }
        }
        group.execute(planOverall);
      }
    }
    catch (...){
      ROS_WARN_STREAM("ERROR ENCOUNTERED");
    }
    command = "idle";
    vel = 0.3;
    acc = 0.3;
    presence = 0;
    lock.unlock();   
    rate.sleep();
    
  } 
  ros::shutdown();
  return 0;
}
