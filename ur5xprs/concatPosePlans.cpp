// Calculates the cubic spline based joint space trajectory and generates the trajectory message based on IPTP
// TODO:  instead of using IPTP consider Parabolic Parameterization for generation of time stamps velocities etc.
//        IPTP is known to have bugs/glitches.

#include <iostream>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include "functions.h"

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLE DECLARATIONS


// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SUBSCRIBER CALLBACKS


bool importPoses(std::vector<geometry_msgs::Pose> &posesIn, std::string fileName)
{
  double buffer;
  int DoF {7};

  geometry_msgs::Pose pose;
  std::ifstream inputFile;

  inputFile.open(fileName);
  
  if (inputFile.is_open()) 
  {
    ROS_INFO_STREAM("Input file opened.");

    while (!inputFile.eof())
    {


      inputFile >> buffer ;

      if(inputFile.eof())
      {
        inputFile.close();
        ROS_INFO_STREAM("Input file closed.");
        if(posesIn.size() == 1) ROS_WARN_STREAM("Keyframe file has only one frame.");
        return 1;   // for success
      }

      ROS_INFO_STREAM("Pose began");

      pose.position.x = buffer;

      inputFile >> buffer ;
      pose.position.y = buffer;

      inputFile >> buffer ;
      pose.position.z = buffer;

      inputFile >> buffer ;
      pose.orientation.x = buffer;

      inputFile >> buffer ;
      pose.orientation.y = buffer;

      inputFile >> buffer ;
      pose.orientation.z = buffer;

      inputFile >> buffer ;
      pose.orientation.w = buffer;

      ROS_INFO_STREAM("Pose ended");


      posesIn.push_back(pose);
    }

  }
  else
  {
    ROS_WARN_STREAM("Cannot open input file... Check file name.");
    inputFile.close();
    return 0; // for failure
  }
}

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // INITIALIZATIONS-----------------------------------------------------------------------------------------------------------
  ros::init(argc, argv, "plan_concatenator");
  ros::NodeHandle nodeHandle;
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::Rate rate(100);

  // SUBSCRIBER/PUBLISHER DECLARATIONS------------------------------------------------------------------------------------------
  

  // VARIABLE DECLARATIONS------------------------------------------------------------------------------------------------------
  moveit::planning_interface::MoveGroupInterface group("manipulator");           // setup a MoveGroup class to control and plan for
  robot_trajectory::RobotTrajectory robotTrajectory(group.getCurrentState()->getRobotModel(), "manipulator");   // rt is used for IPTP
  
  std::vector<geometry_msgs::Pose>  poses;
  std::vector<geometry_msgs::Pose>  interpolatedTrajectory;
  moveit_msgs::RobotTrajectory      trajectoryMsg;       // Interpolated joint trajectory is inserted into this
                                    // trajectoryMsg is then inserted into robotTrajectory class using setRobotTrajectoryMsg(trajectoryMsg).
  
  trajectory_processing::IterativeParabolicTimeParameterization iptp;   // create a IterativeParabolicTimeParameterization object

  moveit::planning_interface::MoveGroupInterface::Plan planOverall;    // Time parametrized trajectory is then taken from rt and inserted into plan
  moveit::planning_interface::MoveGroupInterface::Plan planTemp;

  char    menu_1    {'0'};  // menu selection characters
  int     i         {0};    // counter
  int     j         {0};    // counter #2
  int     nof       {0};    // number of frames
  int     notrial   {0};    // number of trials
  bool    success   {0};    // boolean success tracker
  bool    execFlag  {0};    // controls the loop execution after the first commitment of group.execute()
  
  std::string fileName;     // name of the keyframe file to be imported
  std::vector<std::string> fileNames;     // array to store the keyframe files

  std::vector <double> scale;

  double vel  {0.3};
  double acc  {0.3};
  double eef_step  {0.1};
  double jump_threshold  {0.0};
  double sleep_time  {1.0};
  double dummy {0};
  double  fraction  {0}; 

  // DEBUG: DELETE THESE


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // THE WHOLE BLOODY AFFAIR-----------------------------------------------------------------------------------------------------

  // ----------------------------
  // init: if this section is not used for initialization of the trajectoryMsg and the robotTrajectory struct,
  // the first generated plan happens to only include a single configuration for all trajectory points, and zero
  // velocities. On the second run of the main while loop, everything turns back to normal even without this
  // rock and stone initialization. but still..
  // assign 2 dummy configurations to the trajectory
  trajectoryMsg.joint_trajectory.points.push_back({});      
  trajectoryMsg.joint_trajectory.points[i].positions        = {0,0,0,0,0,0};
  trajectoryMsg.joint_trajectory.points.push_back({});
  trajectoryMsg.joint_trajectory.points[i].positions        = {0.1,0.1,0.1,0.1,0.1,0.1};
  // assign the two-poin to the struct
  robotTrajectory.setRobotTrajectoryMsg(*group.getCurrentState(), trajectoryMsg);
  // get the time parametrization done
  success = iptp.computeTimeStamps(robotTrajectory, vel, acc);
  // get the parametrizet trajectory into trajectoryMsg
  robotTrajectory.getRobotTrajectoryMsg(trajectoryMsg);
  // and eventually clear the trajectoryMsg and get the initialization finalized... bulls. 
  trajectoryMsg.joint_trajectory.points.clear();
  // ---------------------------- end of init:

  std::cout<<"\n\n\n"<<"Plan concatenator for Cubic Spline Interpolation node ('x' to shut down)"<<'\n';
  std::cout<<"------------------------------------------------------"<<'\n';

  while(ros::ok())
  {
  
    // FIRST GET THE KEYFRAME FILE SEQUENCE
    while(fileName != "x.txt")
    {
      std::cout<<"Enter poses filename to CONCATENATE (without '.txt' extension)"<<'\n';
      std::cin>>fileName;       //
      fileName.append(".txt"); 
      if( fileName == "x.txt" ){
        ros::shutdown();
        return 0;
      }
      poses.clear();
      if(importPoses(poses, fileName))
      {
        std::cout<<"File content valid : Pose count "<<poses.size()<<std::endl;
        nof++;
        /*
        for (int i = 0; i < poses.size(); ++i)
        {
          std::cout<<"POSE\n"<<poses[i];
        }
        */
        break;
      }
    }

    // nof = i;
    //i = 0;
    std::cout<<"Enter maximum number of path trials for "<<fileName<<'\n';
    std::cin >> notrial;

    /*

    std::cout<<"Enter sleep duration between trials for "<<fileName<<'\n';
    std::cin >> sleep_time;

    std::cout<<"Enter maximum step size for "<<fileName<<'\n';
    std::cin >> eef_step;
*/
    std::cout<<"Enter maximum jump distance for "<<fileName<<'\n';
    std::cin >> jump_threshold;

    

    

    // NEXT GET THE VEL/ACC SCALING FACTORS FOR EACH KEYFRAME FILE
    std::cout<<"Enter vel scale factor for "<<fileName<<'\n';
    std::cin >> dummy;
    scale.push_back(dummy);
    std::cout<<"Vel scale factor set to "<<scale[0]<<'\n';

    std::cout<<"Enter acc scale factor for "<<fileName<<'\n';
    std::cin >> dummy;
    scale.push_back(dummy);
    std::cout<<"Acc scale factor set to "<<scale[1]<<'\n';     

    std::vector<geometry_msgs::Pose> paths;

    robotTrajectory.getRobotTrajectoryMsg(trajectoryMsg); 
    planOverall.trajectory_ = trajectoryMsg;

    for (int i = 0; i < poses.size(); ++i)
    {
      int maxFraction = 0.0;

      paths.clear();

      //if( i > 0 )
      //  paths.push_back(poses[i-1]);

      paths.push_back(poses[i]);

      /*

      if( i==0 ){
        planOverall.trajectory_ = trajectoryMsg;
      }

      group.setPositionTarget( poses[i].position.x, poses[i].position.y, poses[i].position.z, group.getEndEffectorLink());
      group.plan(planTemp);

      success = iptp.computeTimeStamps(robotTrajectory, scale[0], scale[1]);

      ROS_INFO("IPTP %s ",success?"SUCCEDED":"FAILED");

      planOverall = concatenatePlans(planOverall, planTemp);

      continue;
      */

      for (int j = 0; j < notrial; ++j)
      {
        ROS_INFO("COMPUTING PATH TRIAL NO %d", j+1);
        trajectoryMsg.joint_trajectory.points.clear();

        fraction = group.computeCartesianPath(paths,
                                                eef_step,  // eef_step
                                                jump_threshold,   // jump_threshold
                                                trajectoryMsg, false);
        ROS_INFO("Visualizing plan for cartesian path (%.2f%% acheived)",fraction * 100.0);
        sleep(sleep_time);
        if( fraction >= maxFraction ){
          maxFraction = fraction;
          robotTrajectory.setRobotTrajectoryMsg(*group.getCurrentState(), trajectoryMsg);
          planTemp.trajectory_ = trajectoryMsg;
        }

        if( fraction != 1.0 ){
          ROS_WARN_STREAM("FAILED TO COMPUTE A STRAIGHT LINE BETWEEN INITIAL AND END STATES");
          success = false;
        }
        else{
          ROS_INFO("PATH COMPUTED AT %dTH TRIAL",j+1);
          success = true;
          break;
        }
      }

      success = iptp.computeTimeStamps(robotTrajectory, scale[0], scale[1]);

      ROS_INFO("IPTP %s ",success?"SUCCEDED":"FAILED");

      if( i==0 ){
        planOverall.trajectory_ = trajectoryMsg;
      }

    }

    //planOverall = concatenatePlans(planOverall, planTemp);

    
    /*
    if(success == false)
      continue;

    */
     
  
    

    // initialize the overall plan, planOverall, with the first keyframe file
    //poses.clear(); // Clear current keyframe register
    
    //importPoses(poses, fileName);
    //interpolatedTrajectory = interpolatePoseCubic(poses);
    /*
    for(j=0; j < interpolatedTrajectory.size(); j++)
    { 
      trajectoryMsg.joint_trajectory.points.push_back({});
      trajectoryMsg.joint_trajectory.points[j].positions        = interpolatedTrajectory[j];
    }
    */
    
    

    //robotTrajectory.getRobotTrajectoryMsg(trajectoryMsg); 
    //planOverall.trajectory_ = trajectoryMsg;


    

    // then concatenate onto planOverall with the remaining keyframe files
    //std::cout<<"hi!\n";
    
    //poses.clear(); // Clear current keyframe register
    
    /*
    trajectoryMsg.joint_trajectory.points.clear();


    fraction = group.computeCartesianPath(poses,
                                            0.1,  // eef_step
                                            0.0,   // jump_threshold
                                            trajectoryMsg, false);
    robotTrajectory.setRobotTrajectoryMsg(*group.getCurrentState(), trajectoryMsg);


    success = iptp.computeTimeStamps(robotTrajectory, scale[0], scale[1]);
    //ROS_INFO("IPTP %s ",success?"SUCCEDED":"FAILED");

    */

    //robotTrajectory.getRobotTrajectoryMsg(trajectoryMsg); 
    //std::cout<<trajectoryMsg<<std::endl;
    //planOverall.trajectory_ = trajectoryMsg;

    /*
    std::cout<<"Enter "<<nof+1<<"th pose filename to CONCATENATE  (without '.txt' extension)"<<'\n';
    std::cin>>fileName;       //
    fileName.append(".txt"); 


    if(fileName != "x.txt"){

      if(importPoses(poses, fileName))
      {
        std::cout<<"File content valid : Pose count "<<poses.size()<<std::endl;

        for (int i = 0; i < poses.size(); ++i)
        {
          std::cout<<"POSE\n"<<poses[i];
        }
      }


      std::cout<<"processing "<<fileName<<'\n';
      importPoses(poses, fileName);
      std::cout<<"hi2!\n";
      //interpolatedTrajectory = interpolatePoseCubic(poses);
      
      //for(j=0; j < interpolatedTrajectory.size(); j++)
      //{ 
      //  trajectoryMsg.joint_trajectory.points.push_back({});
      //  trajectoryMsg.joint_trajectory.points[j].positions        = interpolatedTrajectory[j];
      //}
      
    fraction = group.computeCartesianPath(poses,
                                          0.1,  // eef_step
                                          0.0,   // jump_threshold
                                          trajectoryMsg, false);
    robotTrajectory.setRobotTrajectoryMsg(*group.getCurrentState(), trajectoryMsg);
    success = iptp.computeTimeStamps(robotTrajectory, scale[0], scale[1]);
    ROS_INFO("IPTP %s ",success?"SUCCEDED":"FAILED");

      robotTrajectory.getRobotTrajectoryMsg(trajectoryMsg); 
      std::cout<<trajectoryMsg<<std::endl;
      planTemp.trajectory_ = trajectoryMsg;

      planOverall = concatenatePlans(planOverall, planTemp);
    }

    */

    std::cout<<"Enter any character to execute plan, 'x' to exit..."<<'\n';
      std::cin >> menu_1;

    if(menu_1 == 'x')
    {
      ros::shutdown();
      return 0;
    }
    else
      group.execute(planOverall);
    
  } 
  ros::shutdown();
  return 0;
}
