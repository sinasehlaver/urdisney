#include "functions.h"

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS

// description: returns current joint values after confirming them
// can be used to get the confirmed robot state and also to poll for the confirmation of the joint states
std::vector<double> getConfirmedJointValues(moveit::planning_interface::MoveGroupInterface &targetGroup)
{
  std::vector<double> jointStates;
  std::vector<double> pastJointStates;
  double maxDiff = 0;
  double diffThres = 0.001;
  int minIterations = 5;
  int maxIterations = 250;
  int j = 0;

  pastJointStates = targetGroup.getCurrentJointValues();
  jointStates = targetGroup.getCurrentJointValues();

  while(ros::ok() && j < maxIterations && (maxDiff > diffThres || j < minIterations) )
  {
    maxDiff = 0;
    // std::cout<<"Can't confirm robot joint state. Iteration: "<< j+1 <<'\n';
    // std::cout<<"maxDiff "<<maxDiff<<'\n';

    pastJointStates = jointStates;
    // jointStates = jointStates = targetGroup.getCurrentJointValues(); // noticed this on 29 jul 2019. i dont know why is this like this.
    jointStates = targetGroup.getCurrentJointValues();
    
    for( int i=0; i < jointStates.size(); i++)
    {
      if( fabs(jointStates[i] - pastJointStates[i]) > maxDiff )maxDiff = fabs(jointStates[i] - pastJointStates[i]);
      // std::cout<<j<<"- past joint "<< pastJointStates[i] <<" cur coint "<<jointStates[i]<<'\n';
  
    }
    // std::cout<<"maxDiff "<<maxDiff<<'\n';
    j++;
  }

  // // dump joint values (debug only)-------
  // for(int i=0; i < jointStates.size(); i++)
  // {
  //   std::cout<<jointStates[i]<<'\t';
  // }
  // std::cout<<'\n';
  // // -------------------------------------

  if(j<maxIterations)
    std::cout<<"Robot joint state confirmed at iteration: "<< j+1 <<'\n';
  else std::cout<<"CANNOT CONFIRM JOINT STATE!!! Iteration: "<< j+1 <<'\n';
  return jointStates;
}

// description: Imports JS keyframes from file.
bool importKeyframes(std::vector<std::vector<double>>  &framesIn, std::string fileName)
{
  double buffer;
  int DoF {6};

  std::vector<double> frame;
  std::ifstream inputFile;

  inputFile.open(fileName);
  
  if (inputFile.is_open()) 
  {
    while (!inputFile.eof())
    {
      for(int i=0; i<DoF; i++)
      {    
        inputFile >> buffer ;
        if(inputFile.eof())
        {
          inputFile.close();
          if(framesIn.size() == 1) ROS_WARN_STREAM("Keyframe file has only one frame.");
          return 1;   // for success
        }
        frame.push_back(buffer);
      }
      framesIn.push_back(frame);
      frame.clear();
    }
  }
  else
  {
    ROS_WARN_STREAM("Cannot open input file... Check file name.");
    inputFile.close();
    return 0; // for failure
  }
}

// description: Imports a single JS keyframe from file.
bool importSingleFrame(std::vector<double>  &configuration, std::string fileName)
{
  double buffer;
  int DoF {6};

  std::ifstream inputFile;

  inputFile.open(fileName);
  
  if (inputFile.is_open()) 
  {
    ROS_INFO_STREAM("Input file opened.");
    for(int i=0; i<DoF; i++)
    {    
      inputFile >> buffer ;
      configuration.push_back(buffer);
    }
    inputFile.close();
    ROS_INFO_STREAM("Input file closed.");
    return 1; // for success
  }
  else
  {
    ROS_WARN_STREAM("Cannot open input file... Check file name.");
    inputFile.close();
    ROS_INFO_STREAM("Input file closed.");
    return 0; // for failure
  }
}


// description: Imports a single robot pose from file to geometry_msgs::Pose.
bool importSinglePose(geometry_msgs::Pose &robotPose, std::string fileName)
{
  double buffer;

  std::ifstream inputFile;

  inputFile.open(fileName);
  
  if (inputFile.is_open()) 
  {
    ROS_INFO_STREAM("Input file opened.");

    inputFile >> buffer ;
    robotPose.position.x = buffer;

    inputFile >> buffer ;
    robotPose.position.y = buffer;

    inputFile >> buffer ;
    robotPose.position.z = buffer;

    inputFile >> buffer ;
    robotPose.orientation.x = buffer;

    inputFile >> buffer ;
    robotPose.orientation.y = buffer;

    inputFile >> buffer ;
    robotPose.orientation.z = buffer;

    inputFile >> buffer ;
    robotPose.orientation.w = buffer;

    inputFile.close();
    ROS_INFO_STREAM("Input file closed.");
    return 1; // for success
  }
  else
  {
    ROS_WARN_STREAM("Cannot open input file... Check file name.");
    inputFile.close();
    ROS_INFO_STREAM("Input file closed.");
    return 0; // for failure
  }
}

// description: Exports joint space (JS) keyframes into file. Each row is a keyframe; each column is a DOF.
// The file is has one extra empty line with at the end.
void exportFrames(std::vector<std::vector<double>> framesOut, std::string fileName)
{
  std::ofstream outputFile;

  outputFile.open (fileName);
  for(auto i:framesOut)
  {
    for(auto j:i)outputFile  << j << '\t';
    outputFile << '\n';
  }
  outputFile.close();
}

// description: exports a vector into a file with the fileName
void exportVector(std::vector<double> vector, std::string fileName)
{
  std::ofstream outputFile;

  outputFile.open (fileName);
  for(auto i:vector)
  {
    outputFile  << i << '\t';
    outputFile << '\n';
  }
  outputFile.close();
}

// description: exports a section of a RobotTrajectory into a file with the fileName. the section choice is input with the argument exportType.
void exportTrajectory(moveit_msgs::RobotTrajectory trajectoryOut, char exportType, std::string fileName)
{
  std::ofstream outputFile;

  double buffer {-1};

  outputFile.open (fileName);
  switch(exportType)
  {
    case 'p': // positions
      for(auto i:trajectoryOut.joint_trajectory.points)
      {
        for(auto j:i.positions)outputFile  << j << '\t';
        outputFile << '\n';
      }
      break;

    case 'v':   // velocities
      for(auto i:trajectoryOut.joint_trajectory.points)
      {
        for(auto j:i.velocities)outputFile  << j << '\t';
        outputFile << '\n';
      }
      break;

    case 't':   // time stamps
      for(auto i:trajectoryOut.joint_trajectory.points)
      {
        buffer = double(i.time_from_start.sec) + double(i.time_from_start.nsec/double(1000000000));
        outputFile  << buffer << '\t';
        outputFile << '\n';
      }
      break;

    default:
      ROS_WARN_STREAM("Invalid argument for exportTrajectory!");
      break;
  }
  outputFile.close();
}

// description: Interpolates line segments in joint space through keyframes with fixed step size in y-axis 
std::vector<std::vector<double>> interpolateLinear(std::vector<std::vector<double>> frames, float maxStepSize)
{
  std::vector<std::vector<double>> interpolation;
  std::vector<double>   buffer;
  std::vector<double> stepSize;
  double confDiffMax {0};
  int nofFrames {0};
  int nofSubDiv {0};
  int DoF {6};
  
  nofFrames = frames.size();
  std::cout<<"Number of frames to interpolate: "<<nofFrames<<'\n';
  //
  //
  for(int i = 0; i < nofFrames-1; i++)  // for each sement
  {
    confDiffMax = 0;
    stepSize.clear();
    buffer.clear();

    for(int j = 0; j < DoF; j++)      // check the maximum configuration difference at each dof
    {
      if( fabs(frames[i][j]-frames[i+1][j]) > fabs(confDiffMax))
      {
        confDiffMax = frames[i][j]-frames[i+1][j];
        std::cout<<"For segment "<<i<<" max diff is in DoF "<<j<<" and is "<<confDiffMax<<'\n';
      }
    }
    
    nofSubDiv = ceil( fabs(confDiffMax/maxStepSize) );  // find nofSubDiv
    std::cout<<"Number of subdivisions for segment "<<i<<" is "<<nofSubDiv<<'\n';

    // std::cout<<"Steps for each dof in segment "<<i<<" are "<<'\n';
    for(int j = 0; j < DoF; j++)
    {
      stepSize.push_back((frames[i+1][j]-frames[i][j])/nofSubDiv);             // find the step size
      // std::cout<<(frames[i+1][j]-frames[i][j])/nofSubDiv<<'n';
    }

    for(int k = 0; k < nofSubDiv; k++)
    {
      for(int j = 0; j < DoF; j++)
      {
        buffer.push_back(frames[i][j]+ k*stepSize[j]);  
      }
      interpolation.push_back(buffer);
      buffer.clear();
    }
  }

  interpolation.push_back(frames[nofFrames-1]);
  
  std::cout<<"Overall size of the interpolation is "<<interpolation.size()<<'\n';
  return interpolation;
}

// description: Interpolates a cubic spline through the keyframes in joint space for all degrees of freedom.
// note: The step size is currently fixed in the time (x-) axis. It better be fixed with a maximum change for the y-axis.
// note2: Instead of using fixed step sizes, interpolation can be made denser when necessary; and more sparse if the trajectory is mostly linear. 
// note3: Using a very small stepSize (eg. 0.002) for x-axis causes a lot of glitches in the velocities generated by the IPTP.
// keeping it larger reduces these glitches.
std::vector<std::vector<double>> interpolateCubic(std::vector<std::vector<double>> frames)
{
  const int DoF {6};
  // const double stepSize = 0.002;
  // const double stepSize = 0.1;
  const double stepSize = 0.075;

  int nofFrames {0};    // number of frames

  std::vector<std::vector<double>>  interpolation;  // result of the interpolation
  std::vector<double>               buffer;         // buffer used to fill out the overall interpolation
  std::vector<tk::spline>           cubicSplines(DoF);   // vector the hold the DoF many spline interpolations

  std::vector<std::vector<double>>  rowFrames(DoF);   // keyframe argument (frames) is inverted into this as column to row
  std::vector<double> timeAxis;

  for(auto frame:frames)
  {
    for(int i = 0; i < DoF; i++)
    {
      rowFrames[i].push_back(frame[i]);
    }
  }

  while(nofFrames < frames.size())
  {
    timeAxis.push_back(nofFrames);
    nofFrames++;
  }
  
  for(int i = 0; i < DoF; i++)
  {
    cubicSplines[i].set_points(timeAxis, rowFrames[i]);
  }

  for(double d = 0; d < nofFrames-1-stepSize; d+=stepSize)
  {  
    for(int i = 0; i < DoF; i++)
    {
      buffer.push_back(cubicSplines[i](d));
    }
    interpolation.push_back(buffer);
    buffer.clear();
  }
  interpolation.push_back(frames[nofFrames-1]);
  
  return interpolation;
}

// THIS IS ABSOLUTELY USELESS. ABORT ABORT! the interpolation is already scaled by IPTP/ISTP.
// go with interpolateCubicV2 and try to keep the step size fixed in terms of configuration difference, not time/index difference.
// description: The same core with the interpolateCubic() but the x-axis is scaled with respect to the maximum configuration difference
// between two consecutive frames. The non-scaled version uses index numbers of the frames (1, 2, 3, ...) as x-axis.
// The maximum possible difference, 4*pi, is mapped to deltaX = 1 between two frames.
// note: what happens to the minimum difference then? better check it at some point.
std::vector<std::vector<double>> interpolateCubicScaled(std::vector<std::vector<double>> frames)
{
  const int DoF {6};
  // const double stepSize = 0.002;
  const double stepSize {0.1};
  // const double stepSize = 0.075;
  
  int nofFrames {0};    // number of frames
  
  double deltaX {0};   // calculated x-axis difference for the spline segment in scope
  std::vector<double> deltaConf {};

  std::vector<std::vector<double>>  interpolation;  // result of the interpolation
  std::vector<double>               buffer;         // buffer used to fill out the overall interpolation
  std::vector<tk::spline>           cubicSplines(DoF);   // vector the hold the DoF many spline interpolations

  std::vector<std::vector<double>>  rowFrames(DoF);   // keyframe argument (frames) is inverted into this as column to row
  std::vector<double> timeAxis;

  // each component of the frames is a configuration. we need vector components as consecutive configurations for
  // each DoF instead of each robot configuration. this block does that.
  for(auto frame:frames)
  {
    for(int i = 0; i < DoF; i++)
    {
      rowFrames[i].push_back(frame[i]);
    }
  }

  nofFrames = frames.size();

  timeAxis.push_back(0);    // time axis starts from 0

  for(int i = 0; i < nofFrames - 1; i++)
  {
    // deltaConf = frames(i)-frames(i+1);
    // std::transform(frames(i).begin(),frames(i).end(),frames(i+1).begin(),deltaConf.begin(),std::minus<int>());
    // deltaX = *std::max_element(deltaConf.begin(), deltaConf.end());
    for(int j = 0; j < DoF; j++)
    {
      if( fabs( frames[i][j]-frames[i+1][j]) > fabs(deltaX) )
        deltaX = fabs(frames[i][j] - frames[i+1][j]);
    }

    std::cout << "Max element is " << deltaX << '\n';
    std::cout << "Time difference is " << deltaX/(4*3.14159) << '\n';

    timeAxis.push_back(timeAxis[i] + 10*deltaX/(4*3.14159));
    deltaConf.clear();
    deltaX = 0;
  }

  for(int i = 0; i < nofFrames; i++)
  {
    std::cout<<timeAxis[i]<<'\n';
  }
  

  for(int i = 0; i < DoF; i++)
  {
    cubicSplines[i].set_points(timeAxis, rowFrames[i]);
  }

  for(double d = 0; d < timeAxis[nofFrames-1] - stepSize; d+=stepSize)
  {  
    for(int i = 0; i < DoF; i++)
    {
      buffer.push_back(cubicSplines[i](d));
    }
    interpolation.push_back(buffer);
    buffer.clear();
  }
  interpolation.push_back(frames[nofFrames-1]);
  
  return interpolation;
}

// TODO: description
std::vector<std::vector<double>> interpolateCubicV2(std::vector<std::vector<double>> frames)
{
  const int DoF {6};
  const double stepSize = 0.1;
  int nofFrames {0};    // number of frames

  std::vector<std::vector<double>>  interpolation;  // result of the interpolation
  std::vector<double>               buffer;         // buffer used to fill out the overall interpolation
  std::vector<tk::spline>           cubicSplines(DoF);   // vector the hold the DoF many spline interpolations

  std::vector<std::vector<double>>  rowFrames(DoF);   // keyframe argument (frames) is inverted into this as column to row
  std::vector<double> timeAxis;

  for(auto frame:frames)
  {
    for(int i = 0; i < DoF; i++)
    {
      rowFrames[i].push_back(frame[i]);
    }
  }

  while(nofFrames < frames.size())
  {
    timeAxis.push_back(nofFrames);
    nofFrames++;
  }
  
  for(int i = 0; i < DoF; i++)
  {
    cubicSplines[i].set_points(timeAxis, rowFrames[i]);
  }

  for(double d = 0; d < nofFrames-1-stepSize; d+=stepSize)
  {  
    for(int i = 0; i < DoF; i++)
    {
      buffer.push_back(cubicSplines[i](d));
    }
    interpolation.push_back(buffer);
    buffer.clear();
  }
  interpolation.push_back(frames[nofFrames-1]);
  
  return interpolation;
}

// description:   TODO
moveit::planning_interface::MoveGroupInterface::Plan concatenatePlans
  (moveit::planning_interface::MoveGroupInterface::Plan planA, moveit::planning_interface::MoveGroupInterface::Plan planB)
{
  // not tested yet
  ros::Duration endOfPlanA = planA.trajectory_.joint_trajectory.points.back().time_from_start;
  int i = 1;
  
  std::cout<<"size of planA: "<<planA.trajectory_.joint_trajectory.points.size()<<'\n';
  std::cout<<"size of planB: "<<planB.trajectory_.joint_trajectory.points.size()<<'\n';
  for(auto point:planB.trajectory_.joint_trajectory.points)
  {
    // std::cout<<i<<'\n';
    planA.trajectory_.joint_trajectory.points.push_back(point);
    // planA.trajectory_.multi_dof_joint_trajectory.points.push_back({});
    planA.trajectory_.joint_trajectory.points.back().time_from_start = point.time_from_start + endOfPlanA; 
    i++;
  }
  std::cout<<"size of concatenated plan: "<<planA.trajectory_.joint_trajectory.points.size()<<'\n';

  return planA;
  
}