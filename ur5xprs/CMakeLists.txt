cmake_minimum_required(VERSION 2.8.3)
project(ur5xprs)
# The version number.
set (ur5xprs_VERSION_MAJOR 1)
set (ur5xprs_VERSION_MINOR 0)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
	  COMPONENTS
    roscpp
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    cmake_modules
    std_msgs
    message_generation
)


add_message_files(
	FILES
	plotData.msg
)


add_service_files(
  FILES
  gripper.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)


catkin_package(CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    message_runtime
    std_msgs
)


include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# include_directories(/usr/include/eigen3)
#include_directories(~/Downloads/yaml-cpp-master)
# include_directories(~/rosSpace/src)

set(myLib ~/urdisney/src/ur5xprs/myLib)
include_directories(${myLib})

set(SOURCES_devel
	ur5motionPlanning-devel.cpp
	#${myLib}/functions.h
	${myLib}/functions.cpp
	)

set(SOURCES_principlePlanner
	principlePlanner.cpp
	#${myLib}/functions.h
	${myLib}/functions.cpp
	${myLib}/animation_functions.cpp
	)

set(SOURCES_regKeyframes
	registerKeyframes.cpp
	#${myLib}/functions.h
	${myLib}/functions.cpp
	)

set(SOURCES_regRobotPoses
	registerRobotPoses.cpp
	#${myLib}/functions.h
	${myLib}/functions.cpp
	)

set(SOURCES_configToPose
	configToPose.cpp
	#${myLib}/functions.h
	${myLib}/functions.cpp
	)
	
set(SOURCES_poseToConfig
	poseToConfig.cpp
	#${myLib}/functions.h
	${myLib}/functions.cpp
	)

set(SOURCES_goToConfig
	goToConfig.cpp
	#${myLib}/functions.h
	${myLib}/functions.cpp
	)

set(SOURCES_controlGripper
	controlGripper.cpp
	#${myLib}/functions.h
	${myLib}/functions.cpp
	)

set(SOURCES_registerGripper
	registerGripper.cpp
	#${myLib}/functions.h
	${myLib}/functions.cpp
	)

set(SOURCES_toGripper
	toGripper.cpp
	#${myLib}/functions.h
	${myLib}/functions.cpp
	)

set(SOURCES_goToPose
	goToPose.cpp
	#${myLib}/functions.h
	${myLib}/functions.cpp
	)

set(SOURCES_dragPose
	dragPose.cpp
	#${myLib}/functions.h
	${myLib}/functions.cpp
	)


set(SOURCES_initIdle
	initIdle.cpp
	#${myLib}/functions.h
	${myLib}/functions.cpp
	)

set(SOURCES_concatenator
	concatPlans.cpp
	#${myLib}/functions.h
	${myLib}/functions.cpp
	)

# target_link_libraries(ur5MP-devel ${catkin_LIBRARIES} ${myLib})
#add_executable(ur5MP-devel ${SOURCES_devel})
#target_link_libraries(ur5MP-devel ${catkin_LIBRARIES})
# target_link_libraries(ur5MP-devel ${catkin_LIBRARIES} ${myLib})

add_executable(principlePlanner ${SOURCES_principlePlanner})
target_link_libraries(principlePlanner ${catkin_LIBRARIES})

add_executable(regKeyframes ${SOURCES_regKeyframes})
target_link_libraries(regKeyframes ${catkin_LIBRARIES})

add_executable(regRobotPoses ${SOURCES_regRobotPoses})
target_link_libraries(regRobotPoses ${catkin_LIBRARIES})

add_executable(configToPose ${SOURCES_configToPose})
target_link_libraries(configToPose ${catkin_LIBRARIES})

add_executable(poseToConfig ${SOURCES_poseToConfig})
target_link_libraries(poseToConfig ${catkin_LIBRARIES})

add_executable(registerGripper ${SOURCES_registerGripper})
target_link_libraries(registerGripper ${catkin_LIBRARIES})

add_executable(controlGripper ${SOURCES_controlGripper})
target_link_libraries(controlGripper ${catkin_LIBRARIES})

add_executable(toGripper ${SOURCES_toGripper})
target_link_libraries(toGripper ${catkin_LIBRARIES})

add_executable(goToConfig ${SOURCES_goToConfig})
target_link_libraries(goToConfig ${catkin_LIBRARIES})

add_executable(goToPose ${SOURCES_goToPose})
target_link_libraries(goToPose ${catkin_LIBRARIES})

add_executable(dragPose ${SOURCES_dragPose})
target_link_libraries(dragPose ${catkin_LIBRARIES})

add_executable(initIdle ${SOURCES_initIdle})
target_link_libraries(initIdle ${catkin_LIBRARIES})

add_executable(concatenator ${SOURCES_concatenator})
target_link_libraries(concatenator ${catkin_LIBRARIES})

