# Install script for directory: /home/kai/gpmp2/gpmp2/kinematics

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gpmp2/kinematics" TYPE FILE FILES
    "/home/kai/gpmp2/gpmp2/kinematics/PointRobotModel.h"
    "/home/kai/gpmp2/gpmp2/kinematics/GoalFactorArm.h"
    "/home/kai/gpmp2/gpmp2/kinematics/Pose2MobileArm.h"
    "/home/kai/gpmp2/gpmp2/kinematics/ForwardKinematics-inl.h"
    "/home/kai/gpmp2/gpmp2/kinematics/Pose2Mobile2ArmsModel.h"
    "/home/kai/gpmp2/gpmp2/kinematics/GaussianPriorWorkspacePositionArm.h"
    "/home/kai/gpmp2/gpmp2/kinematics/GaussianPriorWorkspacePoseArm.h"
    "/home/kai/gpmp2/gpmp2/kinematics/JointLimitFactorVector.h"
    "/home/kai/gpmp2/gpmp2/kinematics/Pose2MobileVetLinArmModel.h"
    "/home/kai/gpmp2/gpmp2/kinematics/Pose2MobileVetLin2ArmsModel.h"
    "/home/kai/gpmp2/gpmp2/kinematics/Pose2MobileBaseModel.h"
    "/home/kai/gpmp2/gpmp2/kinematics/RobotModel.h"
    "/home/kai/gpmp2/gpmp2/kinematics/Arm.h"
    "/home/kai/gpmp2/gpmp2/kinematics/Pose2MobileVetLin2Arms.h"
    "/home/kai/gpmp2/gpmp2/kinematics/GaussianPriorWorkspaceOrientation.h"
    "/home/kai/gpmp2/gpmp2/kinematics/JointLimitCost.h"
    "/home/kai/gpmp2/gpmp2/kinematics/Pose2Mobile2Arms.h"
    "/home/kai/gpmp2/gpmp2/kinematics/VelocityLimitFactorVector.h"
    "/home/kai/gpmp2/gpmp2/kinematics/Pose2MobileVetLinArm.h"
    "/home/kai/gpmp2/gpmp2/kinematics/PointRobot.h"
    "/home/kai/gpmp2/gpmp2/kinematics/mobileBaseUtils.h"
    "/home/kai/gpmp2/gpmp2/kinematics/ForwardKinematics.h"
    "/home/kai/gpmp2/gpmp2/kinematics/GaussianPriorWorkspacePose.h"
    "/home/kai/gpmp2/gpmp2/kinematics/JointLimitFactorPose2Vector.h"
    "/home/kai/gpmp2/gpmp2/kinematics/Pose2MobileArmModel.h"
    "/home/kai/gpmp2/gpmp2/kinematics/GaussianPriorWorkspaceOrientationArm.h"
    "/home/kai/gpmp2/gpmp2/kinematics/GaussianPriorWorkspacePosition.h"
    "/home/kai/gpmp2/gpmp2/kinematics/ArmModel.h"
    "/home/kai/gpmp2/gpmp2/kinematics/RobotModel-inl.h"
    "/home/kai/gpmp2/gpmp2/kinematics/Pose2MobileBase.h"
    )
endif()

