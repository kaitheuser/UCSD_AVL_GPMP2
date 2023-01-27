# Install script for directory: /home/kai/gpmp2/gpmp2/obstacle

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gpmp2/obstacle" TYPE FILE FILES
    "/home/kai/gpmp2/gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileVetLin2Arms.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileBase.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileBase.h"
    "/home/kai/gpmp2/gpmp2/obstacle/PlanarSDF.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstacleSDFFactorPose2MobileVetLinArm.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileArm.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstacleSDFFactorGP-inl.h"
    "/home/kai/gpmp2/gpmp2/obstacle/SDFexception.h"
    "/home/kai/gpmp2/gpmp2/obstacle/SelfCollisionArm.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactor.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileArm.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorGP-inl.h"
    "/home/kai/gpmp2/gpmp2/obstacle/SelfCollision.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstacleSDFFactorPose2Mobile2Arms.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorPose2Mobile2Arms.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstacleCost.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorGP.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstacleSDFFactorPose2MobileArm.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstacleSDFFactorPose2MobileVetLin2Arms.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileBase.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactor-inl.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileArm.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorArm.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2Mobile2Arms.h"
    "/home/kai/gpmp2/gpmp2/obstacle/SignedDistanceField.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstacleSDFFactorPose2MobileBase.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstacleSDFFactorArm.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstacleSDFFactor-inl.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstacleSDFFactor.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileVetLinArm.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstacleSDFFactorGP.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstacleSDFFactorGPPose2Mobile2Arms.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorGPArm.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstacleSDFFactorGPArm.h"
    "/home/kai/gpmp2/gpmp2/obstacle/ObstaclePlanarSDFFactorGPPointRobot.h"
    )
endif()

