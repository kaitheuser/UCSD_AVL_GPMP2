# Install script for directory: /home/kai/gpmp2/gpmp2/gp

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gpmp2/gp" TYPE FILE FILES
    "/home/kai/gpmp2/gpmp2/gp/GaussianProcessPriorPose3.h"
    "/home/kai/gpmp2/gpmp2/gp/GaussianProcessInterpolatorPose2Vector.h"
    "/home/kai/gpmp2/gpmp2/gp/GPutils.h"
    "/home/kai/gpmp2/gpmp2/gp/GaussianProcessInterpolatorPose2.h"
    "/home/kai/gpmp2/gpmp2/gp/GaussianProcessPriorLinear.h"
    "/home/kai/gpmp2/gpmp2/gp/GaussianProcessInterpolatorLinear.h"
    "/home/kai/gpmp2/gpmp2/gp/GaussianProcessInterpolatorPose3.h"
    "/home/kai/gpmp2/gpmp2/gp/GaussianProcessInterpolatorLie.h"
    "/home/kai/gpmp2/gpmp2/gp/GaussianProcessPriorPose2Vector.h"
    "/home/kai/gpmp2/gpmp2/gp/GaussianProcessPriorPose2.h"
    "/home/kai/gpmp2/gpmp2/gp/GaussianProcessPriorLie.h"
    )
endif()

