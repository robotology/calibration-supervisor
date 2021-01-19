# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Valentina Vasco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
project(camera-calibration)

find_package(YCM REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(YARP REQUIRED COMPONENTS os sig dev math cv)

option(GAZEBO_AVAILABLE "enable any build that require gazebo" ON)
if(GAZEBO_AVAILABLE)
    find_package(GAZEBO REQUIRED)
endif()

list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribOptions)
include(ICUBcontribHelpers)

icubcontrib_set_default_prefix()

add_subdirectory(app)
add_subdirectory(camera-calibration-best-pos)

add_subdirectory(modules)
if(GAZEBO_AVAILABLE)
    add_subdirectory(gazebo)
endif()
include(AddUninstallTarget)
