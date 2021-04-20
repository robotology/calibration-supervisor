# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Valentina Vasco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
project(calibration-supervisor)

find_package(OpenCV REQUIRED)
find_package(YCM REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(ICUB REQUIRED)
find_package(YARP REQUIRED COMPONENTS os sig dev math cv)

option(SUPERVISE_CAMERA "enable camera calibration supervision" ON)
option(SUPERVISE_REDBALL "enable demo red ball calibration" ON)
if (SUPERVISE_CAMERA)
    add_subdirectory(cameraSupervision)    
endif()
if (SUPERVISE_REDBALL)
    add_subdirectory(demoRedBallCalibration)
endif()

list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribOptions)
include(ICUBcontribHelpers)

icubcontrib_set_default_prefix()
include(AddUninstallTarget)
