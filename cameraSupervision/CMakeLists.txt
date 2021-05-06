# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Valentina Vasco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(cameraSupervision)
find_package(GAZEBO QUIET)

add_subdirectory(app)
add_subdirectory(modules)
if(GAZEBO_FOUND)
    add_subdirectory(gazebo)
    add_subdirectory(camera-calibration-best-pos)
endif()
