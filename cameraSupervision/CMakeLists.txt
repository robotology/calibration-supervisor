# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Valentina Vasco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

option(GAZEBO_AVAILABLE "enable any build that require gazebo" ON)
if(GAZEBO_AVAILABLE)
    find_package(GAZEBO REQUIRED)
endif()

add_subdirectory(app)
add_subdirectory(camera-calibration-best-pos)
add_subdirectory(modules)
if(GAZEBO_AVAILABLE)
    add_subdirectory(gazebo)
endif()
