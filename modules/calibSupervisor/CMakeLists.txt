# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.9)
project(calibSupervisor)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(YarpIDL)
include(YarpInstallationHelpers)
include(ICUBcontribOptions)
include(ICUBcontribHelpers)

icubcontrib_set_default_prefix()

yarp_add_idl(IDL_GEN_FILES ${PROJECT_NAME}.thrift)

set(idl ${PROJECT_NAME}.thrift)
set(doc ${PROJECT_NAME}.xml)
source_group("DOC Files" FILES ${doc})

include_directories(${YARP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp ${doc} ${idl} ${IDL_GEN_FILES})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
yarp_install(FILES ${doc} DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})

add_subdirectory(app)
