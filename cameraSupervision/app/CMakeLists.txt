################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istitito Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

file(GLOB scripts ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/app/modify-params.sh ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/app/run-all.sh)
install(PROGRAMS ${scripts} DESTINATION bin)

if(GAZEBO_FOUND)
    file(GLOB gazebo-app ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/app/*.xml)
    file(GLOB gazebo-scripts ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/app/*.sh)
    file(GLOB rgb-cameras-app ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/app/rgb-cameras/*)
    file(GLOB event-cameras-app ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/app/event-cameras/*)
    yarp_install(FILES ${gazebo-app} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR}/${PROJECT_NAME}/camera-calibration-best-pos)
    yarp_install(DIRECTORY ${rgb-cameras-app} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR}/${PROJECT_NAME}/camera-calibration-best-pos/rgb-cameras)
    yarp_install(DIRECTORY ${event-cameras-app} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR}/${PROJECT_NAME}/camera-calibration-best-pos/event-cameras)
    install(PROGRAMS ${gazebo-scripts} DESTINATION bin)
endif()
