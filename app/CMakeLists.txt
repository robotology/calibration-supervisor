################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istitito Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

file(GLOB app ${CMAKE_SOURCE_DIR}/app/*.xml)
file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/*.sh)
file(GLOB event-cameras-app ${CMAKE_SOURCE_DIR}/app/event-cameras/*.xml)
yarp_install(FILES ${app} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR}/${PROJECT_NAME}/camera-calibration-best-pos)
yarp_install(FILES ${event-cameras-app} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR}/${PROJECT_NAME}/camera-calibration-best-pos/event-cameras)
install(PROGRAMS ${scripts} DESTINATION bin)
