################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istitito Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB event_cameras_conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/event-cameras/*.ini)
yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/camera-calibration-best-pos)
yarp_install(FILES ${event_cameras_conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/camera-calibration-best-pos/event-cameras)
