################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istitito Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

file(GLOB app ${CMAKE_SOURCE_DIR}/app/*.xml)
file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/*.sh)
yarp_install(FILES ${app} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR}/${PROJECT_NAME})
install(PROGRAMS ${scripts} DESTINATION bin)
