# Copyright: (C) 2016 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

file(GLOB conf    ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

file(GLOB rgb-assets  ${CMAKE_CURRENT_SOURCE_DIR}/assets/rgb-cameras/*)
file(GLOB event-assets  ${CMAKE_CURRENT_SOURCE_DIR}/assets/event-cameras/*)

yarp_install(FILES ${conf}    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
yarp_install(DIRECTORY ${rgb-assets}    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME}/rgb-cameras)
yarp_install(DIRECTORY ${event-assets}    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME}/event-cameras)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
