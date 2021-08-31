################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istitito Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

file(GLOB scripts ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/app/*.sh)
install(PROGRAMS ${scripts} DESTINATION bin)