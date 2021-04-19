# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(calibOffsets)

yarp_add_idl(IDL_GEN_FILES ${PROJECT_NAME}.thrift)

set(idl ${PROJECT_NAME}.thrift)
set(doc ${PROJECT_NAME}.xml)

add_executable(${PROJECT_NAME} main.cpp ${doc} ${idl} ${IDL_GEN_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR})
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ctrlLib)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

add_subdirectory(app)
