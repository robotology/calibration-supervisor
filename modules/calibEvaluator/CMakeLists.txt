# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Valentina Vasco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(calibEvaluator)

find_package(OpenCV REQUIRED)

yarp_add_idl(IDL_GEN_FILES ${PROJECT_NAME}.thrift)

set(idl ${PROJECT_NAME}.thrift)

add_executable(${PROJECT_NAME} main.cpp ${idl} ${IDL_GEN_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

add_subdirectory(app)
