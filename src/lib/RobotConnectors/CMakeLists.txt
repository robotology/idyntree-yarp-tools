set(LIBRARY_NAME idyntree-yarp-robot-connectors)

set (HDR RobotConnectors.h)
set (SRC RobotConnectors.cpp)

add_library(${LIBRARY_NAME} ${HDR} ${SRC})
target_link_libraries(${LIBRARY_NAME} PRIVATE YARP::YARP_os
                                              YARP::YARP_rosmsg
                                              YARP::YARP_dev
                                              ReadOnlyRemoteControlBoardLib)

target_include_directories(${LIBRARY_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
