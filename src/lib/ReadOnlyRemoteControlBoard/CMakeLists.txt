# This piece of software was borrowed from human-dynamics-estimation (https://github.com/francesco-romano/human-dynamics-estimation)

yarp_prepare_plugin(readonlyremotecontrolboard TYPE yarp::dev::ReadOnlyRemoteControlBoard
                                               INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/ReadOnlyRemoteControlBoard.h
                                               CATEGORY device
                                               DEFAULT  ON)

set(PLUGIN_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/ReadOnlyRemoteControlBoard.cpp"
                   "${CMAKE_CURRENT_SOURCE_DIR}/stateExtendedReader.cpp")

set(PLUGIN_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/ReadOnlyRemoteControlBoard.h"
                   "${CMAKE_CURRENT_SOURCE_DIR}/stateExtendedReader.h")

yarp_add_plugin(readonlyremotecontrolboard ${PLUGIN_SOURCES} ${PLUGIN_HEADERS})

target_include_directories(readonlyremotecontrolboard PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(readonlyremotecontrolboard YARP::YARP_os YARP::YARP_dev)

yarp_install(TARGETS readonlyremotecontrolboard
             COMPONENT Runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}/
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}/)

yarp_install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/conf/readonlyremotecontrolboard.ini
             COMPONENT Runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}/)


