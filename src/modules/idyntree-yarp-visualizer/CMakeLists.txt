set(MODULE_NAME idyntree-yarp-visualizer)

set (HDR Visualizer.h
         RobotConnectors.h)
set (SRC main.cpp Visualizer.cpp
                  RobotConnectors.cpp)
set (THRIFTS thrifts/VisualizerCommands.thrift)
set (THRIFTS_VECTORS_COLLECTION thrifts/VectorsCollection.thrift)

yarp_add_idl(THRIFT_GEN_FILES ${THRIFTS})
yarp_add_idl(THRIFT_VECTORS_COLLECTION_GEN_FILES ${THRIFTS_VECTORS_COLLECTION})

add_executable(${MODULE_NAME} ${SRC} ${HDR} ${THRIFT_GEN_FILES} ${THRIFT_VECTORS_COLLECTION_GEN_FILES})
target_link_libraries(${MODULE_NAME} PRIVATE idyntree-yarp-utilities ReadOnlyRemoteControlBoardLib Eigen3::Eigen
                                             iDynTree::idyntree-core iDynTree::idyntree-modelio iDynTree::idyntree-visualization
                                             YARP::YARP_init YARP::YARP_os)

install(TARGETS ${MODULE_NAME} DESTINATION bin)
