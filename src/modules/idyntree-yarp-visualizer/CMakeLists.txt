set(MODULE_NAME idyntree-yarp-visualizer)

set (HDR Visualizer.h
         RobotConnectors.h)
set (SRC main.cpp Visualizer.cpp
                  RobotConnectors.cpp)
set (THRIFTS thrifts/VisualizerCommands.thrift)

yarp_add_idl(THRIFT_GEN_FILES ${THRIFTS})

add_executable(${MODULE_NAME} ${SRC} ${HDR} ${THRIFT_GEN_FILES})
target_link_libraries(${MODULE_NAME} PRIVATE idyntree-yarp-utilities Eigen3::Eigen
                                             iDynTree::idyntree-core iDynTree::idyntree-modelio-urdf iDynTree::idyntree-visualization
                                             ${YARP_LIBRARIES})
