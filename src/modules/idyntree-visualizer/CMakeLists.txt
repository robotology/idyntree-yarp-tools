set(MODULE_NAME idyntree-visualizer)

set (HDR Visualizer.h)
set (SRC main.cpp Visualizer.cpp)

add_executable(${MODULE_NAME} ${SRC} ${HDR})
target_link_libraries(${MODULE_NAME} PRIVATE idyntree-yarp-utilities Eigen3::Eigen
                                             iDynTree::idyntree-core iDynTree::idyntree-modelio-urdf iDynTree::idyntree-visualization
                                             ${YARP_LIBRARIES})
