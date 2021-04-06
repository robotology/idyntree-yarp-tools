set(MODULE_NAME idyntree-visualizer)

set (SRC main.cpp)

add_executable(${MODULE_NAME} ${SRC})
target_link_libraries(${MODULE_NAME} PRIVATE Eigen3::Eigen
                                             iDynTree::idyntree-core iDynTree::idyntree-modelio-urdf iDynTree::idyntree-visualization
                                             ${YARP_LIBRARIES})
