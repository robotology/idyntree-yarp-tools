add_executable(urdf2dh urdf2dh.cpp)
target_link_libraries(urdf2dh iKin iDynTree::idyntree-core iDynTree::idyntree-model iDynTree::idyntree-icub iDynTree::idyntree-high-level)
install(TARGETS urdf2dh DESTINATION bin)
