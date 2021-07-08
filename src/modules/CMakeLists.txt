add_subdirectory(idyntree-yarp-visualizer)
add_subdirectory(yarprobotstatepublisher)


if(IDYNTREE_YARP_TOOLS_USES_ICUB_MAIN)
  add_subdirectory(urdf2dh)
endif()

if(IDYNTREE_YARP_TOOLS_USES_QT)
  add_subdirectory(idyntree-plotter)
endif()

if(IDYNTREE_YARP_TOOLS_USES_QT)
  add_subdirectory(idyntree-sole-gui)
endif()
