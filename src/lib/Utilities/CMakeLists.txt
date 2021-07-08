set(LIBRARY_NAME idyntree-yarp-utilities)

set (HDR Utilities.h)
set (SRC Utilities.cpp)

add_library(${LIBRARY_NAME} ${HDR} ${SRC})
target_link_libraries(${LIBRARY_NAME} PRIVATE YARP::YARP_os)
target_include_directories(${LIBRARY_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
