cmake_minimum_required(VERSION 3.5)

project(idyntree-yarp-tools
        LANGUAGES CXX C
        VERSION 0.0.1)

# Defines the CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_BINDIR and many other useful macros.
# See https://cmake.org/cmake/help/latest/module/GNUInstallDirs.html
include(GNUInstallDirs)

# Control where libraries and executables are placed during the build.
# With the following settings executables are placed in <the top level of the
# build tree>/bin and libraries/archives in <top level of the build tree>/lib.
# This is particularly useful to run ctests on libraries built on Windows
# machines: tests, which are executables, are placed in the same folders of
# dlls, which are treated as executables as well, so that they can properly
# find the libraries to run. This is a because of missing RPATH on Windows.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

# To build shared libraries in Windows, we set CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS to TRUE.
# See https://cmake.org/cmake/help/v3.4/variable/CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS.html
# See https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Under MSVC, we set CMAKE_DEBUG_POSTFIX to "d" to add a trailing "d" to library
# built in debug mode. In this Windows user can compile, build and install the
# library in both Release and Debug configuration avoiding naming clashes in the
# installation directories.
if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

# Build position independent code.
# Position Independent Code (PIC) is commonly used for shared libraries so that
# the same shared library code can be loaded in each program address space in a
# location where it will not overlap with any other uses of such memory.
# In particular, this option avoids problems occurring when a process wants to
# load more than one shared library at the same virtual address.
# Since shared libraries cannot predict where other shared libraries could be
# loaded, this is an unavoidable problem with the traditional shared library
# concept.
# Generating position-independent code is often the default behavior for most
# modern compilers.
# Moreover linking a static library that is not built with PIC from a shared
# library will fail on some compiler/architecture combinations.
# Further details on PIC can be found here:
# https://eli.thegreenplace.net/2011/11/03/position-independent-code-pic-in-shared-libraries/
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Disable C and C++ compiler extensions.
# C/CXX_EXTENSIONS are ON by default to allow the compilers to use extended
# variants of the C/CXX language.
# However, this could expose cross-platform bugs in user code or in the headers
# of third-party dependencies and thus it is strongly suggested to turn
# extensions off.
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(YCM REQUIRED)
find_package(YARP 3.3 REQUIRED COMPONENTS os dev math rosmsg idl_tools)
find_package(iDynTree 3.1.0 REQUIRED)
find_package(Eigen3 REQUIRED)

# Optional dependencies
find_package(ICUB QUIET)
option(IDYNTREE_YARP_TOOLS_USES_ICUB_MAIN "Compile the tools that depend on icub-main." ${ICUB_FOUND})
if(IDYNTREE_YARP_TOOLS_USES_ICUB_MAIN)
  find_package(ICUB REQUIRED)
endif()

find_package(Qt5 COMPONENTS Widgets QUIET)
option(IDYNTREE_YARP_TOOLS_USES_QT "Compile the tools that depend on Qt." ${Qt5_FOUND})

find_package(Qt5 COMPONENTS Charts QUIET)
option(IDYNTREE_YARP_TOOLS_USES_QT_CHARTS "Compile the tools that depend on Qt Charts." ${Qt5_FOUND})

if(IDYNTREE_YARP_TOOLS_USES_QT AND NOT IDYNTREE_YARP_TOOLS_USES_QT_CHARTS)
  find_package(Qt5 COMPONENTS Widgets REQUIRED)
endif()

if(IDYNTREE_YARP_TOOLS_USES_QT AND IDYNTREE_YARP_TOOLS_USES_QT_CHARTS)
  find_package(Qt5 COMPONENTS Widgets Charts REQUIRED)
endif()

### Options
# Shared/Dynamic or Static library?
# Setting BUILD_SHARED_LIBS to false by default to make sure that the ReadOnlyControlBoardPlugin is found also without installing.
option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" OFF)

# Build test related commands?
option(BUILD_TESTING "Create tests using CMake" OFF)
if(BUILD_TESTING)
    enable_testing()
endif()

# Enable RPATH support for installed binaries and libraries
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

# Encourage user to specify a build type (e.g. Release, Debug, etc.), otherwise set it to Release.
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

# Enable the use of M_PI on MSVC
if(MSVC)
  add_definitions(-D_USE_MATH_DEFINES)
endif()

### Compile- and install-related commands.
add_subdirectory(src)

# Add the uninstall target
include(AddUninstallTarget)
