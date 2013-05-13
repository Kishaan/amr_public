cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#...: compiler options :......................................................

#...: gnu++0x
if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        add_definitions(-std=gnu++0x)
    else(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        message(SEND_ERROR "You need GCC version 4.6 or greater to compile this package.")
    endif(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
endif(CMAKE_COMPILER_IS_GNUCXX)

#...: treat warnings as errors and disable centain warnings
add_definitions(-Werror)
add_definitions(-Wno-error=unused-variable)
add_definitions(-Wno-error=unknown-pragmas)
add_definitions(-Wno-unknown-pragmas)
add_definitions(-Wno-deprecated)

#...: determine OS type
if((CMAKE_SIZEOF_VOID_P MATCHES 4) OR (CMAKE_CL_64 MATCHES 0))
  set(SUFFIX x32)
elseif((CMAKE_SIZEOF_VOID_P MATCHES 8) OR (CMAKE_CL_64 MATCHES 1))
  set(SUFFIX x64)
else()
  message(SEND_ERROR "Unable to determine whether the OS is 32 or 64 bit.")
endif()

#...: find mapstore library
find_library(MAPSTORE_LIBRARY libmapstore_${SUFFIX}.so
  lib
)

#...: target executables :....................................................

#...: sonar_mapper
rosbuild_add_executable(sonar_mapper
  nodes/sonar_mapper.cpp
  src/sonar_map.cpp
)
target_link_libraries(sonar_mapper
  ${MAPSTORE_LIBRARY}
)

#...: sonar_simulation_test
rosbuild_add_executable(sonar_simulation_test
  nodes/sonar_simulation_test.cpp
)
