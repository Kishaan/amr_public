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

# add dynamic reconfigure API
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#...: compiler options :......................................................

add_definitions(-std=gnu++0x)
add_definitions(-Wall)
add_definitions(-Wno-error=unused-variable)
add_definitions(-Wno-error=unknown-pragmas)
add_definitions(-Wno-error=address)
add_definitions(-Wno-deprecated)

#...: dependencies :..........................................................

#...: stage
find_path(STAGE_INCLUDE_DIR stage.hh
  /opt/stage/include/Stage-4.1
)
find_library(STAGE_LIBRARY stage
  /opt/stage/lib
  /opt/stage/lib64
)

if(NOT STAGE_INCLUDE_DIR OR NOT STAGE_LIBRARY)
  message(SEND_ERROR "Could not find Stage-4.1 installation under /opt/stage.")
endif(NOT STAGE_INCLUDE_DIR OR NOT STAGE_LIBRARY)

include_directories(${STAGE_INCLUDE_DIR})

#...: target executables :....................................................

rosbuild_add_executable(stage
  nodes/stage.cpp
  src/ranger.cpp
  src/laser.cpp
  src/sonar.cpp
  src/base.cpp
)
target_link_libraries(stage ${STAGE_LIBRARY})
