cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMakeModules )

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

FIND_PACKAGE(SDL)
FIND_PACKAGE(camiface)
FIND_PACKAGE(OpenGL)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(camnode src/camnode.cpp)
include_directories(${CAMIFACE_INCLUDE_PATH})
target_link_libraries(camnode ${CAMIFACE_LIBRARY})

# include the C shaders directly in the executable program
add_custom_command (
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/demosaic_frg.c
  COMMAND python ${PROJECT_SOURCE_DIR}/tools/shader2c.py
                 ${PROJECT_SOURCE_DIR}/src/demosaic.frg
                 ${CMAKE_CURRENT_BINARY_DIR}/demosaic_frg.c
  DEPENDS ${PROJECT_SOURCE_DIR}/tools/shader2c.py
  )
add_custom_command (
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/demosaic_vrt.c
  COMMAND python ${PROJECT_SOURCE_DIR}/tools/shader2c.py
                 ${PROJECT_SOURCE_DIR}/src/demosaic.vrt
                 ${CMAKE_CURRENT_BINARY_DIR}/demosaic_vrt.c
  DEPENDS ${PROJECT_SOURCE_DIR}/tools/shader2c.py
  )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

rosbuild_add_executable(gl_view src/gl_view.cpp ${CMAKE_CURRENT_BINARY_DIR}/demosaic_frg.c ${CMAKE_CURRENT_BINARY_DIR}/demosaic_vrt.c)
target_link_libraries(gl_view ${OPENGL_gl_LIBRARY} GLEW ${SDL_LIBRARY})

rosbuild_add_executable(raw_info src/raw_info.cpp)
