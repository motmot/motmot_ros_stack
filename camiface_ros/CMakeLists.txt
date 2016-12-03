cmake_minimum_required(VERSION 2.8.12)
project(camiface_ros)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/motmot_ros_stack/camiface_ros/CMakeModules/")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  image_transport
  camera_info_manager
  )

FIND_PACKAGE(SDL)
FIND_PACKAGE(camiface)
FIND_PACKAGE(OpenGL)

generate_dynamic_reconfigure_options(
  cfg/camera_config.cfg
  )

# see http://wiki.ros.org/ROS/Tutorials/Using%20a%20C%2B%2B%20class%20in%20Python
catkin_package(
  CATKIN_DEPENDS roscpp dynamic_reconfigure
  LIBRARIES camiface_ros
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

ADD_EXECUTABLE(camnode src/camnode.cpp)
add_dependencies(camnode ${PROJECT_NAME}_gencfg)
include_directories(${CAMIFACE_INCLUDE_PATH} "cfg/cpp") # for dynamic_reconfigure
target_link_libraries(camnode
  ${CAMIFACE_LIBRARY}
  ${catkin_LIBRARIES})

# include the C shaders directly in the executable program
add_custom_command (
  OUTPUT demosaic_frg.c
  COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/tools/shader2c.py
                 ${CMAKE_CURRENT_SOURCE_DIR}/src/demosaic.frg
                 demosaic_frg.c
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tools/shader2c.py
  )
add_custom_command (
  OUTPUT demosaic_vrt.c
  COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/tools/shader2c.py
                 ${CMAKE_CURRENT_SOURCE_DIR}/src/demosaic.vrt
                 demosaic_vrt.c
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tools/shader2c.py
  )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

add_executable(gl_view src/gl_view.cpp demosaic_frg.c demosaic_vrt.c)
target_link_libraries(gl_view
  ${OPENGL_gl_LIBRARY}
  GLEW
  ${SDL_LIBRARY}
  ${catkin_LIBRARIES}
  )

add_executable(raw_info src/raw_info.cpp)
target_link_libraries(raw_info
  ${catkin_LIBRARIES}
  )
