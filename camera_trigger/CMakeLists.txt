cmake_minimum_required(VERSION 2.8)
project(camera_trigger)

find_package(catkin REQUIRED message_generation std_msgs)

add_service_files()

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

catkin_install_python(PROGRAMS nodes/camtrigger
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
