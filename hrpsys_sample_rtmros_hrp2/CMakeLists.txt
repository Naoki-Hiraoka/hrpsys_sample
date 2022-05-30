cmake_minimum_required(VERSION 3.0.2)
project(hrpsys_sample_rtmros_hrp2)

find_package(catkin REQUIRED COMPONENTS
  jvrc_models
  hrpsys_choreonoid_tutorials)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hrpsys_sample
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

configure_file(${PROJECT_SOURCE_DIR}/config/JAXON_JVRC.cnoid.in ${PROJECT_SOURCE_DIR}/config/JAXON_JVRC.cnoid @ONLY)
