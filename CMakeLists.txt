cmake_minimum_required(VERSION 3.0.2)
project(hrpsys_sample)

find_package(catkin REQUIRED)
include(FindPkgConfig)
pkg_check_modules(openhrp3 REQUIRED openhrp3.1)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hrpsys_sample
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

set(OPENHRP_SAMPLE_DIR ${openhrp3_PREFIX}/share/OpenHRP-3.1/sample)
configure_file(${PROJECT_SOURCE_DIR}/config/SampleRobot.cnoid.in ${PROJECT_SOURCE_DIR}/config/SampleRobot.cnoid @ONLY)
