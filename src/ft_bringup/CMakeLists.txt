cmake_minimum_required(VERSION 3.0.2)
project(ft_bringup)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/view_debug_image.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
