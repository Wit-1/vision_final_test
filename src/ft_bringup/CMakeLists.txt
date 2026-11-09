cmake_minimum_required(VERSION 3.0.2)
project(ft_bringup)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY config launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/view_debug_image.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/view_rviz.py scripts/view_reconfigure.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/compare_detections.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/compare_predictions.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
