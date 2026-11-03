cmake_minimum_required(VERSION 3.0.2)
project(ft_bringup)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY config launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/view_debug_image.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/view_rviz.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_install_python(PROGRAMS test/pipeline_test.py test/tracker_test.py test/tuning_test.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/pipeline.test)
  add_rostest(test/tracker.test)
  add_rostest(test/tuning.test)
endif()
