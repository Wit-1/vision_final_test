cmake_minimum_required(VERSION 3.0.2)
project(ft_detector)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge image_transport)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ft_detector_core
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport
  DEPENDS OpenCV
)

# Algorithm and drawing are independent of ROS, so geometry can be tested directly.
add_library(ft_detector_core src/traditional_detector.cpp src/debug_draw.cpp)
target_compile_features(ft_detector_core PUBLIC cxx_std_14)
target_compile_options(ft_detector_core PRIVATE -Wall -Wextra)
target_include_directories(ft_detector_core PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ft_detector_core ${OpenCV_LIBRARIES})

add_executable(ft_detector_node src/detector_node.cpp)
target_compile_features(ft_detector_node PRIVATE cxx_std_14)
target_compile_options(ft_detector_node PRIVATE -Wall -Wextra)
target_include_directories(ft_detector_node PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(ft_detector_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ft_detector_node ft_detector_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(detector_geometry_test test/detector_geometry_test.cpp)
  if(TARGET detector_geometry_test)
    target_link_libraries(detector_geometry_test ft_detector_core)
    target_compile_options(detector_geometry_test PRIVATE -Wall -Wextra)
  endif()
endif()

install(TARGETS ft_detector_core ft_detector_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
