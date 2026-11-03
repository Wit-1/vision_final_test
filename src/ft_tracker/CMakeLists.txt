cmake_minimum_required(VERSION 3.0.2)
project(ft_tracker)

find_package(catkin REQUIRED COMPONENTS roscpp ft_detection ft_msgs geometry_msgs visualization_msgs)
find_package(OpenCV 4 REQUIRED COMPONENTS core video)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ft_tracker_core
  CATKIN_DEPENDS roscpp ft_detection ft_msgs geometry_msgs visualization_msgs
  DEPENDS OpenCV
)

add_library(ft_tracker_core src/linear_tracker.cpp)
target_compile_features(ft_tracker_core PUBLIC cxx_std_14)
target_compile_options(ft_tracker_core PRIVATE -Wall -Wextra)
target_include_directories(ft_tracker_core PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ft_tracker_core ${OpenCV_LIBRARIES})

add_executable(ft_tracker_node src/tracker_node.cpp)
target_compile_options(ft_tracker_node PRIVATE -Wall -Wextra)
target_include_directories(ft_tracker_node PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(ft_tracker_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ft_tracker_node ft_tracker_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(linear_tracker_test test/linear_tracker_test.cpp)
  if(TARGET linear_tracker_test)
    target_compile_options(linear_tracker_test PRIVATE -Wall -Wextra)
    target_link_libraries(linear_tracker_test ft_tracker_core)
  endif()
endif()

install(TARGETS ft_tracker_core ft_tracker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
