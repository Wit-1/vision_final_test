cmake_minimum_required(VERSION 3.0.2)
project(ft_selector)

find_package(catkin REQUIRED COMPONENTS roscpp ft_detection ft_msgs geometry_msgs sensor_msgs std_msgs visualization_msgs dynamic_reconfigure)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV 4 REQUIRED COMPONENTS core calib3d)
generate_dynamic_reconfigure_options(cfg/Selector.cfg)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ft_selector_core
  CATKIN_DEPENDS roscpp ft_detection ft_msgs geometry_msgs sensor_msgs std_msgs visualization_msgs dynamic_reconfigure
  DEPENDS EIGEN3 OpenCV
)

add_library(ft_selector_core src/rotating_tracker.cpp src/target_selector.cpp src/observation_noise.cpp)
target_compile_features(ft_selector_core PUBLIC cxx_std_17)
target_compile_options(ft_selector_core PRIVATE -Wall -Wextra)
target_include_directories(ft_selector_core PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ft_selector_core PUBLIC Eigen3::Eigen ${OpenCV_LIBRARIES})

add_executable(ft_selector_node src/selector_node.cpp)
target_compile_options(ft_selector_node PRIVATE -Wall -Wextra)
target_include_directories(ft_selector_node PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(ft_selector_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(ft_selector_node ft_selector_core ${catkin_LIBRARIES})

install(TARGETS ft_selector_core ft_selector_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
