cmake_minimum_required(VERSION 3.0.2)
project(ft_detector)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge image_transport)
find_package(OpenCV 4 REQUIRED COMPONENTS core)

catkin_package(CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport)

add_executable(ft_detector_node src/detector_node.cpp)
target_compile_features(ft_detector_node PRIVATE cxx_std_14)
target_compile_options(ft_detector_node PRIVATE -Wall -Wextra)
target_include_directories(ft_detector_node PRIVATE ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_dependencies(ft_detector_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ft_detector_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS ft_detector_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
