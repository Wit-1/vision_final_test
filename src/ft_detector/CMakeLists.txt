cmake_minimum_required(VERSION 3.0.2)
project(ft_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs cv_bridge image_transport
  geometry_msgs std_msgs tf2 tf2_ros
  ft_detection dynamic_reconfigure
)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc calib3d)
generate_dynamic_reconfigure_options(cfg/Detector.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ft_detector_core ft_detector_output ft_detector_tuning
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport geometry_msgs std_msgs tf2 tf2_ros ft_detection dynamic_reconfigure
  DEPENDS OpenCV
)

# Keep algorithm and drawing code independent of ROS communication.
add_library(ft_detector_core
  src/traditional_detector.cpp
  src/debug_draw.cpp
  src/pnp_solver.cpp
  src/pose_history.cpp
)
target_compile_features(ft_detector_core PUBLIC cxx_std_14)
target_compile_options(ft_detector_core PRIVATE -Wall -Wextra)
target_include_directories(ft_detector_core PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ft_detector_core ${OpenCV_LIBRARIES})

# Keep ROS message/TF conversion outside the ROS-independent algorithm library.
add_library(ft_detector_output src/detection_output.cpp)
target_compile_features(ft_detector_output PUBLIC cxx_std_14)
target_compile_options(ft_detector_output PRIVATE -Wall -Wextra)
target_include_directories(ft_detector_output PUBLIC include ${catkin_INCLUDE_DIRS})
add_dependencies(ft_detector_output ${catkin_EXPORTED_TARGETS})
target_link_libraries(ft_detector_output PUBLIC ft_detector_core ${catkin_LIBRARIES})

add_library(ft_detector_tuning src/tuning_parameters.cpp)
target_compile_features(ft_detector_tuning PUBLIC cxx_std_14)
target_compile_options(ft_detector_tuning PRIVATE -Wall -Wextra)
target_include_directories(ft_detector_tuning PUBLIC include ${catkin_INCLUDE_DIRS})
add_dependencies(ft_detector_tuning ${PROJECT_NAME}_gencfg)
target_link_libraries(ft_detector_tuning PUBLIC ft_detector_core ${catkin_LIBRARIES})

add_executable(ft_detector_node src/detector_node.cpp)
target_compile_features(ft_detector_node PRIVATE cxx_std_14)
target_compile_options(ft_detector_node PRIVATE -Wall -Wextra)
target_include_directories(ft_detector_node PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(ft_detector_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(ft_detector_node ft_detector_output ft_detector_tuning ${catkin_LIBRARIES})

install(TARGETS ft_detector_core ft_detector_output ft_detector_tuning ft_detector_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
