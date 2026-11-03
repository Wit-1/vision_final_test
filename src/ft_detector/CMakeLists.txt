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

# Algorithm and drawing are independent of ROS, so geometry can be tested directly.
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

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(detector_geometry_test test/detector_geometry_test.cpp)
  if(TARGET detector_geometry_test)
    target_link_libraries(detector_geometry_test ft_detector_core)
    target_compile_options(detector_geometry_test PRIVATE -Wall -Wextra)
  endif()
  catkin_add_gtest(pnp_solver_test test/pnp_solver_test.cpp)
  if(TARGET pnp_solver_test)
    target_link_libraries(pnp_solver_test ft_detector_core)
    target_compile_options(pnp_solver_test PRIVATE -Wall -Wextra)
  endif()
  catkin_add_gtest(pose_history_test test/pose_history_test.cpp)
  if(TARGET pose_history_test)
    target_link_libraries(pose_history_test ft_detector_core)
    target_compile_options(pose_history_test PRIVATE -Wall -Wextra)
  endif()
  catkin_add_gtest(detection_output_test test/detection_output_test.cpp)
  if(TARGET detection_output_test)
    add_dependencies(detection_output_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(detection_output_test ft_detector_output)
    target_compile_options(detection_output_test PRIVATE -Wall -Wextra)
  endif()
endif()

install(TARGETS ft_detector_core ft_detector_output ft_detector_tuning ft_detector_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
