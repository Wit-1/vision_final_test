cmake_minimum_required(VERSION 3.10)
project(ft_tools LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS ft_detector ft_tracker ft_selector rosbag sensor_msgs)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

add_executable(benchmark_pipeline benchmark_pipeline.cpp)
target_compile_features(benchmark_pipeline PRIVATE cxx_std_17)
target_compile_options(benchmark_pipeline PRIVATE -Wall -Wextra)
target_include_directories(benchmark_pipeline PRIVATE ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(benchmark_pipeline PRIVATE ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} Eigen3::Eigen yaml-cpp)
