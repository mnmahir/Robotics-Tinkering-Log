cmake_minimum_required(VERSION 3.8)
project(240804_ROS2_PCL_Clustering)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(pcl_segmentation src/pcl_segmentation.cpp)
ament_target_dependencies(pcl_segmentation rclcpp sensor_msgs std_msgs pcl_conversions visualization_msgs)
target_link_libraries(pcl_segmentation ${PCL_LIBRARIES})


install(TARGETS
  pcl_segmentation
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
