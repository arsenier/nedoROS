cmake_minimum_required(VERSION 3.8)
project(apriltag_pose_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(apriltag_pose_detector_node src/apriltag_pose_detector.cpp)
ament_target_dependencies(apriltag_pose_detector_node
  rclcpp sensor_msgs geometry_msgs cv_bridge image_transport tf2 tf2_geometry_msgs
)
target_link_libraries(apriltag_pose_detector_node ${OpenCV_LIBS})

install(TARGETS apriltag_pose_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()