cmake_minimum_required(VERSION 3.5)
project(ndt_localizer)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(tf2_eigen REQUIRED) 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(small_gicp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msgs/NdtStat.msg
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

include_directories(include
  ${PCL_INCLUDE_DIRS}
)

ament_export_dependencies(rosidl_default_runtime)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(voxel_grid_filter nodes/points_downsampler.cpp)
ament_target_dependencies(voxel_grid_filter
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)
target_link_libraries(voxel_grid_filter
  ${PCL_LIBRARIES}
)

add_executable(map_loader nodes/map_loader.cpp)
ament_target_dependencies(map_loader
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
)
target_link_libraries(map_loader
  ${PCL_LIBRARIES}
)

add_executable(ndt_localizer_node nodes/ndt.cpp)
ament_target_dependencies(ndt_localizer_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
  diagnostic_msgs
  tf2_eigen
)
target_link_libraries(ndt_localizer_node
  ${PCL_LIBRARIES}
  small_gicp::small_gicp
)

install(TARGETS
  voxel_grid_filter
  map_loader
  ndt_localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY map/
  DESTINATION share/${PROJECT_NAME}/map
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY cfgs/
  DESTINATION share/${PROJECT_NAME}/cfgs
)

ament_package()
