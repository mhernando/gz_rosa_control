cmake_minimum_required(VERSION 3.5)
project(gz_rosa_control)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(gazebo_ros_omni_drive SHARED
  src/gazebo_ros_omni_drive.cpp
)
#inlcude here additional include dirs
#target_include_directories(gazebo_ros_rosa_control PUBLIC include)

#igual es necesario incluir nav_msgs 
ament_target_dependencies(gazebo_ros_omni_drive
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
  "nav_msgs"
)
ament_export_libraries(gazebo_ros_omni_drive)

install(TARGETS
gazebo_ros_omni_drive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
