cmake_minimum_required(VERSION 3.8)
project(ricoh_theta)

# Enable C++17
add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)

# External dependencies via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_BASE REQUIRED gstreamer-base-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GLIB REQUIRED glib-2.0)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# Include directories
include_directories(
  include
  src
  ${GST_INCLUDE_DIRS}
  ${GST_BASE_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  ${GLIB_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
  /usr/local/include  # for libuvc if installed manually
)

# Link directories (for libuvc from /usr/local/lib)
link_directories(
  ${GST_LIBRARY_DIRS}
  ${GST_BASE_LIBRARY_DIRS}
  ${GST_APP_LIBRARY_DIRS}
  ${GLIB_LIBRARY_DIRS}
  ${LIBUSB_LIBRARY_DIRS}
  /usr/local/lib
)

# Main library (including the .c and .h file for thetauvc)
add_library(ricoh_theta SHARED
  src/ricoh_theta_node.cpp
  src/thetauvc.c
)

target_link_libraries(ricoh_theta
  ${OpenCV_LIBS}
  ${GST_LIBRARIES}
  ${GST_BASE_LIBRARIES}
  ${GST_APP_LIBRARIES}
  ${GLIB_LIBRARIES}
  ${LIBUSB_LIBRARIES}
  uvc
)

ament_target_dependencies(ricoh_theta
  rclcpp
  sensor_msgs
  rclcpp_components
)

# Register component
rclcpp_components_register_nodes(ricoh_theta "theta_driver::ThetaDriver")

# Install targets and headers
install(TARGETS
  ricoh_theta
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
add_executable(theta_node src/theta_node.cpp)
target_link_libraries(theta_node ricoh_theta)
ament_target_dependencies(theta_node rclcpp sensor_msgs rclcpp_components)

install(TARGETS theta_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
