cmake_minimum_required(VERSION 3.8)
project(htc_tracker_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Find htc_vive_tracker library
find_path(htc_vive_tracker_INCLUDE_DIR htc_vive_tracker.h 
  PATHS /usr/local/include/iridrivers /usr/include/iridrivers
        ${CMAKE_CURRENT_SOURCE_DIR}/../../htc_vive_tracker/src)

find_library(htc_vive_tracker_LIBRARY
  NAMES htc_vive_tracker
  PATHS /usr/local/lib/iridrivers /usr/lib/iridrivers
        ${CMAKE_CURRENT_SOURCE_DIR}/../../htc_vive_tracker/lib)

# Find OpenVR library
find_library(OPENVR_LIBRARY
  NAMES openvr_api
  PATHS /usr/local/lib ${CMAKE_CURRENT_SOURCE_DIR}/../../openvr/lib/linux64)

if(NOT htc_vive_tracker_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find htc_vive_tracker.h")
endif()

if(NOT htc_vive_tracker_LIBRARY)
  message(FATAL_ERROR "Could not find htc_vive_tracker library")
endif()

if(NOT OPENVR_LIBRARY)
  message(FATAL_ERROR "Could not find openvr_api library")
endif()

message(STATUS "Found htc_vive_tracker include: ${htc_vive_tracker_INCLUDE_DIR}")
message(STATUS "Found htc_vive_tracker library: ${htc_vive_tracker_LIBRARY}")
message(STATUS "Found OpenVR library: ${OPENVR_LIBRARY}")

add_executable(htc_tracker_pos_publisher src/htc_tracker_pos_publisher.cpp)

target_include_directories(htc_tracker_pos_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${htc_vive_tracker_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../openvr/headers)

target_compile_features(htc_tracker_pos_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Get the directory paths of the libraries for RPATH
get_filename_component(htc_vive_tracker_LIB_DIR ${htc_vive_tracker_LIBRARY} DIRECTORY)
get_filename_component(OPENVR_LIB_DIR ${OPENVR_LIBRARY} DIRECTORY)

# Set RPATH to find the libraries at runtime
set_target_properties(htc_tracker_pos_publisher PROPERTIES
  INSTALL_RPATH "${htc_vive_tracker_LIB_DIR}:${OPENVR_LIB_DIR}"
  BUILD_RPATH "${htc_vive_tracker_LIB_DIR}:${OPENVR_LIB_DIR}"
  INSTALL_RPATH_USE_LINK_PATH TRUE)

target_link_libraries(htc_tracker_pos_publisher
  ${htc_vive_tracker_LIBRARY}
  ${OPENVR_LIBRARY}
  ${CMAKE_DL_LIBS})

ament_target_dependencies(htc_tracker_pos_publisher
  rclcpp
  geometry_msgs
  tf2_ros)

install(TARGETS htc_tracker_pos_publisher
  DESTINATION lib/${PROJECT_NAME})

# Add the new controller_6d_pose_publisher executable
add_executable(controller_6d_pose_publisher_robot src/controller_6d_pose_publisher_robot.cpp)

add_executable(controller_6d_pose_publisher src/controller_6d_pose_publisher.cpp)

target_include_directories(controller_6d_pose_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${htc_vive_tracker_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../openvr/headers)

target_compile_features(controller_6d_pose_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Set RPATH to find the libraries at runtime
set_target_properties(controller_6d_pose_publisher PROPERTIES
  INSTALL_RPATH "${htc_vive_tracker_LIB_DIR}:${OPENVR_LIB_DIR}"
  BUILD_RPATH "${htc_vive_tracker_LIB_DIR}:${OPENVR_LIB_DIR}"
  INSTALL_RPATH_USE_LINK_PATH TRUE)

target_link_libraries(controller_6d_pose_publisher
  ${htc_vive_tracker_LIBRARY}
  ${OPENVR_LIBRARY}
  ${CMAKE_DL_LIBS})

ament_target_dependencies(controller_6d_pose_publisher
  rclcpp
  geometry_msgs
  tf2_ros)

install(TARGETS controller_6d_pose_publisher
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(controller_6d_pose_publisher_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${htc_vive_tracker_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../openvr/headers)
target_compile_features(controller_6d_pose_publisher_robot PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# Set RPATH to find the libraries at runtime
set_target_properties(controller_6d_pose_publisher_robot PROPERTIES
  INSTALL_RPATH "${htc_vive_tracker_LIB_DIR}:${OPENVR_LIB_DIR}"
  BUILD_RPATH "${htc_vive_tracker_LIB_DIR}:${OPENVR_LIB_DIR}"
  INSTALL_RPATH_USE_LINK_PATH TRUE)
target_link_libraries(controller_6d_pose_publisher_robot
  ${htc_vive_tracker_LIBRARY}
  ${OPENVR_LIBRARY}
  ${CMAKE_DL_LIBS})
ament_target_dependencies(controller_6d_pose_publisher_robot
  rclcpp
  geometry_msgs
  tf2_ros
  sensor_msgs
  std_msgs)
install(TARGETS controller_6d_pose_publisher_robot
  DESTINATION lib/${PROJECT_NAME})
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install rviz config files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
