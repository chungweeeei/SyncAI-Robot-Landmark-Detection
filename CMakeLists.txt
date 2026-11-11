cmake_minimum_required(VERSION 3.8)
project(syncai_landmark_detection)

# Detection runs on every camera frame on the Jetson; unoptimised it falls
# behind. Default to Release, but let an explicit CMAKE_BUILD_TYPE through.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
# SetParameters (called): the camera feeder's exposure_override_us.
find_package(rcl_interfaces REQUIRED)
# DetectLandmark (served).
find_package(syncai_common REQUIRED)
# aruco is the contrib module (libopencv-contrib-dev); calib3d for solvePnP
# and the fisheye model.
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d aruco)
find_package(tf2 REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
  rcl_interfaces
  syncai_common
)

# The plugin interface (landmark_detector.hpp) is header-only, as
# syncai_nav_core's is: installed below and exported, so a detector plugin in
# another package can build against it.

# ---- Camera exposure --------------------------------------------------------
# Which exposure shows a landmark, decided from the landmark's own region of
# the frame. Beside plugins/, not inside one: nothing in it is ArUco's. The
# policy and the measurement are ROS-free, so the gtests link them without a
# node; the client that writes the feeder's parameter is built into the
# plugin below.
add_library(camera_exposure_core STATIC
  camera_exposure/exposure_policy.cpp
  camera_exposure/region_brightness.cpp
)
target_include_directories(camera_exposure_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_exposure_core PUBLIC ${OpenCV_LIBS})
# Linked into a shared plugin library.
set_target_properties(camera_exposure_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The policy's ROS parameters, declared and read into ExposureParams: its own
# library so a gtest can take the params-file path on a plain node.
add_library(camera_exposure_parameters STATIC
  camera_exposure/exposure_parameters.cpp
)
target_include_directories(camera_exposure_parameters PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(camera_exposure_parameters rclcpp)
target_link_libraries(camera_exposure_parameters camera_exposure_core)
# Linked into a shared plugin library.
set_target_properties(camera_exposure_parameters PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---- The ArUco plugin -------------------------------------------------------
# Its geometry and solution choice are ROS-free (OpenCV only), so the gtests
# link them without a node, as syncai_global_planner's Octo3D core is.
add_library(aruco_core STATIC
  plugins/aruco_detector/marker_geometry.cpp
  plugins/aruco_detector/solution_selector.cpp
)
target_include_directories(aruco_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(aruco_core PUBLIC ${OpenCV_LIBS})
# Linked into a shared plugin library.
set_target_properties(aruco_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The plugin itself, loaded by pluginlib (landmark_detector_plugin.xml names
# this library).
add_library(${PROJECT_NAME}_aruco SHARED
  plugins/aruco_detector/aruco_detector.cpp
  camera_exposure/feeder_exposure_client.cpp
)
target_include_directories(${PROJECT_NAME}_aruco PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_aruco ${dependencies})
target_link_libraries(${PROJECT_NAME}_aruco aruco_core camera_exposure_core
  camera_exposure_parameters)

pluginlib_export_plugin_description_file(${PROJECT_NAME} landmark_detector_plugin.xml)

# ---- The server: landmark_detector -----------------------------------------
# Loads the plugins through pluginlib only; it links none of them.
add_executable(landmark_detector
  src/main.cpp
  src/landmark_detector_server.cpp
)
target_include_directories(landmark_detector PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(landmark_detector ${dependencies})

install(TARGETS ${PROJECT_NAME}_aruco
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS landmark_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The ArUco geometry is ported from SyncAI-Robot-Charger without per-file
  # licence headers (LICENSE covers the repo).
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # .clang-format (checked in CI) and uncrustify's ROS style contradict each other.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_marker_geometry test/test_marker_geometry.cpp)
  target_link_libraries(test_marker_geometry aruco_core)
  ament_add_gtest(test_solution_selector test/test_solution_selector.cpp)
  target_link_libraries(test_solution_selector aruco_core)
  ament_add_gtest(test_exposure_policy test/test_exposure_policy.cpp)
  target_link_libraries(test_exposure_policy camera_exposure_core)
  ament_add_gtest(test_region_brightness test/test_region_brightness.cpp)
  target_link_libraries(test_region_brightness camera_exposure_core)
  ament_add_gtest(test_exposure_parameters test/test_exposure_parameters.cpp)
  target_link_libraries(test_exposure_parameters camera_exposure_parameters)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
