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
# DetectLandmark (served).
find_package(syncai_common REQUIRED)
# aruco is the contrib module (libopencv-contrib-dev); calib3d for solvePnP
# and the fisheye model.
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d aruco)

# The repo is set up before any code: the plugin interface, the ArUco plugin
# and landmark_detector are added here as they are written.

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
endif()

ament_package()
