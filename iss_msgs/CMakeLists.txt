cmake_minimum_required(VERSION 3.8)
project(iss_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(vision_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies "geometry_msgs" "nav_msgs" "sensor_msgs" "std_msgs")

set(msg_files
  "msg/BBox3D.msg"
  "msg/ControlCommand.msg"
  "msg/ObjectDetection3D.msg"
  "msg/ObjectDetection3DArray.msg"
  "msg/State.msg"
  "msg/StateArray.msg"
)
set(srv_files
  "srv/SetGoal.srv"
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES ${dependencies}
)

ament_export_dependencies(rosidl_default_runtime)

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
