cmake_minimum_required(VERSION 3.8)
project(tfm_landmark_based_localization_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### Project dependences

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(detection_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# Dependences for own messages
find_package(rosidl_default_generators REQUIRED)

# Dependences for g2o examples
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(g2o REQUIRED)

add_subdirectory(libs/json)

### Define own messages

set(msg_files
  "msg/Results.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    std_msgs
    geometry_msgs
  ADD_LINTER_TESTS
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

### Define utils/commons files

add_library(my_commons SHARED 
  src/utils/utils.cpp
  src/utils/errors.cpp
)

ament_target_dependencies(my_commons rclcpp std_msgs geometry_msgs tf2_geometry_msgs)
target_link_libraries(my_commons Eigen3::Eigen nlohmann_json)

### Declare executables

# localitation_node declaration
add_executable(localitation_node src/localitation_node.cpp src/association_algorithm/Hungarian.cpp)
ament_target_dependencies(localitation_node rclcpp std_msgs geometry_msgs detection_msgs sensor_msgs)
target_link_libraries(localitation_node my_commons nlohmann_json "${cpp_typesupport_target}"
	Eigen3::Eigen g2o::core 
	g2o::types_sba g2o::types_slam2d 
	g2o::solver_dense g2o::solver_cholmod g2o::solver_eigen)

# fake_pole_detection_node declaration
add_executable(fake_pole_detection_node src/fake_pole_detection_node.cpp)
ament_target_dependencies(fake_pole_detection_node rclcpp std_msgs nav_msgs detection_msgs tf2_geometry_msgs)
target_link_libraries(fake_pole_detection_node g2o::core g2o::types_sba Eigen3::Eigen nlohmann_json)

# statistics_node declaration
add_executable(statistics_node src/statistics_node.cpp)
ament_target_dependencies(statistics_node rclcpp std_msgs nav_msgs detection_msgs message_filters)
target_link_libraries(statistics_node my_commons "${cpp_typesupport_target}")

### g2o examples declaration
add_executable(g2o_ba_demo
  src/g2o_examples/g2o_ba_demo.cpp
)
target_include_directories(g2o_ba_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(g2o_ba_demo
  rclcpp
)
target_link_libraries(g2o_ba_demo
  Eigen3::Eigen
  g2o::core
  g2o::types_sba
  g2o::solver_dense
  g2o::solver_cholmod
)

# G2O custom test
add_executable(g2o_test
  src/g2o_examples/g2o_test.cpp
)
target_include_directories(g2o_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(g2o_test
  rclcpp
  geometry_msgs
  tf2
)
target_link_libraries(g2o_test
  Eigen3::Eigen
  g2o::core
  g2o::types_sba
  g2o::types_slam2d
  g2o::solver_dense
  g2o::solver_cholmod
  g2o::solver_eigen
)

### Declare all targets
install(TARGETS 
  localitation_node
  fake_pole_detection_node
  statistics_node
  g2o_ba_demo
  g2o_test
  DESTINATION lib/${PROJECT_NAME}
)

### Install directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
