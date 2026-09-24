cmake_minimum_required(VERSION 3.8)
project(eskf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Rotation quaternions library
add_library(quaternions
  src/quaternions.cpp
)

# GNSS conversion library
add_library(gnss2ned
  src/gnss2ned.cpp
)

# ESKF implementation
add_library(eskf
  src/eskf.cpp
)

target_include_directories(
  quaternions
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(
  gnss2ned
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(
  eskf
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(eskf_node src/eskf_node.cpp)

ament_target_dependencies(eskf_node rclcpp std_msgs sensor_msgs nav_msgs)

target_include_directories(eskf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(eskf_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(quaternions
PUBLIC
Eigen3::Eigen
)

target_link_libraries(gnss2ned
PUBLIC
Eigen3::Eigen
)

target_link_libraries(eskf
PUBLIC
quaternions
Eigen3::Eigen
)


target_link_libraries(eskf_node
Eigen3::Eigen
eskf
gnss2ned
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  quaternions
  eskf_node
  DESTINATION lib/${PROJECT_NAME})

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
