cmake_minimum_required(VERSION 3.8)
project(air_quality_sensor_driver)


set(CMAKE_BUILD_TYPE "RelWithDebInfo")


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(traxxas1_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)


# Publisher
add_executable(pub src/pub.cpp)
target_compile_features(pub PUBLIC c_std_99 cxx_std_17)
target_include_directories(pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  pub
  rclcpp
  traxxas1_interfaces
  sensor_msgs)

  
# Mette dentro install gli eseguibili che si va a generare
install(TARGETS pub
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
