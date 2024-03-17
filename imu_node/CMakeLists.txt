cmake_minimum_required(VERSION 3.8)
project(imu_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(xsens_library REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# find dependencies
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# let the compiler search for headers in the include folder
set(MOVELLA_INSTALL_PREFIX /usr/local/movella/)
include_directories(${MOVELLA_INSTALL_PREFIX}/include ${MOVELLA_INSTALL_PREFIX}/include/movelladot_pc_sdk)
include_directories(${MOVELLA_INSTALL_PREFIX}/lib)

link_directories(${MOVELLA_INSTALL_PREFIX}/lib)

add_executable(imu_pub src/imu_publisher.cpp)
add_executable(imu_rec src/imu_receiver.cpp)

ament_target_dependencies(imu_pub xsens_library rclcpp std_msgs sensor_msgs)
ament_target_dependencies(imu_rec xsens_library rclcpp std_msgs sensor_msgs)

# install the executable in the lib folder to make it detectable through setup.bash
install(TARGETS
  imu_pub
  imu_rec
  DESTINATION lib/${PROJECT_NAME}/
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

ament_package()
