cmake_minimum_required(VERSION 3.8)
project(braccio_tp)

# Compile as C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(tag_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Dependencies list for target_link_libraries
set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tag_msgs
)

#############################################
#################### TP1 ####################
#############################################

add_library(Transformation
  student/Transformation.cpp
)
ament_target_dependencies(Transformation ${dependencies})
target_link_libraries(Transformation yaml-cpp)


# Localization library
add_library(Localization
  student/Localizer.cpp
)
ament_target_dependencies(Localization ${dependencies})
target_link_libraries(Localization Transformation )

# robot library
add_library(Robot
  student/Robot.cpp
)
ament_target_dependencies(Robot ${dependencies})
target_link_libraries(Robot Transformation )

# marker_broadcast executable
add_executable(marker_broadcast src/MarkersBroadcast.cpp)
ament_target_dependencies(marker_broadcast ${dependencies})
target_link_libraries(marker_broadcast Localization)


# marker_broadcast executable
add_executable(check_mgd src/CheckMGD.cpp)
ament_target_dependencies(check_mgd ${dependencies})
target_link_libraries(check_mgd Robot Localization)

# # rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# # target_link_libraries(marker_broadcast "${cpp_typesupport_target}")
#
# #############################################
# #################### TP2 ####################
# #############################################
#
# # RobotModel library
# add_library(RobotModel
#   lib/Robot.cpp
# )
# ament_target_dependencies(RobotModel ${dependencies})
#
# # check_MGD executable
# add_executable(check_MGD src/check_MGD.cpp)
# ament_target_dependencies(check_MGD ${dependencies})
# target_link_libraries(check_MGD RobotModel Localization)
# target_link_libraries(check_MGD "${cpp_typesupport_target}")
#
# # check_inverse_kinematic_joystick executable
# add_executable(check_inverse_kinematic_joystick src/InverseKinematicJoystick.cpp)
# ament_target_dependencies(check_inverse_kinematic_joystick ${dependencies})
# target_link_libraries(check_inverse_kinematic_joystick RobotModel Localization)
# target_link_libraries(check_inverse_kinematic_joystick "${cpp_typesupport_target}")
#
# # check_control_joystick executable
# add_executable(check_control_joystick src/RobotControlJoystick.cpp)
# ament_target_dependencies(check_control_joystick ${dependencies})
# target_link_libraries(check_control_joystick RobotModel Localization)
# target_link_libraries(check_control_joystick "${cpp_typesupport_target}")
#
# # robot_control_cube executable
# add_executable(robot_control_cube src/RobotControlCube.cpp)
# ament_target_dependencies(robot_control_cube ${dependencies})
# target_link_libraries(robot_control_cube RobotModel Localization)
# target_link_libraries(robot_control_cube "${cpp_typesupport_target}")
#
# # pick_and_place executable
# add_executable(pick_and_place src/PickAndPlace.cpp)
# ament_target_dependencies(pick_and_place ${dependencies})
# target_link_libraries(pick_and_place RobotModel)
# target_link_libraries(pick_and_place "${cpp_typesupport_target}")
#
# # robot_mover executable
# add_executable(robot_mover src/robot_mover.cpp)
# ament_target_dependencies(robot_mover ${dependencies})
# target_link_libraries(robot_mover "${cpp_typesupport_target}")

#############
## Install ##
#############

# Install executables
install(TARGETS
  marker_broadcast
  check_mgd
  # check_inverse_kinematic_joystick
  # check_control_joystick
  # robot_control_cube
  # pick_and_place
  # robot_mover
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(TARGETS
  Transformation
  Localization
  Robot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files, config files, etc.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(Transformation Localization Robot)
ament_export_dependencies(${dependencies})

ament_package()
