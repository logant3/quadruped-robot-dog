cmake_minimum_required(VERSION 3.8)
project(robot_dog_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes rviz xacro worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/send_leg_pose.py
    scripts/test_leg_motion.py
    scripts/log_joint_response.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
