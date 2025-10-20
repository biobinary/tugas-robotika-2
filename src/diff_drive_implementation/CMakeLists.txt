cmake_minimum_required(VERSION 3.8)
project(diff_drive_implementation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/${PROJECT_NAME}/pid_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pid_controller
)

install(PROGRAMS
  src/${PROJECT_NAME}/file_handling.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME file_handling
)

install(PROGRAMS
  src/${PROJECT_NAME}/robot_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME robot_controller
)

# Install directories
install(
  DIRECTORY launch rviz config worlds models description maps data
  DESTINATION share/${PROJECT_NAME}
)

# Environment hooks
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/${PROJECT_NAME}.sh.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
