cmake_minimum_required(VERSION 3.8)
project(tugas-robotika)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(PIDController
  PACKAGE_DIR src/PIDController/PIDController)
ament_python_install_package(PIDNavigator
  PACKAGE_DIR src/PIDNavigator/PIDNavigator)
ament_python_install_package(RobotController
  PACKAGE_DIR src/RobotController/RobotController)

# Install Python executables
install(PROGRAMS
  src/PIDController/PIDController/pid_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pid_controller
)

install(PROGRAMS
  src/PIDNavigator/PIDNavigator/file_handling.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME file_handling
)

install(PROGRAMS
  src/RobotController/RobotController/robot_controller.py
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
