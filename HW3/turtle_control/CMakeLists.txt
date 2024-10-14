cmake_minimum_required(VERSION 3.8)
project(turtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(ros2_custom_interface REQUIRED)

# Set up Qt automatic handling for MOC, UIC, RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}
)

# Specify where the .ui file is located
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# Add library for MainWindow
add_library(mainwindow_lib
  src/mainwindow.cpp
  include/mainwindow.h
)

# Link necessary libraries
target_link_libraries(mainwindow_lib Qt5::Widgets)
ament_target_dependencies(mainwindow_lib rclcpp ros2_custom_interface)

# Add executable 
add_executable(turtle_control
  src/main.cpp
)

target_link_libraries(turtle_control
  mainwindow_lib
  Qt5::Widgets
)
ament_target_dependencies(turtle_control rclcpp ros2_custom_interface)

# Install targets
install(TARGETS
  turtle_control
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

