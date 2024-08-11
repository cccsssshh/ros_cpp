cmake_minimum_required(VERSION 3.8)
project(test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

set(UI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ui)
set(UI_FILE ${UI_DIR}/test.ui)

qt5_wrap_ui(UIC_FILES ${UI_FILE})
qt5_wrap_cpp(MOC_FILES include/test/main_window.hpp)

add_executable(qt_app
  src/qt_main.cpp
  src/main_window.cpp
  src/image_subscriber_node.cpp
  ${UIC_FILES}
  ${MOC_FILES}
)

set_property(TARGET qt_app PROPERTY AUTOUIC_SEARCH_PATHS 
  ${CMAKE_CURRENT_SOURCE_DIR}/ui
)

target_include_directories(qt_app PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Use ament_target_dependencies before target_link_libraries
ament_target_dependencies(qt_app
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# Change this to use the "plain" signature
target_link_libraries(qt_app
  Qt5::Widgets
  ${OpenCV_LIBS}
)

install(TARGETS
  qt_app
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
