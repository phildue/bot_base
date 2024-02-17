cmake_minimum_required(VERSION 3.8)
project(bot_differential_drive)

find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(pluginlib REQUIRED)
include_directories(
  src/
)
add_library(${PROJECT_NAME} SHARED 
  src/DifferentialDriveSerialInterface.h
  src/DifferentialDriveMock.cpp
  src/DifferentialDriveSerialInterface.cpp
  src/SerialProtocol.h
  src/SerialProtocol.cpp
  src/SerialPort.hpp
  src/SerialPort.cpp)

ament_export_include_directories(src)

ament_target_dependencies(${PROJECT_NAME} 
rclcpp
hardware_interface
std_msgs
sensor_msgs
tf2
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "BOT_DIFFERENTIAL_DRIVE_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface description/bot_differential_drive.xml)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/)

  install(DIRECTORY
  description
  DESTINATION share/${PROJECT_NAME}/)

install(
  TARGETS 
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include)


if(BUILD_TESTING)
  add_executable(test_control test/test_control.cpp)
  target_link_libraries(test_control ${PROJECT_NAME})
  add_executable(test_query test/test_query.cpp)
  target_link_libraries(test_query ${PROJECT_NAME})

    if(BUILD_LINTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
    endif()
endif()
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PROJECT_NAME} 
rclcpp
hardware_interface
std_msgs
sensor_msgs
tf2
)
ament_package()
