cmake_minimum_required(VERSION 3.8)
project(bot_base)
find_package(ament_cmake REQUIRED
)
install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
