cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)
install(FILES LICENSE NOTICE
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
