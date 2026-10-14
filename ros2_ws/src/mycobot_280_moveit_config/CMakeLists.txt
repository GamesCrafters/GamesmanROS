cmake_minimum_required(VERSION 3.8)
project(mycobot_280_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
