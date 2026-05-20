cmake_minimum_required(VERSION 3.8)
project(bio_transport_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

# 액션 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/BioCommand.action"
  "action/RobotMove.action"
  "action/TubeTransport.action"
  DEPENDENCIES action_msgs
)

ament_package()