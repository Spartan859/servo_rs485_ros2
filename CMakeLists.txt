cmake_minimum_required(VERSION 3.5)
project(servo_rs485_ros2)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

# 添加所有舵机源文件（RS485 + PWM）
add_executable(servo_node 
  src/servo_node.cpp 
  src/servo_rs485.cpp
  src/servo_pwm.cpp
)
ament_target_dependencies(servo_node rclcpp std_msgs)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PingServo.srv"
  "srv/SetAngle.srv"
  "srv/GetAngle.srv"
  "srv/SetAngleWithSpeed.srv"
  DEPENDENCIES rclcpp std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(servo_node ${cpp_typesupport_target})

install(TARGETS servo_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
