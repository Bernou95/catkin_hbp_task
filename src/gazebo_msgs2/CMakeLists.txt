cmake_minimum_required(VERSION 3.8)
project(gazebo_msgs2)

find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
  
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointStates.msg"
  "msg/OpenSimPhysics.msg"
  "msg/WheelSpeeds.msg"
  
  "srv/GetNextPoint.srv"
  "srv/GetNextColor.srv"
  DEPENDENCIES std_msgs geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_package()
