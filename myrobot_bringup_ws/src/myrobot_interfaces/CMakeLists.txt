cmake_minimum_required(VERSION 3.8)
project(myrobot_interfaces)

find_package(rosidl_default_generators REQUIRED)

# Generate StartStop.srv
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartStop.srv"
)

ament_package()
