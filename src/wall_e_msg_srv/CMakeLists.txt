cmake_minimum_required(VERSION 3.5)
project(wall_e_msg_srv)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SwitchAI.srv"
  "srv/PlaySound.srv"
  "srv/SetVolume.srv"
  "srv/SetLightIntensity.srv"
  "srv/Move.srv"
  "srv/MoveArm.srv"
  "srv/MoveHead.srv"
  "srv/MoveEye.srv"
  "srv/SwitchAutomaticMode.srv"
  "srv/GetAutomaticMode.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
