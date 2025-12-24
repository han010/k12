cmake_minimum_required(VERSION 3.10)
project(revo_ugv_ros2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/revo_ugv_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    package.xml
  DESTINATION share/${PROJECT_NAME}
)

# 在环境中自动加入仓库自带的 Python SDK 路径，免去手工设置 PYTHONPATH
# CMAKE_CURRENT_SOURCE_DIR = .../RevoSDK_ws/src/revo_ugv_ros2
# ../../../python => .../revosdk/python
set(PY_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../python")
configure_file(
  env_hooks/sdk_pythonpath.sh.in
  "${CMAKE_CURRENT_BINARY_DIR}/sdk_pythonpath.sh"
  @ONLY
)
ament_environment_hooks("${CMAKE_CURRENT_BINARY_DIR}/sdk_pythonpath.sh")

ament_package()
