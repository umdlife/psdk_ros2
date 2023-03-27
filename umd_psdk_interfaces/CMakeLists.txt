cmake_minimum_required(VERSION 3.5)
project(umd_psdk_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CameraStartShootSinglePhoto.action"
  "action/CameraStartShootBurstPhoto.action"
  "action/CameraStartShootAEBPhoto.action"
  "action/CameraStartShootIntervalPhoto.action"
  "action/CameraStopShootPhoto.action"
  "action/CameraRecordVideo.action"
  "action/CameraGetLaserRangingInfo.action"
  "action/CameraDownloadFileList.action"
  "action/CameraDownloadFileByIndex.action"
  "action/CameraDeleteFileByIndex.action"
  "action/CameraStreaming.action"
  "action/GimbalRotation.action"
  "msg/AircraftStatus.msg"
  "msg/Altitude.msg"
  "msg/Battery.msg"
  "msg/FlightAnomaly.msg"
  "msg/GimbalStatus.msg"
  "msg/HomePosition.msg"
  "msg/PositionFused.msg"
  "msg/RelativeObstacleInfo.msg"
  "msg/SDKCtrlInfo.msg"
  "srv/CameraGetType.srv"
  "srv/CameraSetEV.srv"
  "srv/CameraGetEV.srv"
  "srv/CameraSetShutterSpeed.srv"
  "srv/CameraGetShutterSpeed.srv"
  "srv/CameraSetISO.srv"
  "srv/CameraGetISO.srv"
  "srv/CameraSetFocusTarget.srv"
  "srv/CameraGetFocusTarget.srv"
  "srv/CameraSetFocusMode.srv"
  "srv/CameraGetFocusMode.srv"
  "srv/CameraSetOpticalZoom.srv"
  "srv/CameraGetOpticalZoom.srv"
  "srv/CameraSetInfraredZoom.srv"
  "srv/GimbalSetMode.srv"
  "srv/GimbalReset.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
