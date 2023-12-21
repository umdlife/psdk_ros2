cmake_minimum_required(VERSION 3.5)
project(psdk_interfaces)

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
  "msg/DisplayMode.msg"
  "msg/FlightAnomaly.msg"
  "msg/GimbalStatus.msg"
  "msg/HomePosition.msg"
  "msg/PositionFused.msg"
  "msg/GPSDetails.msg"
  "msg/RelativeObstacleInfo.msg"
  "msg/SDKCtrlInfo.msg"
  "msg/GimbalRotation.msg"
  "msg/RTKYaw.msg"
  "msg/FlightStatus.msg"
  "msg/RCConnectionStatus.msg"
  "msg/ControlMode.msg"
  "srv/CameraGetType.srv"
  "srv/CameraSetExposureModeEV.srv"
  "srv/CameraGetExposureModeEV.srv"
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
  "srv/CameraSetAperture.srv"
  "srv/CameraGetAperture.srv"
  "srv/CameraShootSinglePhoto.srv"
  "srv/CameraShootBurstPhoto.srv"
  "srv/CameraShootAEBPhoto.srv"
  "srv/CameraShootIntervalPhoto.srv"
  "srv/CameraStopShootPhoto.srv"
  "srv/CameraRecordVideo.srv"
  "srv/CameraGetLaserRangingInfo.srv"
  "srv/CameraDownloadFileList.srv"
  "srv/CameraDownloadFileByIndex.srv"
  "srv/CameraDeleteFileByIndex.srv"
  "srv/CameraSetupStreaming.srv"
  "srv/GimbalSetMode.srv"
  "srv/GimbalReset.srv"
  "srv/SetHomeFromGPS.srv"
  "srv/SetGoHomeAltitude.srv"
  "srv/GetGoHomeAltitude.srv"
  "srv/SetObstacleAvoidance.srv"
  "srv/GetObstacleAvoidance.srv"
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
