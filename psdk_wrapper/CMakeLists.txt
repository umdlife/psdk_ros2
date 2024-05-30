cmake_minimum_required(VERSION 3.14)
project(psdk_wrapper)

SET(BUILD_SHARED_LIBS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
add_definitions(-DSYSTEM_ARCH_LINUX)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(psdk_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED)

# FetchContent module
include(FetchContent)

# Declare the content to fetch
FetchContent_Declare(
  Payload_SDK
  GIT_REPOSITORY https://github.com/dji-sdk/Payload-SDK.git
  GIT_TAG 3.8.1
)

FetchContent_GetProperties(Payload_SDK)
if(NOT Payload_SDK_POPULATED)
  FetchContent_Populate(Payload_SDK)
  message(STATUS "Payload_SDK_SOURCE_DIR: ${payload_sdk_SOURCE_DIR}")
endif()

# Set the path to the fetched content
set(PSDK_PATH ${payload_sdk_SOURCE_DIR})

message(STATUS "PSDK_PATH: ${PSDK_PATH}")

# Add path to FindFFMPEG.cmake and FindLIBUSB.cmake files
set(CMAKE_MODULE_PATH ${PSDK_PATH}/samples/sample_c++/platform/linux/common/3rdparty)

find_package(FFMPEG REQUIRED)
find_package(LIBUSB REQUIRED)

set(DJI_OSAL_PATH ${PSDK_PATH}/samples/sample_c++/platform/linux/common/osal)
set(DJI_HAL_PATH ${PSDK_PATH}/samples/sample_c++/platform/linux/manifold2/hal)
set(DJI_STREAMING_PATH ${PSDK_PATH}/samples/sample_c++/module_sample/liveview)
set(DJI_UTILS_PATH ${PSDK_PATH}/samples/sample_c/module_sample)
set(DJI_HMS_RETURN_CODES_PATH ${PSDK_PATH}/samples/sample_c/module_sample/hms/data)

# Find toolchain name needed to select the right PSDK lib
execute_process(COMMAND uname -m
  OUTPUT_VARIABLE DEVICE_SYSTEM_ID)

if(DEVICE_SYSTEM_ID MATCHES x86_64)
  set(TOOLCHAIN_NAME x86_64-linux-gnu-gcc)
  add_definitions(-DPLATFORM_ARCH_x86_64=1)
elseif(DEVICE_SYSTEM_ID MATCHES aarch64)
  set(TOOLCHAIN_NAME aarch64-linux-gnu-gcc)
  add_definitions(-DPLATFORM_ARCH_aarch64=1)
else()
  message(FATAL_ERROR "FATAL: Please confirm your platform.")
endif()

# Check if PSDK lib is found
set(PSDK_LIB_PATH ${PSDK_PATH}/psdk_lib/lib/${TOOLCHAIN_NAME})
find_library(PSDK_LIB NAMES libpayloadsdk.a PATHS ${PSDK_LIB_PATH})
if (PSDK_LIB)
  message(STATUS "PSDK library found: ${PSDK_LIB}")
else()
  message(FATAL_ERROR "PSDK library NOT found")
endif()

if (FFMPEG_FOUND)
    message(STATUS "Found FFMPEG installed in the system")
    message(STATUS " - Includes: ${FFMPEG_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${FFMPEG_LIBRARIES}")

    EXECUTE_PROCESS(COMMAND ffmpeg -version
            OUTPUT_VARIABLE ffmpeg_version_psdk_libput
            OUTPUT_STRIP_TRAILING_WHITESPACE
            )
    string(REGEX MATCH "version.*Copyright" ffmpeg_version_line "${ffmpeg_version_psdk_libput}")
    string(REGEX MATCH " .* " ffmpeg_version "${ffmpeg_version_line}")
    string(REGEX MATCH "^ 5.*$" ffmpeg_major_version "${ffmpeg_version}")

    if (HEAD${ffmpeg_major_version} STREQUAL "HEAD")
        message(STATUS " - Version: ${ffmpeg_version}")
    else ()
        message(FATAL_ERROR " - Not support FFMPEG version: ${ffmpeg_major_version}, please install 4.x.x instead.")
    endif ()
    add_definitions(-DFFMPEG_INSTALLED)
else ()
    message(STATUS "Cannot Find FFMPEG")
endif (FFMPEG_FOUND)

if (LIBUSB_FOUND)
    message(STATUS "Found LIBUSB installed in the system")
    message(STATUS " - Includes: ${LIBUSB_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${LIBUSB_LIBRARY}")

    add_definitions(-DLIBUSB_INSTALLED)
else ()
    message(STATUS "Cannot Find LIBUSB")
endif (LIBUSB_FOUND)

if (NOT EXECUTABLE_OUTPUT_PATH)
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
endif ()

# Include PSDK header files
include_directories(${FFMPEG_INCLUDE_DIR})
include_directories(${DJI_OSAL_PATH})
include_directories(${DJI_HAL_PATH})
include_directories(${DJI_STREAMING_PATH})
include_directories(${DJI_UTILS_PATH})
include_directories(${PSDK_PATH}/psdk_lib/include)
include_directories(include)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  psdk_interfaces
  std_srvs
)

add_executable(psdk_wrapper_node
  src/main.cpp
)

add_library(psdk_wrapper_libs SHARED
  src/psdk_wrapper.cpp
  src/modules/telemetry.cpp
  src/modules/flight_control.cpp
  src/modules/camera.cpp
  src/modules/gimbal.cpp
  src/modules/liveview.cpp
  src/modules/hms.cpp
  src/utils/node_thread.cpp
  ${DJI_OSAL_PATH}/osal_fs.c
  ${DJI_OSAL_PATH}/osal_socket.c
  ${DJI_OSAL_PATH}/osal.c
  ${DJI_HAL_PATH}/hal_network.c
  ${DJI_HAL_PATH}/hal_usb_bulk.c
  ${DJI_HAL_PATH}/hal_uart.c
  ${DJI_STREAMING_PATH}/dji_camera_image_handler.cpp
  ${DJI_STREAMING_PATH}/dji_camera_stream_decoder.cpp
  ${DJI_UTILS_PATH}/utils/util_misc.c
  ${DJI_UTILS_PATH}/utils/util_file.c
  ${DJI_UTILS_PATH}/utils/cJSON.c
  ${DJI_UTILS_PATH}/utils/dji_config_manager.c
)

ament_target_dependencies(psdk_wrapper_libs
  ${dependencies}
)

target_link_libraries(
  psdk_wrapper_libs
  ${PSDK_LIB}
  nlohmann_json::nlohmann_json
)

target_link_libraries(
  psdk_wrapper_node
  psdk_wrapper_libs
  ${FFMPEG_LIBRARIES}
  -lusb-1.0
  -lavcodec
  -lswscale
)

install(TARGETS
  psdk_wrapper_node
  psdk_wrapper_libs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  include/
  ${PSDK_PATH}/psdk_lib/include/
  ${DJI_STREAMING_PATH}/
  ${DJI_HAL_PATH}/
  ${DJI_OSAL_PATH}/
  ${DJI_UTILS_PATH}/
  DESTINATION include/
  FILES_MATCHING 
    PATTERN "*.h"
    PATTERN "*.hpp"
)

install(DIRECTORY
  cfg
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  ${DJI_HMS_RETURN_CODES_PATH}/
  DESTINATION share/${PROJECT_NAME}/cfg
)

ament_export_dependencies(
  "rclcpp"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(psdk_wrapper_libs)
ament_export_dependencies(${dependencies})

ament_package()
