cmake_minimum_required(VERSION 3.5)
project(umd_psdk_wrapper)

SET(BUILD_SHARED_LIBS ON)
# Samples
set(CMAKE_C_FLAGS "-pthread -std=gnu99")
set(CMAKE_EXE_LINKER_FLAGS "-pthread")

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(umd_psdk_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(umd_rtsp REQUIRED)

# Samples
# include_directories(/Tools/Payload-SDK/samples/sample_c++/module_sample)
# include_directories(/Tools/Payload-SDK/samples/sample_c/module_sample)
# include_directories(/Tools/Payload-SDK/samples/sample_c++/platform/linux/common)
# include_directories(/Tools/Payload-SDK/samples/sample_c++/platform/linux/manifold2/application)
# file(GLOB_RECURSE MODULE_SAMPLE_SRC
#         /Tools/Payload-SDK/samples/sample_c++/module_sample/liveview/*.c*
#         # /Tools/Payload-SDK/samples/sample_c++/module_sample/perception/*.c*
#         /Tools/Payload-SDK/samples/sample_c/module_sample/camera_manager/test_camera_manager.c
#         )
# file(GLOB_RECURSE MODULE_COMMON_SRC /Tools/Payload-SDK/samples/sample_c++/platform/linux/common/*.c*)
# file(GLOB_RECURSE MODULE_HAL_SRC /Tools/Payload-SDK/samples/sample_c++/platform/linux/manifold2/hal/*.c*)
# file(GLOB_RECURSE MODULE_APP_SRC /Tools/Payload-SDK/samples/sample_c++/platform/linux/manifold2/application/*.c*)


include_directories(include
                    ${OpenCV_INCLUDE_DIRS})

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
set(PSDK_PATH /Tools/Payload-SDK/psdk_lib/lib/${TOOLCHAIN_NAME})
find_library(PSDK_LIB NAMES libpayloadsdk.a PATHS ${PSDK_PATH})
if (PSDK_LIB)
  message(STATUS "PSDK library found: ${PSDK_LIB}")
else()
  message(FATAL_ERROR "PSDK library NOT found")
endif()

set(OSAL_PATH /Tools/Payload-SDK/samples/sample_c++/platform/linux/common/osal)
set(HAL_PATH /Tools/Payload-SDK/samples/sample_c++/platform/linux/manifold2/hal)

# Include PSDK header files + low level OSAL and HAL libraries
include_directories(/Tools/Payload-SDK/psdk_lib/include)
#include_directories(${OSAL_PATH})
#include_directories(${HAL_PATH})
include_directories(include)
include_directories(src/common)
include_directories(src/streaming)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_util
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  umd_psdk_interfaces
  std_srvs
)

# Samples
set(CMAKE_MODULE_PATH /Tools/Payload-SDK/samples/sample_c++/platform/linux/common/3rdparty)

add_executable(psdk_wrapper_node
  src/main.cpp
)

add_library(psdk_wrapper_libs SHARED
  src/psdk_wrapper.cpp
  src/modules/telemetry.cpp
  src/modules/camera.cpp
  src/modules/gimbal.cpp
  src/modules/dji_camera_image_handler.cpp
  src/modules/dji_camera_stream_decoder.cpp
  src/common/osal_fs.c
  src/common/osal_socket.c
  src/common/osal.c
  src/common/hal_network.c
  src/common/hal_uart.c
  src/common/hal_usb_bulk.c
)

# Samples
# add_executable(${PROJECT_NAME}
#   /Tools/Payload-SDK/samples/sample_c++/platform/linux/manifold2/application/main.cpp
# )

# add_library(liveview_libs SHARED
#   ${MODULE_APP_SRC}
#   ${MODULE_SAMPLE_SRC}
#   ${MODULE_COMMON_SRC}
#   ${MODULE_HAL_SRC}
# )

# target_link_libraries(liveview_libs
#   ${OpenCV_LIBRARIES}
#   ${PSDK_LIB}
# )

# target_link_libraries(
#   ${PROJECT_NAME}
#   liveview_libs
# )

if (OpenCV_FOUND)
  message( "\n${PROJECT_NAME}...")
  message( STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
  message( STATUS " - Libraries: ${OpenCV_LIBRARIES}")
  add_definitions(-DOPEN_CV_INSTALLED)
else()
  message( STATUS "OpenCV is required for this sample to do the target tracking")
endif ()

find_package(FFMPEG REQUIRED)
if (FFMPEG_FOUND)
    message(STATUS "Found FFMPEG installed in the system")
    message(STATUS " - Includes: ${FFMPEG_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${FFMPEG_LIBRARIES}")

    EXECUTE_PROCESS(COMMAND ffmpeg -version
            OUTPUT_VARIABLE ffmpeg_version_psdk_libput
            OUTPUT_STRIP_TRAILING_WHITESPACE
            )
    string(REGEX MATCH "version.*Copyright" ffmpeg_version_line ${ffmpeg_version_psdk_libput})
    string(REGEX MATCH " .* " ffmpeg_version ${ffmpeg_version_line})
    string(REGEX MATCH "^ 5.*$" ffmpeg_major_version ${ffmpeg_version})

    if (HEAD${ffmpeg_major_version} STREQUAL "HEAD")
        message(STATUS " - Version: ${ffmpeg_version}")
    else ()
        message(FATAL_ERROR " - Not support FFMPEG version: ${ffmpeg_major_version}, please install 4.x.x instead.")
    endif ()

    target_link_libraries(psdk_wrapper_node ${FFMPEG_LIBRARIES})
else ()
    message(STATUS "Cannot Find FFMPEG")
endif (FFMPEG_FOUND)
include_directories(${FFMPEG_INCLUDE_DIR})

find_package(OPUS REQUIRED)
if (OPUS_FOUND)
    message(STATUS "Found OPUS installed in the system")
    message(STATUS " - Includes: ${OPUS_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${OPUS_LIBRARY}")

    add_definitions(-DOPUS_INSTALLED)
    target_link_libraries(psdk_wrapper_node /usr/local/lib/libopus.a)
else ()
    message(STATUS "Cannot Find OPUS")
endif (OPUS_FOUND)

find_package(LIBUSB REQUIRED)
if (LIBUSB_FOUND)
    message(STATUS "Found LIBUSB installed in the system")
    message(STATUS " - Includes: ${LIBUSB_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${LIBUSB_LIBRARY}")

    add_definitions(-DLIBUSB_INSTALLED)
    target_link_libraries(psdk_wrapper_node usb-1.0)
else ()
    message(STATUS "Cannot Find LIBUSB")
endif (LIBUSB_FOUND)

if (NOT EXECUTABLE_OUTPUT_PATH)
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
endif ()


ament_target_dependencies(psdk_wrapper_libs
  ${dependencies}
)

target_link_libraries(
  psdk_wrapper_libs
  ${PSDK_LIB}
  ${OpenCV_LIBRARIES}
  -lusb-1.0
  -lavcodec
  -lswscale
  umd_rtsp::umd_rtsp
)

target_link_libraries(
  psdk_wrapper_node
  psdk_wrapper_libs
)


target_link_libraries(psdk_wrapper_node m)



install(TARGETS
  psdk_wrapper_node
  psdk_wrapper_libs
  # ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  cfg
  DESTINATION share/${PROJECT_NAME}/
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