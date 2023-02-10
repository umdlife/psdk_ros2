cmake_minimum_required(VERSION 3.5)
project(umd_psdk_wrapper)

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)

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
include_directories(${OSAL_PATH})
include_directories(${HAL_PATH})
include_directories(include)



set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_util
)

add_executable(psdk_wrapper_node
  src/main.cpp
)

add_library(psdk_wrapper_libs SHARED
  src/psdk_wrapper.cpp
  ${OSAL_PATH}/osal_fs.c
  ${OSAL_PATH}/osal_socket.c
  ${OSAL_PATH}/osal.c
  ${HAL_PATH}/hal_network.c
  ${HAL_PATH}/hal_uart.c
  ${HAL_PATH}/hal_usb_bulk.c
)

ament_target_dependencies(psdk_wrapper_libs
  ${dependencies}
)

target_link_libraries(
  psdk_wrapper_libs
  ${PSDK_LIB}
)

target_link_libraries(
  psdk_wrapper_node
  psdk_wrapper_libs
)

install(TARGETS
  psdk_wrapper_libs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/psdk_wrapper_libs
)
install(DIRECTORY include/
  DESTINATION include/
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
