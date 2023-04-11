/* Copyright (C) 2023 Unmanned Life - All Rights Reserved
 *
 * This file is part of the `umd_psdk_wrapper` source code package and is
 * subject to the terms and conditions defined in the file LICENSE.txt contained
 * therein.
 */
/**
 * @file psdk_wrapper.cpp
 *
 * @brief
 *
 * @author Bianca Bendris
 * Contact: bianca@unmanned.life
 *
 */

#include "psdk_wrapper/psdk_wrapper.hpp"

using namespace std::placeholders;  // NOLINT

namespace psdk_ros2
{
using namespace std::placeholders;

PSDKWrapper::PSDKWrapper(const std::string &node_name)
    : nav2_util::LifecycleNode(node_name, "", rclcpp::NodeOptions())
{
  RCLCPP_INFO(get_logger(), "Creating Constructor PSDKWrapper");
  declare_parameter("app_name", rclcpp::ParameterValue(""));
  declare_parameter("app_id", rclcpp::ParameterValue(""));
  declare_parameter("app_key", rclcpp::ParameterValue(""));
  declare_parameter("app_license", rclcpp::ParameterValue(""));
  declare_parameter("developer_account", rclcpp::ParameterValue(""));
  declare_parameter("baudrate", rclcpp::ParameterValue(""));
  declare_parameter("hardware_connection", rclcpp::ParameterValue(""));
  declare_parameter("uart_dev_1", rclcpp::ParameterValue(""));
  declare_parameter("uart_dev_2", rclcpp::ParameterValue(""));

  declare_parameter("data_frequency.imu", 1);
  declare_parameter("data_frequency.timestamp", 1);
  declare_parameter("data_frequency.attitude", 1);
  declare_parameter("data_frequency.acceleration", 1);
  declare_parameter("data_frequency.velocity", 1);
  declare_parameter("data_frequency.angular_velocity", 1);
  declare_parameter("data_frequency.position", 1);
  declare_parameter("data_frequency.gps_data", 1);
  declare_parameter("data_frequency.rtk_data", 1);
  declare_parameter("data_frequency.magnetometer", 1);
  declare_parameter("data_frequency.rc_channels_data", 1);
  declare_parameter("data_frequency.gimbal_data", 1);
  declare_parameter("data_frequency.flight_status", 1);
  declare_parameter("data_frequency.battery_level", 1);
  declare_parameter("data_frequency.control_information", 1);
}
PSDKWrapper::~PSDKWrapper() {}

nav2_util::CallbackReturn
PSDKWrapper::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Configuring PSDKWrapper");
  load_parameters();
  if (!set_environment())
  {
    return nav2_util::CallbackReturn::FAILURE;
  }
  initialize_ros_elements();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PSDKWrapper::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating PSDKWrapper");

  T_DjiUserInfo user_info;
  set_user_info(&user_info);

  if (!init(&user_info))
  {
    return nav2_util::CallbackReturn::FAILURE;
  }

  activate_ros_elements();

  if (!init_telemetry() || !init_flight_control())
  {
    return nav2_util::CallbackReturn::FAILURE;
  }

  subscribe_psdk_topics();

  if (!init_camera_manager())
  {
    return nav2_util::CallbackReturn::FAILURE;
  }
  if (!init_liveview_manager())
  {
    return nav2_util::CallbackReturn::FAILURE;
  }
  if (!init_gimbal_manager())
  {
    return nav2_util::CallbackReturn::FAILURE;
  }
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PSDKWrapper::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Deactivating PSDKWrapper");
  unsubscribe_psdk_topics();
  deactivate_ros_elements();

  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PSDKWrapper::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaning up PSDKWrapper");
  clean_ros_elements();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PSDKWrapper::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  int deinit_result = DjiFlightController_Deinit() ^
                      DjiFcSubscription_DeInit() ^ DjiCore_DeInit();
  if (deinit_result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    return nav2_util::CallbackReturn::FAILURE;
  }
  global_ptr_.reset();
  RCLCPP_INFO(get_logger(), "Shutting down PSDKWrapper");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool
PSDKWrapper::set_environment()
{
  RCLCPP_INFO(get_logger(), "Setting environment");
  T_DjiReturnCode return_code;
  T_DjiOsalHandler osal_handler;
  T_DjiHalUartHandler uart_handler;
  T_DjiFileSystemHandler file_system_handler;
  T_DjiSocketHandler socket_handler;

  socket_handler.Socket = Osal_Socket;
  socket_handler.Bind = Osal_Bind;
  socket_handler.Close = Osal_Close;
  socket_handler.UdpSendData = Osal_UdpSendData;
  socket_handler.UdpRecvData = Osal_UdpRecvData;
  socket_handler.TcpListen = Osal_TcpListen;
  socket_handler.TcpAccept = Osal_TcpAccept;
  socket_handler.TcpConnect = Osal_TcpConnect;
  socket_handler.TcpSendData = Osal_TcpSendData;
  socket_handler.TcpRecvData = Osal_TcpRecvData;

  osal_handler.TaskCreate = Osal_TaskCreate;
  osal_handler.TaskDestroy = Osal_TaskDestroy;
  osal_handler.TaskSleepMs = Osal_TaskSleepMs;
  osal_handler.MutexCreate = Osal_MutexCreate;
  osal_handler.MutexDestroy = Osal_MutexDestroy;
  osal_handler.MutexLock = Osal_MutexLock;
  osal_handler.MutexUnlock = Osal_MutexUnlock;
  osal_handler.SemaphoreCreate = Osal_SemaphoreCreate;
  osal_handler.SemaphoreDestroy = Osal_SemaphoreDestroy;
  osal_handler.SemaphoreWait = Osal_SemaphoreWait;
  osal_handler.SemaphoreTimedWait = Osal_SemaphoreTimedWait;
  osal_handler.SemaphorePost = Osal_SemaphorePost;
  osal_handler.Malloc = Osal_Malloc;
  osal_handler.Free = Osal_Free;
  osal_handler.GetTimeMs = Osal_GetTimeMs;
  osal_handler.GetTimeUs = Osal_GetTimeUs;

  uart_handler.UartInit = HalUart_Init;
  uart_handler.UartDeInit = HalUart_DeInit;
  uart_handler.UartWriteData = HalUart_WriteData;
  uart_handler.UartReadData = HalUart_ReadData;
  uart_handler.UartGetStatus = HalUart_GetStatus;

  file_system_handler.FileOpen = Osal_FileOpen;
  file_system_handler.FileClose = Osal_FileClose;
  file_system_handler.FileWrite = Osal_FileWrite;
  file_system_handler.FileRead = Osal_FileRead;
  file_system_handler.FileSync = Osal_FileSync;
  file_system_handler.FileSeek = Osal_FileSeek;
  file_system_handler.DirOpen = Osal_DirOpen;
  file_system_handler.DirClose = Osal_DirClose;
  file_system_handler.DirRead = Osal_DirRead;
  file_system_handler.Mkdir = Osal_Mkdir;
  file_system_handler.Unlink = Osal_Unlink;
  file_system_handler.Rename = Osal_Rename;
  file_system_handler.Stat = Osal_Stat;

  return_code = DjiPlatform_RegOsalHandler(&osal_handler);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Register OSAL handler error. Error code is: %ld",
                 return_code);
    return false;
  }
  RCLCPP_INFO(get_logger(), "Registered OSAL handler");

  return_code = DjiPlatform_RegHalUartHandler(&uart_handler);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Register HAL handler error. Error code is: %ld",
                 return_code);
    return false;
  }
  RCLCPP_INFO(get_logger(), "Registered HAL handler");

#if (HARDWARE_CONNECTION == DJI_USE_UART_AND_USB_BULK_DEVICE)
  RCLCPP_INFO(get_logger(), "Using DJI_USE_UART_AND_USB_BULK_DEVICE");
  T_DjiHalUsbBulkHandler usb_bulk_handler;
  usb_bulk_handler.UsbBulkInit = HalUsbBulk_Init;
  usb_bulk_handler.UsbBulkDeInit = HalUsbBulk_DeInit;
  usb_bulk_handler.UsbBulkWriteData = HalUsbBulk_WriteData;
  usb_bulk_handler.UsbBulkReadData = HalUsbBulk_ReadData;
  usb_bulk_handler.UsbBulkGetDeviceInfo = HalUsbBulk_GetDeviceInfo;
  return_code = DjiPlatform_RegHalUsbBulkHandler(&usb_bulk_handler);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Register HAL USB BULK handler error. Error code is: %ld",
                 return_code);
    return false;
  }
#elif (HARDWARE_CONNECTION == DJI_USE_UART_AND_NETWORK_DEVICE)
  RCLCPP_INFO(get_logger(), "Using DJI_USE_UART_AND_NETWORK_DEVICE");
  T_DjiHalNetworkHandler network_handler;
  network_handler.NetworkInit = HalNetWork_Init;
  network_handler.NetworkDeInit = HalNetWork_DeInit;
  network_handler.NetworkGetDeviceInfo = HalNetWork_GetDeviceInfo;
  return_code = DjiPlatform_RegHalNetworkHandler(&network_handler);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Register HAL Network handler error. Error code is: %ld",
                 return_code);
  }
#elif (HARDWARE_CONNECTION == DJI_USE_ONLY_UART)
  RCLCPP_INFO(get_logger(), "Using DJI_USE_ONLY_UART");
  /*!< Attention: Only use uart hardware connection.
   */
#endif

  // Attention: if you want to use camera stream view function, please uncomment
  // it.
  return_code = DjiPlatform_RegSocketHandler(&socket_handler);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Register OSAL SOCKET handler error. Error code is: %ld",
                 return_code);
    return false;
  }

  return_code = DjiPlatform_RegFileSystemHandler(&file_system_handler);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Register OSAL filesystem handler error.Error code is: %ld",
                 return_code);
    return false;
  }
  RCLCPP_INFO(get_logger(), "Environment has been set!");
  return true;
}

void
PSDKWrapper::load_parameters()
{
  RCLCPP_INFO(get_logger(), "Loading parameters");
  if (!get_parameter("app_name", params_.app_name))
  {
    RCLCPP_ERROR(get_logger(), "app_name param not defined");
    exit(-1);
  }
  if (!get_parameter("app_id", params_.app_id))
  {
    RCLCPP_ERROR(get_logger(), "app_id param not defined");
    exit(-1);
  }
  if (!get_parameter("app_key", params_.app_key))
  {
    RCLCPP_ERROR(get_logger(), "app_key param not defined");
    exit(-1);
  }
  if (!get_parameter("app_license", params_.app_license))
  {
    RCLCPP_ERROR(get_logger(), "app_license param not defined");
    exit(-1);
  }
  if (!get_parameter("developer_account", params_.developer_account))
  {
    RCLCPP_ERROR(get_logger(), "developer_account param not defined");
    exit(-1);
  }
  if (!get_parameter("baudrate", params_.baudrate))
  {
    RCLCPP_ERROR(get_logger(), "baudrate param not defined");
    exit(-1);
  }
  RCLCPP_INFO(get_logger(), "Baudrate: %s", params_.baudrate.c_str());
  if (!get_parameter("hardware_connection", params_.hardware_connection))
  {
    RCLCPP_ERROR(get_logger(), "hardware_connection param not defined");
    exit(-1);
  }
#define HARDWARE_CONNECTION params_.hardware_connection;
  RCLCPP_INFO(get_logger(), "Hardware connection: %s",
              params_.hardware_connection.c_str());

  if (!get_parameter("uart_dev_1", params_.uart_dev_1))
  {
    RCLCPP_ERROR(get_logger(), "uart_dev_1 param not defined");
    exit(-1);
  }
  const char *name = "UART_DEV_1";
  setenv(name, params_.uart_dev_1.c_str(), 1);
  RCLCPP_INFO(get_logger(), "Uart dev 1: %s", params_.uart_dev_1.c_str());

  if (!get_parameter("uart_dev_2", params_.uart_dev_2))
  {
    RCLCPP_ERROR(get_logger(), "uart_dev_2 param not defined");
    exit(-1);
  }
  name = "UART_DEV_2";
  setenv(name, params_.uart_dev_2.c_str(), 1);
  RCLCPP_INFO(get_logger(), "Uart dev 2: %s", params_.uart_dev_2.c_str());

  // Get data frequency
  if (!get_parameter("data_frequency.imu", params_.imu_frequency))
  {
    RCLCPP_ERROR(get_logger(), "imu frequency param not defined");
    exit(-1);
  }
  if (params_.imu_frequency > IMU_TOPIC_MAX_FREQ)
  {
    RCLCPP_WARN(
        get_logger(),
        "Frequency defined for the imu topics is higher than the maximum "
        "allowed %d. Tha maximum value is set",
        IMU_TOPIC_MAX_FREQ);
    params_.imu_frequency = IMU_TOPIC_MAX_FREQ;
  }

  if (!get_parameter("data_frequency.attitude", params_.attitude_frequency))
  {
    RCLCPP_ERROR(get_logger(), "attitude param not defined");
    exit(-1);
  }
  if (params_.attitude_frequency > ATTITUDE_TOPICS_MAX_FREQ)
  {
    RCLCPP_WARN(
        get_logger(),
        "Frequency defined for the attitude topics is higher than the maximum "
        "allowed %d. Tha maximum value is set",
        ATTITUDE_TOPICS_MAX_FREQ);
    params_.attitude_frequency = ATTITUDE_TOPICS_MAX_FREQ;
  }

  if (!get_parameter("data_frequency.acceleration",
                     params_.acceleration_frequency))
  {
    RCLCPP_ERROR(get_logger(), "acceleration param not defined");
    exit(-1);
  }
  if (params_.acceleration_frequency > ACCELERATION_TOPICS_MAX_FREQ)
  {
    RCLCPP_WARN(get_logger(),
                "Frequency defined for the acceleration topics is higher than "
                "the maximum "
                "allowed %d. Tha maximum value is set",
                ACCELERATION_TOPICS_MAX_FREQ);
    params_.acceleration_frequency = ACCELERATION_TOPICS_MAX_FREQ;
  }

  if (!get_parameter("data_frequency.velocity", params_.velocity_frequency))
  {
    RCLCPP_ERROR(get_logger(), "velocity param not defined");
    exit(-1);
  }
  if (params_.velocity_frequency > VELOCITY_TOPICS_MAX_FREQ)
  {
    RCLCPP_WARN(
        get_logger(),
        "Frequency defined for the velocity topics is higher than the maximum "
        "allowed %d. Tha maximum value is set",
        VELOCITY_TOPICS_MAX_FREQ);
    params_.velocity_frequency = VELOCITY_TOPICS_MAX_FREQ;
  }

  if (!get_parameter("data_frequency.angular_velocity",
                     params_.angular_velocity_frequency))
  {
    RCLCPP_ERROR(get_logger(), "angular_velocity param not defined");
    exit(-1);
  }
  if (params_.angular_velocity_frequency > ANGULAR_VELOCITY_TOPICS_MAX_FREQ)
  {
    RCLCPP_WARN(get_logger(),
                "Frequency defined for the angular velocity topics is higher "
                "than the maximum "
                "allowed %d. Tha maximum value is set",
                ANGULAR_VELOCITY_TOPICS_MAX_FREQ);
    params_.angular_velocity_frequency = ANGULAR_VELOCITY_TOPICS_MAX_FREQ;
  }

  if (!get_parameter("data_frequency.position", params_.position_frequency))
  {
    RCLCPP_ERROR(get_logger(), "position param not defined");
    exit(-1);
  }
  if (params_.position_frequency > POSITION_TOPICS_MAX_FREQ)
  {
    RCLCPP_WARN(
        get_logger(),
        "Frequency defined for the position topics is higher than the maximum "
        "allowed %d. Tha maximum value is set",
        POSITION_TOPICS_MAX_FREQ);
    params_.position_frequency = POSITION_TOPICS_MAX_FREQ;
  }

  if (!get_parameter("data_frequency.gps_data", params_.gps_data_frequency))
  {
    RCLCPP_ERROR(get_logger(), "gps_data param not defined");
    exit(-1);
  }
  if (params_.gps_data_frequency > GPS_DATA_TOPICS_MAX_FREQ)
  {
    RCLCPP_WARN(
        get_logger(),
        "Frequency defined for the GPS topics is higher than the maximum "
        "allowed %d. Tha maximum value is set",
        GPS_DATA_TOPICS_MAX_FREQ);
    params_.gps_data_frequency = GPS_DATA_TOPICS_MAX_FREQ;
  }

  if (!get_parameter("data_frequency.rtk_data", params_.rtk_data_frequency))
  {
    RCLCPP_ERROR(get_logger(), "rtk_data param not defined");
    exit(-1);
  }
  if (params_.rtk_data_frequency > RTK_DATA_TOPICS_MAX_FREQ)
  {
    RCLCPP_WARN(
        get_logger(),
        "Frequency defined for the RTK topics is higher than the maximum "
        "allowed %d. Tha maximum value is set",
        RTK_DATA_TOPICS_MAX_FREQ);
    params_.rtk_data_frequency = RTK_DATA_TOPICS_MAX_FREQ;
  }

  if (!get_parameter("data_frequency.magnetometer",
                     params_.magnetometer_frequency))
  {
    RCLCPP_ERROR(get_logger(), "magnetometer param not defined");
    exit(-1);
  }
  if (params_.magnetometer_frequency > MAGNETOMETER_TOPICS_MAX_FREQ)
  {
    RCLCPP_WARN(get_logger(),
                "Frequency defined for the magnetometer topics is higher than "
                "the maximum "
                "allowed %d. Tha maximum value is set",
                MAGNETOMETER_TOPICS_MAX_FREQ);
    params_.magnetometer_frequency = MAGNETOMETER_TOPICS_MAX_FREQ;
  }

  if (!get_parameter("data_frequency.rc_channels_data",
                     params_.rc_channels_data_frequency))
  {
    RCLCPP_ERROR(get_logger(), "rc_channels_data param not defined");
    exit(-1);
  }
  if (params_.rc_channels_data_frequency > RC_CHANNELS_TOPICS_MAX_FREQ)
  {
    RCLCPP_WARN(get_logger(),
                "Frequency defined for the RC channel topics is higher than "
                "the maximum "
                "allowed %d. Tha maximum value is set",
                RC_CHANNELS_TOPICS_MAX_FREQ);
    params_.rc_channels_data_frequency = RC_CHANNELS_TOPICS_MAX_FREQ;
  }

  if (!get_parameter("data_frequency.gimbal_data",
                     params_.gimbal_data_frequency))
  {
    RCLCPP_ERROR(get_logger(), "gimbal_data param not defined");
    exit(-1);
  }
  if (params_.gimbal_data_frequency > GIMBAL_DATA_TOPICS_MAX_FREQ)
  {
    RCLCPP_WARN(
        get_logger(),
        "Frequency defined for the gimbal topics is higher than the maximum "
        "allowed %d. Tha maximum value is set",
        GIMBAL_DATA_TOPICS_MAX_FREQ);
    params_.gimbal_data_frequency = GIMBAL_DATA_TOPICS_MAX_FREQ;
  }

  if (!get_parameter("data_frequency.flight_status",
                     params_.flight_status_frequency))
  {
    RCLCPP_ERROR(get_logger(), "flight_status param not defined");
    exit(-1);
  }
  if (params_.flight_status_frequency > FLIGHT_STATUS_TOPICS_MAX_FREQ)
  {
    RCLCPP_WARN(get_logger(),
                "Frequency defined for the flight status topics is higher than "
                "the maximum "
                "allowed %d. Tha maximum value is set",
                FLIGHT_STATUS_TOPICS_MAX_FREQ);
    params_.flight_status_frequency = FLIGHT_STATUS_TOPICS_MAX_FREQ;
  }

  if (!get_parameter("data_frequency.battery_level",
                     params_.battery_level_frequency))
  {
    RCLCPP_ERROR(get_logger(), "battery_level param not defined");
    exit(-1);
  }
  if (params_.battery_level_frequency > BATTERY_STATUS_TOPICS_MAX_FREQ)
  {
    RCLCPP_WARN(get_logger(),
                "Frequency defined for the battery status topics is higher "
                "than the maximum "
                "allowed %d. Tha maximum value is set",
                BATTERY_STATUS_TOPICS_MAX_FREQ);
    params_.battery_level_frequency = BATTERY_STATUS_TOPICS_MAX_FREQ;
  }

  if (!get_parameter("data_frequency.control_information",
                     params_.control_information_frequency))
  {
    RCLCPP_ERROR(get_logger(), "control_information param not defined");
    exit(-1);
  }
  if (params_.control_information_frequency > CONTROL_DATA_TOPICS_MAX_FREQ)
  {
    RCLCPP_WARN(
        get_logger(),
        "Frequency defined for the control topics is higher than the maximum "
        "allowed %d. Tha maximum value is set",
        CONTROL_DATA_TOPICS_MAX_FREQ);
    params_.control_information_frequency = CONTROL_DATA_TOPICS_MAX_FREQ;
  }
}

bool
PSDKWrapper::set_user_info(T_DjiUserInfo *user_info)
{
  memset(user_info->appName, 0, sizeof(user_info->appName));
  memset(user_info->appId, 0, sizeof(user_info->appId));
  memset(user_info->appKey, 0, sizeof(user_info->appKey));
  memset(user_info->appLicense, 0, sizeof(user_info->appLicense));
  memset(user_info->developerAccount, 0, sizeof(user_info->developerAccount));
  memset(user_info->baudRate, 0, sizeof(user_info->baudRate));

  if (strlen(params_.app_name.c_str()) >= sizeof(user_info->appName) ||
      strlen(params_.app_id.c_str()) > sizeof(user_info->appId) ||
      strlen(params_.app_key.c_str()) > sizeof(user_info->appKey) ||
      strlen(params_.app_license.c_str()) > sizeof(user_info->appLicense) ||
      strlen(params_.developer_account.c_str()) >=
          sizeof(user_info->developerAccount) ||
      strlen(params_.baudrate.c_str()) > sizeof(user_info->baudRate))
  {
    RCLCPP_ERROR(get_logger(), "User information set is out of bounds");
    return false;
  }

  strncpy(user_info->appName, params_.app_name.c_str(),
          sizeof(user_info->appName) - 1);
  memcpy(user_info->appId, params_.app_id.c_str(),
         strlen(params_.app_id.c_str()));
  memcpy(user_info->appKey, params_.app_key.c_str(),
         strlen(params_.app_key.c_str()));
  memcpy(user_info->appLicense, params_.app_license.c_str(),
         strlen(params_.app_license.c_str()));
  strncpy(user_info->developerAccount, params_.developer_account.c_str(),
          sizeof(user_info->developerAccount) - 1);
  memcpy(user_info->baudRate, params_.baudrate.c_str(),
         strlen(params_.baudrate.c_str()));

  return true;
}

bool
PSDKWrapper::init(T_DjiUserInfo *user_info)
{
  RCLCPP_INFO(get_logger(), "Init DJI Core...");
  auto result = DjiCore_Init(user_info);
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "DJI core could not be initiated. Error code is: %ld", result);
    return false;
  }

  T_DjiAircraftInfoBaseInfo aircraft_base_info;
  if (DjiAircraftInfo_GetBaseInfo(&aircraft_base_info) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not get aircraft information.");
    return false;
  }

  if (aircraft_base_info.mountPosition != DJI_MOUNT_POSITION_EXTENSION_PORT)
  {
    RCLCPP_ERROR(get_logger(), "Please use the extension port");
    return false;
  }

  if (DjiCore_SetAlias("PSDK_UMD") != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not set alias.");
    return false;
  }

  if (DjiCore_ApplicationStart() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not start application.");
    return false;
  }
  return true;
}

void
PSDKWrapper::initialize_ros_elements()
{
  RCLCPP_INFO(get_logger(), "Initializing ROS publishers");
  attitude_pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>(
      "dji_psdk_ros/attitude", 10);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("dji_psdk_ros/imu", 10);
  velocity_ground_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "dji_psdk_ros/velocity_ground_ENU", 10);
  position_fused_pub_ =
      create_publisher<umd_psdk_interfaces::msg::PositionFused>(
          "dji_psdk_ros/position_fused", 10);
  gps_fused_pub_ = create_publisher<umd_psdk_interfaces::msg::GPSFused>(
      "dji_psdk_ros/gps_fused", 10);
  gps_position_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      "dji_psdk_ros/gps_position", 10);
  gps_velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "dji_psdk_ros/gps_velocity", 10);
  gps_details_pub_ = create_publisher<umd_psdk_interfaces::msg::GPSDetails>(
      "dji_psdk_ros/gps_details", 10);
  gps_signal_pub_ = create_publisher<std_msgs::msg::UInt8>(
      "dji_psdk_ros/gps_signal_level", 10);
  gps_control_pub_ = create_publisher<std_msgs::msg::UInt8>(
      "dji_psdk_ros/gps_control_level", 10);
  rtk_position_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      "dji_psdk_ros/rtk_position", 10);
  rtk_velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "dji_psdk_ros/rtk_velocity", 10);
  rtk_yaw_pub_ = create_publisher<umd_psdk_interfaces::msg::RTKYaw>(
      "dji_psdk_ros/rtk_yaw", 10);
  rtk_position_info_pub_ = create_publisher<std_msgs::msg::UInt8>(
      "dji_psdk_ros/rtk_position_info", 10);
  rtk_yaw_info_pub_ = create_publisher<std_msgs::msg::UInt8>(
      "dji_psdk_ros/rtk_position_info", 10);
  magnetic_field_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(
      "dji_psdk_ros/magnetic_field", 10);
  rc_pub_ = create_publisher<sensor_msgs::msg::Joy>("dji_psdk_ros/rc", 10);
  gimbal_angles_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "dji_psdk_ros/gimbal_angles", 10);
  gimbal_status_pub_ = create_publisher<umd_psdk_interfaces::msg::GimbalStatus>(
      "dji_psdk_ros/gimbal_status", 10);
  flight_status_pub_ = create_publisher<umd_psdk_interfaces::msg::FlightStatus>(
      "dji_psdk_ros/flight_status", 10);
  aircraft_status_pub_ =
      create_publisher<umd_psdk_interfaces::msg::AircraftStatus>(
          "dji_psdk_ros/aircraft_status", 10);
  landing_gear_pub_ = create_publisher<std_msgs::msg::UInt8>(
      "dji_psdk_ros/landing_gear_status", 10);
  motor_start_error_pub_ = create_publisher<std_msgs::msg::UInt16>(
      "dji_psdk_ros/motor_start_error", 10);
  flight_anomaly_pub_ =
      create_publisher<umd_psdk_interfaces::msg::FlightAnomaly>(
          "dji_psdk_ros/flight_anomaly", 10);
  battery_pub_ = create_publisher<umd_psdk_interfaces::msg::Battery>(
      "dji_psdk_ros/battery", 10);
  height_fused_pub_ =
      create_publisher<std_msgs::msg::Float32>("dji_psdk_ros/height_fused", 10);

  /** @todo Implement other useful publishers */
  // acceleration_ground_pub_ =
  // create_publisher<geometry_msgs::msg::AccelStamped>(
  //     "dji_psdk_ros/acceleration_ground", 10);
  // acceleration_body_pub_ =
  // create_publisher<geometry_msgs::msg::AccelStamped>(
  //     "dji_psdk_ros/acceleration_body", 10);
  // altitude_pub_ =
  //     create_publisher<umd_psdk_interfaces::msg::Altitude>("dji_psdk_ros/altitude",
  //     10);
  // relative_height_pub_ =
  //     create_publisher<std_msgs::msg::Float32>("dji_psdk_ros/relative_height",
  //     10);
  // relative_obstacle_info_pub_ =
  //     create_publisher<umd_psdk_interfaces::msg::RelativeObstacleInfo>(
  //         "dji_psdk_ros/relative_obstacle_info", 10);
  // home_position_pub_ =
  // create_publisher<umd_psdk_interfaces::msg::HomePosition>(
  //     "dji_psdk_ros/home_position", 10);

  RCLCPP_INFO(get_logger(), "Creating subscribers");
  flight_control_generic_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "dji_psdk_ros/flight_control_setpoint_generic", 10,
      std::bind(&PSDKWrapper::flight_control_generic_cb, this, _1));
  flight_control_position_yaw_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "dji_psdk_ros/flight_control_setpoint_ENUposition_yaw", 10,
      std::bind(&PSDKWrapper::flight_control_position_yaw_cb, this, _1));
  flight_control_velocity_yawrate_sub_ =
      create_subscription<sensor_msgs::msg::Joy>(
          "dji_psdk_ros/flight_control_setpoint_ENUvelocity_yawrate", 10,
          std::bind(&PSDKWrapper::flight_control_velocity_yawrate_cb, this,
                    _1));
  flight_control_body_velocity_yawrate_sub_ =
      create_subscription<sensor_msgs::msg::Joy>(
          "dji_psdk_ros/flight_control_setpoint_FRUvelocity_yawrate", 10,
          std::bind(&PSDKWrapper::flight_control_body_velocity_yawrate_cb, this,
                    _1));
  flight_control_rollpitch_yawrate_vertpos_sub_ =
      create_subscription<sensor_msgs::msg::Joy>(
          "dji_psdk_ros/flight_control_setpoint_rollpitch_yawrate_zposition",
          10,
          std::bind(&PSDKWrapper::flight_control_rollpitch_yawrate_vertpos_cb,
                    this, _1));

  RCLCPP_INFO(get_logger(), "Creating services");
  set_home_from_gps_srv_ = create_service<SetHomeFromGPS>(
      "set_home_from_gps",
      std::bind(&PSDKWrapper::set_home_from_gps_cb, this, _1, _2));
  set_home_from_current_location_srv_ = create_service<Trigger>(
      "set_home_from_current_location",
      std::bind(&PSDKWrapper::set_home_from_current_location_cb, this, _1, _2));
  set_home_altitude_srv_ = create_service<SetHomeAltitude>(
      "set_home_altitude",
      std::bind(&PSDKWrapper::set_home_altitude_cb, this, _1, _2));
  get_home_altitude_srv_ = create_service<GetHomeAltitude>(
      "get_home_altitude",
      std::bind(&PSDKWrapper::get_home_altitude_cb, this, _1, _2));
  start_go_home_srv_ = create_service<Trigger>(
      "start_go_home", std::bind(&PSDKWrapper::start_go_home_cb, this, _1, _2));
  cancel_go_home_srv_ = create_service<Trigger>(
      "cancel_go_home",
      std::bind(&PSDKWrapper::cancel_go_home_cb, this, _1, _2));
  obtain_ctrl_authority_srv_ = create_service<Trigger>(
      "obtain_ctrl_authority",
      std::bind(&PSDKWrapper::obtain_ctrl_authority_cb, this, _1, _2));
  release_ctrl_authority_srv_ = create_service<Trigger>(
      "release_ctrl_authority",
      std::bind(&PSDKWrapper::release_ctrl_authority_cb, this, _1, _2));
  turn_on_motors_srv_ = create_service<Trigger>(
      "turn_on_motors",
      std::bind(&PSDKWrapper::turn_on_motors_cb, this, _1, _2));
  turn_off_motors_srv_ = create_service<Trigger>(
      "turn_off_motors",
      std::bind(&PSDKWrapper::turn_off_motors_cb, this, _1, _2));
  takeoff_srv_ = create_service<Trigger>(
      "takeoff", std::bind(&PSDKWrapper::start_takeoff_cb, this, _1, _2));
  land_srv_ = create_service<Trigger>(
      "land", std::bind(&PSDKWrapper::start_landing_cb, this, _1, _2));
  cancel_landing_srv_ = create_service<Trigger>(
      "cancel_landing",
      std::bind(&PSDKWrapper::cancel_landing_cb, this, _1, _2));
  start_confirm_landing_srv_ = create_service<Trigger>(
      "start_confirm_landing",
      std::bind(&PSDKWrapper::start_confirm_landing_cb, this, _1, _2));
  start_force_landing_srv_ = create_service<Trigger>(
      "start_force_landing",
      std::bind(&PSDKWrapper::start_force_landing_cb, this, _1, _2));
  set_horizontal_vo_obstacle_avoidance_srv_ =
      create_service<SetObstacleAvoidance>(
          "set_horizontal_vo_obstacle_avoidance",
          std::bind(&PSDKWrapper::set_horizontal_vo_obstacle_avoidance_cb, this,
                    _1, _2));
  set_horizontal_radar_obstacle_avoidance_srv_ =
      create_service<SetObstacleAvoidance>(
          "set_horizontal_radar_obstacle_avoidance",
          std::bind(&PSDKWrapper::set_horizontal_radar_obstacle_avoidance_cb,
                    this, _1, _2));
  set_upwards_vo_obstacle_avoidance_srv_ = create_service<SetObstacleAvoidance>(
      "set_upwards_vo_obstacle_avoidance",
      std::bind(&PSDKWrapper::set_upwards_vo_obstacle_avoidance_cb, this, _1,
                _2));
  set_upwards_radar_obstacle_avoidance_srv_ =
      create_service<SetObstacleAvoidance>(
          "set_upwards_radar_obstacle_avoidance",
          std::bind(&PSDKWrapper::set_upwards_radar_obstacle_avoidance_cb, this,
                    _1, _2));
  set_downwards_vo_obstacle_avoidance_srv_ =
      create_service<SetObstacleAvoidance>(
          "set_downwards_vo_obstacle_avoidance",
          std::bind(&PSDKWrapper::set_downwards_vo_obstacle_avoidance_cb, this,
                    _1, _2));
  get_horizontal_vo_obstacle_avoidance_srv_ =
      create_service<GetObstacleAvoidance>(
          "get_horizontal_vo_obstacle_avoidance",
          std::bind(&PSDKWrapper::get_horizontal_vo_obstacle_avoidance_cb, this,
                    _1, _2));
  get_upwards_vo_obstacle_avoidance_srv_ = create_service<GetObstacleAvoidance>(
      "get_upwards_vo_obstacle_avoidance",
      std::bind(&PSDKWrapper::get_upwards_vo_obstacle_avoidance_cb, this, _1,
                _2));
  get_upwards_radar_obstacle_avoidance_srv_ =
      create_service<GetObstacleAvoidance>(
          "get_upwards_radar_obstacle_avoidance",
          std::bind(&PSDKWrapper::get_upwards_radar_obstacle_avoidance_cb, this,
                    _1, _2));
  get_downwards_vo_obstacle_avoidance_srv_ =
      create_service<GetObstacleAvoidance>(
          "get_downwards_vo_obstacle_avoidance",
          std::bind(&PSDKWrapper::get_downwards_vo_obstacle_avoidance_cb, this,
                    _1, _2));
  get_horizontal_radar_obstacle_avoidance_srv_ =
      create_service<GetObstacleAvoidance>(
          "get_horizontal_radar_obstacle_avoidance",
          std::bind(&PSDKWrapper::get_horizontal_radar_obstacle_avoidance_cb,
                    this, _1, _2));
  // Camera
  camera_start_shoot_single_photo_service_= 
      create_service<CameraStartShootSinglePhoto>(
      "camera_start_shoot_single_photo",
      std::bind(&PSDKWrapper::camera_start_shoot_single_photo_callback_, this, _1, _2),
      qos_profile_);
  camera_start_shoot_burst_photo_service_ = 
      create_service<CameraStartShootBurstPhoto>(
      "camera_start_shoot_burst_photo",
      std::bind(&PSDKWrapper::camera_start_shoot_burst_photo_callback_, this, _1, _2),
      qos_profile_);
  camera_start_shoot_aeb_photo_service_ = create_service<CameraStartShootAEBPhoto>(
      "camera_start_shoot_aeb_photo",
      std::bind(&PSDKWrapper::camera_start_shoot_aeb_photo_callback_,
      this, _1, _2),
      qos_profile_);
  camera_start_shoot_interval_photo_service_ = 
      create_service<CameraStartShootIntervalPhoto>(
      "camera_start_shoot_interval_photo",
      std::bind(&PSDKWrapper::camera_start_shoot_interval_photo_callback_, this, _1, _2),
      qos_profile_);
  camera_stop_shoot_photo_service_ = create_service<CameraStopShootPhoto>(
      "camera_stop_shoot_photo",
      std::bind(&PSDKWrapper::camera_stop_shoot_photo_callback_, this, _1, _2),
      qos_profile_);
  camera_record_video_service_ = create_service<CameraRecordVideo>(
      "camera_record_video",
      std::bind(&PSDKWrapper::camera_record_video_callback_, this, _1, _2),
      qos_profile_);
  camera_get_laser_ranging_info_service_ = create_service<CameraGetLaserRangingInfo>(
      "camera_get_laser_ranging_info",
      std::bind(&PSDKWrapper::camera_get_laser_ranging_info_callback_, this, _1, _2),
      qos_profile_);
  // TODO(@lidiadltv): Enable these actions once are working properly
  // camera_download_file_list_action_ =
  //     std::make_unique<nav2_util::SimpleActionServer<CameraDownloadFileList>>(
  //           shared_from_this(), "camera_download_file_list",
  //           std::bind(&PSDKWrapper::camera_download_file_list_callback_,
  //           this));
  // camera_download_file_by_index_action_ =
  //     std::make_unique<nav2_util::SimpleActionServer<CameraDownloadFileByIndex>>(
  //           shared_from_this(), "camera_download_file_by_index",
  //           std::bind(&PSDKWrapper::camera_download_file_by_index_callback_,
  //           this));
  // camera_delete_file_by_index_action_ =
  //     std::make_unique<nav2_util::SimpleActionServer<CameraDeleteFileByIndex>>(
  //           shared_from_this(), "camera_delete_file_by_index",
  //           std::bind(&PSDKWrapper::camera_delete_file_by_index_callback_,
  //           this));
  camera_streaming_service_ = create_service<CameraStreaming>(
      "camera_streaming",
      std::bind(&PSDKWrapper::camera_streaming_callback_, this, _1, _2),
      qos_profile_);
  camera_get_type_service_ = create_service<CameraGetType>(
      "camera_get_type",
      std::bind(&PSDKWrapper::camera_get_type_callback_, this, _1, _2),
      qos_profile_);
  camera_set_ev_service_ = create_service<CameraSetEV>(
      "camera_set_ev",
      std::bind(&PSDKWrapper::camera_set_ev_callback_, this, _1, _2),
      qos_profile_);
  camera_get_ev_service_ = create_service<CameraGetEV>(
      "camera_get_ev",
      std::bind(&PSDKWrapper::camera_get_ev_callback_, this, _1, _2),
      qos_profile_);
  camera_set_shutter_speed_service_ = create_service<CameraSetShutterSpeed>(
      "camera_set_shutter_speed",
      std::bind(&PSDKWrapper::camera_set_shutter_speed_callback_, this, _1, _2),
      qos_profile_);
  camera_get_shutter_speed_service_ = create_service<CameraGetShutterSpeed>(
      "camera_get_shutter_speed",
      std::bind(&PSDKWrapper::camera_get_shutter_speed_callback_, this, _1, _2),
      qos_profile_);
  camera_set_iso_service_ = create_service<CameraSetISO>(
      "camera_set_iso",
      std::bind(&PSDKWrapper::camera_set_iso_callback_, this, _1, _2),
      qos_profile_);
  camera_get_iso_service_ = create_service<CameraGetISO>(
      "camera_get_iso",
      std::bind(&PSDKWrapper::camera_get_iso_callback_, this, _1, _2),
      qos_profile_);
  camera_set_focus_target_service_ = create_service<CameraSetFocusTarget>(
      "camera_set_focus_target",
      std::bind(&PSDKWrapper::camera_set_focus_target_callback_, this, _1, _2),
      qos_profile_);
  camera_get_focus_target_service_ = create_service<CameraGetFocusTarget>(
      "camera_get_focus_target",
      std::bind(&PSDKWrapper::camera_get_focus_target_callback_, this, _1, _2),
      qos_profile_);
  camera_set_focus_mode_service_ = create_service<CameraSetFocusMode>(
      "camera_set_focus_mode",
      std::bind(&PSDKWrapper::camera_set_focus_mode_callback_, this, _1, _2),
      qos_profile_);
  camera_get_focus_mode_service_ = create_service<CameraGetFocusMode>(
      "camera_get_focus_mode",
      std::bind(&PSDKWrapper::camera_get_focus_mode_callback_, this, _1, _2),
      qos_profile_);
  camera_set_optical_zoom_service_ = create_service<CameraSetOpticalZoom>(
      "camera_set_optical_zoom",
      std::bind(&PSDKWrapper::camera_set_optical_zoom_callback_, this, _1, _2),
      qos_profile_);
  camera_get_optical_zoom_service_ = create_service<CameraGetOpticalZoom>(
      "camera_get_optical_zoom",
      std::bind(&PSDKWrapper::camera_get_optical_zoom_callback_, this, _1, _2),
      qos_profile_);
  camera_set_infrared_zoom_service_ = create_service<CameraSetInfraredZoom>(
      "camera_set_infrared_zoom",
      std::bind(&PSDKWrapper::camera_set_infrared_zoom_callback_, this, _1, _2),
      qos_profile_);
  //// Gimbal
  // Services
  gimbal_set_mode_service_ = create_service<GimbalSetMode>(
      "gimbal_set_mode",
      std::bind(&PSDKWrapper::gimbal_set_mode_callback_, this, _1, _2),
      qos_profile_);
  gimbal_reset_service_ = create_service<GimbalReset>(
      "gimbal_reset",
      std::bind(&PSDKWrapper::gimbal_reset_callback_, this, _1, _2),
      qos_profile_);
  gimbal_rotation_service_ = create_service<GimbalRotation>(
      "gimbal_rotation",
      std::bind(&PSDKWrapper::gimbal_rotation_callback_, this,  _1, _2),
      qos_profile_);
}

void
PSDKWrapper::activate_ros_elements()
{
  RCLCPP_INFO(get_logger(), "Activating ROS elements");
  attitude_pub_->on_activate();
  imu_pub_->on_activate();
  velocity_ground_pub_->on_activate();
  position_fused_pub_->on_activate();
  gps_fused_pub_->on_activate();
  gps_position_pub_->on_activate();
  gps_velocity_pub_->on_activate();
  gps_details_pub_->on_activate();
  gps_signal_pub_->on_activate();
  gps_control_pub_->on_activate();
  rtk_position_pub_->on_activate();
  rtk_velocity_pub_->on_activate();
  rtk_yaw_pub_->on_activate();
  rtk_position_info_pub_->on_activate();
  rtk_yaw_info_pub_->on_activate();
  magnetic_field_pub_->on_activate();
  rc_pub_->on_activate();
  gimbal_angles_pub_->on_activate();
  gimbal_status_pub_->on_activate();
  flight_status_pub_->on_activate();
  aircraft_status_pub_->on_activate();
  landing_gear_pub_->on_activate();
  motor_start_error_pub_->on_activate();
  flight_anomaly_pub_->on_activate();
  battery_pub_->on_activate();
  height_fused_pub_->on_activate();
  // acceleration_ground_pub_->on_activate();
  // acceleration_body_pub_->on_activate();
  // altitude_pub_->on_activate();
  // relative_height_pub_->on_activate();
  // relative_obstacle_info_pub_->on_activate();
  // home_position_pub_->on_activate();
}

void
PSDKWrapper::deactivate_ros_elements()
{
  RCLCPP_INFO(get_logger(), "Deactivating ROS elements");
  attitude_pub_->on_deactivate();
  imu_pub_->on_deactivate();
  velocity_ground_pub_->on_deactivate();
  position_fused_pub_->on_deactivate();
  gps_fused_pub_->on_deactivate();
  gps_position_pub_->on_deactivate();
  gps_velocity_pub_->on_deactivate();
  gps_details_pub_->on_deactivate();
  gps_signal_pub_->on_deactivate();
  gps_control_pub_->on_deactivate();
  rtk_position_pub_->on_deactivate();
  rtk_velocity_pub_->on_deactivate();
  rtk_yaw_pub_->on_deactivate();
  rtk_position_info_pub_->on_deactivate();
  rtk_yaw_info_pub_->on_deactivate();
  magnetic_field_pub_->on_deactivate();
  rc_pub_->on_deactivate();
  gimbal_angles_pub_->on_deactivate();
  gimbal_status_pub_->on_deactivate();
  flight_status_pub_->on_deactivate();
  aircraft_status_pub_->on_deactivate();
  motor_start_error_pub_->on_deactivate();
  landing_gear_pub_->on_deactivate();
  flight_anomaly_pub_->on_deactivate();
  battery_pub_->on_deactivate();
  height_fused_pub_->on_deactivate();
  // acceleration_ground_pub_->on_deactivate();
  // acceleration_body_pub_->on_deactivate();
  // altitude_pub_->on_deactivate();
  // relative_height_pub_->on_deactivate();
  // relative_obstacle_info_pub_->on_deactivate();
  // home_position_pub_->on_deactivate();
}

void
PSDKWrapper::clean_ros_elements()
{
  RCLCPP_INFO(get_logger(), "Cleaning ROS elements");
  attitude_pub_.reset();
  imu_pub_.reset();
  velocity_ground_pub_.reset();
  position_fused_pub_.reset();
  gps_fused_pub_.reset();
  gps_position_pub_.reset();
  gps_velocity_pub_.reset();
  gps_details_pub_.reset();
  gps_signal_pub_.reset();
  gps_control_pub_.reset();
  rtk_position_pub_.reset();
  rtk_velocity_pub_.reset();
  rtk_yaw_pub_.reset();
  rtk_position_info_pub_.reset();
  rtk_yaw_info_pub_.reset();
  magnetic_field_pub_.reset();
  rc_pub_.reset();
  gimbal_angles_pub_.reset();
  gimbal_status_pub_.reset();
  flight_status_pub_.reset();
  aircraft_status_pub_.reset();
  landing_gear_pub_.reset();
  motor_start_error_pub_.reset();
  flight_anomaly_pub_.reset();
  battery_pub_.reset();
  height_fused_pub_.reset();
  // acceleration_ground_pub_.reset();
  // acceleration_body_pub_.reset();
  // altitude_pub_.reset();
  // relative_height_pub_.reset();
  // relative_obstacle_info_pub_.reset();
  // home_position_pub_.reset();

  // Subscribers
  flight_control_generic_sub_.reset();
  flight_control_position_yaw_sub_.reset();
  flight_control_velocity_yawrate_sub_.reset();
  flight_control_body_velocity_yawrate_sub_.reset();
  flight_control_rollpitch_yawrate_vertpos_sub_.reset();

  // Services
  set_home_from_gps_srv_.reset();
  set_home_from_current_location_srv_.reset();
  set_home_altitude_srv_.reset();
  get_home_altitude_srv_.reset();
  start_go_home_srv_.reset();
  cancel_go_home_srv_.reset();
  obtain_ctrl_authority_srv_.reset();
  release_ctrl_authority_srv_.reset();
  turn_on_motors_srv_.reset();
  turn_off_motors_srv_.reset();
  takeoff_srv_.reset();
  land_srv_.reset();
  cancel_landing_srv_.reset();
  start_confirm_landing_srv_.reset();
  start_force_landing_srv_.reset();
  set_horizontal_vo_obstacle_avoidance_srv_.reset();
  set_horizontal_radar_obstacle_avoidance_srv_.reset();
  set_upwards_vo_obstacle_avoidance_srv_.reset();
  set_upwards_radar_obstacle_avoidance_srv_.reset();
  set_downwards_vo_obstacle_avoidance_srv_.reset();
  get_horizontal_vo_obstacle_avoidance_srv_.reset();
  get_upwards_vo_obstacle_avoidance_srv_.reset();
  get_upwards_radar_obstacle_avoidance_srv_.reset();
  get_downwards_vo_obstacle_avoidance_srv_.reset();
  get_horizontal_radar_obstacle_avoidance_srv_.reset();

  // Camera
  camera_start_shoot_single_photo_service_.reset();
  camera_start_shoot_burst_photo_service_.reset();
  camera_start_shoot_aeb_photo_service_.reset();
  camera_start_shoot_interval_photo_service_.reset();
  camera_stop_shoot_photo_service_.reset();
  camera_record_video_service_.reset();
  camera_get_type_service_.reset();
  camera_set_ev_service_.reset();
  camera_get_ev_service_.reset();
  camera_set_shutter_speed_service_.reset();
  camera_get_shutter_speed_service_.reset();
  camera_set_iso_service_.reset();
  camera_get_iso_service_.reset();
  camera_set_focus_target_service_.reset();
  camera_get_focus_target_service_.reset();
  camera_set_focus_mode_service_.reset();
  camera_get_focus_mode_service_.reset();
  camera_set_optical_zoom_service_.reset();
  camera_get_optical_zoom_service_.reset();
  camera_set_infrared_zoom_service_.reset();
  camera_get_laser_ranging_info_service_.reset();
  camera_download_file_list_service_.reset();
  camera_download_file_by_index_service_.reset();
  camera_delete_file_by_index_service_.reset();
  camera_streaming_service_.reset();
  // Gimbal
  gimbal_set_mode_service_.reset();
  gimbal_reset_service_.reset();
  gimbal_rotation_service_.reset();
}

}  // namespace psdk_ros2
