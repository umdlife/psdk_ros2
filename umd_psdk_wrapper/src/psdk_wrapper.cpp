/* Copyright (C) 2023 Unmanned Life - All Rights Reserved
 *
 * This file is part of the `umd_psdk_wrapper` source code package and is subject to
 * the terms and conditions defined in the file LICENSE.txt contained therein.
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

#include "umd_psdk_wrapper/psdk_wrapper.hpp"

namespace umd_psdk {
PSDKWrapper::PSDKWrapper(const std::string &node_name)
    : nav2_util::LifecycleNode(node_name, "", rclcpp::NodeOptions())
{
}

PSDKWrapper::~PSDKWrapper() {}

nav2_util::CallbackReturn
PSDKWrapper::on_configure(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "Configuring PSDKWrapper");
  umd_psdk::PSDKWrapper::on_configure(state);
  set_environment();
  load_parameters();
  initialize_ros_publishers();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PSDKWrapper::on_activate(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "Activating PSDKWrapper");
  umd_psdk::PSDKWrapper::on_activate(state);

  T_DjiUserInfo user_info;
  set_user_info(&user_info);

  if (!init(&user_info)) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  if (!init_telemetry()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PSDKWrapper::on_deactivate(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "Deactivating PSDKWrapper");
  umd_psdk::PSDKWrapper::on_deactivate(state);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PSDKWrapper::on_cleanup(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up PSDKWrapper");
  umd_psdk::PSDKWrapper::on_cleanup(state);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PSDKWrapper::on_shutdown(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "Shutting down PSDKWrapper");
  umd_psdk::PSDKWrapper::on_shutdown(state);

  if (DjiCore_DeInit() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  if (DjiFcSubscription_DeInit() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

void
PSDKWrapper::set_environment()
{
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

  file_system_handler.FileOpen = Osal_FileOpen,
  file_system_handler.FileClose = Osal_FileClose,
  file_system_handler.FileWrite = Osal_FileWrite,
  file_system_handler.FileRead = Osal_FileRead,
  file_system_handler.FileSync = Osal_FileSync,
  file_system_handler.FileSeek = Osal_FileSeek,
  file_system_handler.DirOpen = Osal_DirOpen,
  file_system_handler.DirClose = Osal_DirClose,
  file_system_handler.DirRead = Osal_DirRead, file_system_handler.Mkdir = Osal_Mkdir,
  file_system_handler.Unlink = Osal_Unlink, file_system_handler.Rename = Osal_Rename,
  file_system_handler.Stat = Osal_Stat,

  return_code = DjiPlatform_RegOsalHandler(&osal_handler);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Register OSAL handler error");
  }

  return_code = DjiPlatform_RegHalUartHandler(&uart_handler);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Register HAL handler error");
  }

#if (HARDWARE_CONNECTION == DJI_USE_UART_AND_USB_BULK_DEVICE)
  T_DjiHalUsbBulkHandler usb_bulk_handler;
  usb_bulk_handler.UsbBulkInit = HalUsbBulk_Init;
  usb_bulk_handler.UsbBulkDeInit = HalUsbBulk_DeInit;
  usb_bulk_handler.UsbBulkWriteData = HalUsbBulk_WriteData;
  usb_bulk_handler.UsbBulkReadData = HalUsbBulk_ReadData;
  usb_bulk_handler.UsbBulkGetDeviceInfo = HalUsbBulk_GetDeviceInfo;
  if (DjiPlatform_RegHalUsbBulkHandler(&usb_bulk_handler) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Register HAL USB BULK handler error");
  }
#elif (HARDWARE_CONNECTION == DJI_USE_UART_AND_NETWORK_DEVICE)
  T_DjiHalNetworkHandler network_handler;
  network_handler.NetworkInit = HalNetWork_Init;
  network_handler.NetworkDeInit = HalNetWork_DeInit;
  network_handler.NetworkGetDeviceInfo = HalNetWork_GetDeviceInfo;
  if (DjiPlatform_RegHalNetworkHandler(&network_handler) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Register HAL Network handler error");
  }
#elif (HARDWARE_CONNECTION == DJI_USE_ONLY_UART)
  /*!< Attention: Only use uart hardware connection.
   */
#endif

  // Attention: if you want to use camera stream view function, please uncomment it.
  return_code = DjiPlatform_RegSocketHandler(&socket_handler);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Register OSAL SOCKET handler error");
  }

  return_code = DjiPlatform_RegFileSystemHandler(&file_system_handler);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Register OSAL filesystem handler error");
    throw std::runtime_error("Register osal filesystem handler error.");
  }
}

void
PSDKWrapper::load_parameters()
{
  if (!get_parameter("app_name", params_.app_name)) {
    RCLCPP_ERROR(get_logger(), "app_name param not defined");
    exit(-1);
  }
  if (!get_parameter("app_id", params_.app_id)) {
    RCLCPP_ERROR(get_logger(), "app_id param not defined");
    exit(-1);
  }
  if (!get_parameter("app_key", params_.app_key)) {
    RCLCPP_ERROR(get_logger(), "app_key param not defined");
    exit(-1);
  }
  if (!get_parameter("app_license", params_.app_license)) {
    RCLCPP_ERROR(get_logger(), "app_license param not defined");
    exit(-1);
  }
  if (!get_parameter("developer_account", params_.developer_account)) {
    RCLCPP_ERROR(get_logger(), "developer_account param not defined");
    exit(-1);
  }
  if (!get_parameter("baudrate", params_.baudrate)) {
    RCLCPP_ERROR(get_logger(), "baudrate param not defined");
    exit(-1);
  }
  if (!get_parameter("hardware_connection", params_.hardware_connection)) {
    RCLCPP_ERROR(get_logger(), "hardware_connection param not defined");
    exit(-1);
  }
#define HARDWARE_CONNECTION params_.hardware_connection;

  // Get data frequency
  if (!get_parameter("timestamp", data_frequency_.timestamp)) {
    RCLCPP_ERROR(get_logger(), "timestamp param not defined");
    exit(-1);
  }
  if (!get_parameter("attitude_quaternions", data_frequency_.attitude_quaternions)) {
    RCLCPP_ERROR(get_logger(), "attitude_quaternions param not defined");
    exit(-1);
  }
  if (!get_parameter("acceleration", data_frequency_.acceleration)) {
    RCLCPP_ERROR(get_logger(), "acceleration param not defined");
    exit(-1);
  }
  if (!get_parameter("velocity", data_frequency_.velocity)) {
    RCLCPP_ERROR(get_logger(), "velocity param not defined");
    exit(-1);
  }
  if (!get_parameter("angular_velocity", data_frequency_.angular_velocity)) {
    RCLCPP_ERROR(get_logger(), "angular_velocity param not defined");
    exit(-1);
  }
  if (!get_parameter("position", data_frequency_.position)) {
    RCLCPP_ERROR(get_logger(), "position param not defined");
    exit(-1);
  }
  if (!get_parameter("gps_data", data_frequency_.gps_data)) {
    RCLCPP_ERROR(get_logger(), "gps_data param not defined");
    exit(-1);
  }
  if (!get_parameter("rtk_data", data_frequency_.rtk_data)) {
    RCLCPP_ERROR(get_logger(), "rtk_data param not defined");
    exit(-1);
  }
  if (!get_parameter("magnetometer", data_frequency_.magnetometer)) {
    RCLCPP_ERROR(get_logger(), "magnetometer param not defined");
    exit(-1);
  }
  if (!get_parameter("rc_channels_data", data_frequency_.rc_channels_data)) {
    RCLCPP_ERROR(get_logger(), "rc_channels_data param not defined");
    exit(-1);
  }
  if (!get_parameter("gimbal_data", data_frequency_.gimbal_data)) {
    RCLCPP_ERROR(get_logger(), "gimbal_data param not defined");
    exit(-1);
  }
  if (!get_parameter("flight_status", data_frequency_.flight_status)) {
    RCLCPP_ERROR(get_logger(), "flight_status param not defined");
    exit(-1);
  }
  if (!get_parameter("battery_level", data_frequency_.battery_level)) {
    RCLCPP_ERROR(get_logger(), "battery_level param not defined");
    exit(-1);
  }
  if (!get_parameter("control_information", data_frequency_.control_information)) {
    RCLCPP_ERROR(get_logger(), "control_information param not defined");
    exit(-1);
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
      strlen(params_.baudrate.c_str()) > sizeof(user_info->baudRate)) {
    RCLCPP_ERROR(get_logger(), "User information set is out of bounds");
    return false;
  }

  strncpy(user_info->appName, params_.app_name.c_str(), sizeof(user_info->appName) - 1);
  memcpy(user_info->appId, params_.app_id.c_str(), strlen(params_.app_id.c_str()));
  memcpy(user_info->appKey, params_.app_key.c_str(), strlen(params_.app_key.c_str()));
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
  if (DjiCore_Init(user_info) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "DJI core could not be initiated.");
    return false;
  }

  T_DjiAircraftInfoBaseInfo aircraft_base_info;
  if (DjiAircraftInfo_GetBaseInfo(&aircraft_base_info) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not get aircraft information.");
    return false;
  }

  if (aircraft_base_info.mountPosition != DJI_MOUNT_POSITION_EXTENSION_PORT) {
    RCLCPP_ERROR(get_logger(), "Please use the extension port");
    return false;
  }

  if (DjiCore_SetAlias("PSDK_UMD") != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not set alias.");
    return false;
  }

  if (DjiCore_ApplicationStart() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not start application.");
    return false;
  }
  return true;
}

}  // namespace umd_psdk