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
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PSDKWrapper::on_activate(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "Activating PSDKWrapper");
  umd_psdk::PSDKWrapper::on_activate(state);

  load_parameters();
  T_DjiUserInfo user_info;
  set_user_info(&user_info);

  if (!init(&user_info)) {
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

  return nav2_util::CallbackReturn::SUCCESS;
}

void
PSDKWrapper::set_environment()
{
  T_DjiReturnCode returnCode;
  T_DjiOsalHandler osalHandler;
  T_DjiHalUartHandler uartHandler;
  T_DjiHalUsbBulkHandler usbBulkHandler;
  T_DjiFileSystemHandler fileSystemHandler;
  T_DjiSocketHandler socketHandler;
  T_DjiHalNetworkHandler networkHandler;

  networkHandler.NetworkInit = HalNetWork_Init;
  networkHandler.NetworkDeInit = HalNetWork_DeInit;
  networkHandler.NetworkGetDeviceInfo = HalNetWork_GetDeviceInfo;

  socketHandler.Socket = Osal_Socket;
  socketHandler.Bind = Osal_Bind;
  socketHandler.Close = Osal_Close;
  socketHandler.UdpSendData = Osal_UdpSendData;
  socketHandler.UdpRecvData = Osal_UdpRecvData;
  socketHandler.TcpListen = Osal_TcpListen;
  socketHandler.TcpAccept = Osal_TcpAccept;
  socketHandler.TcpConnect = Osal_TcpConnect;
  socketHandler.TcpSendData = Osal_TcpSendData;
  socketHandler.TcpRecvData = Osal_TcpRecvData;

  osalHandler.TaskCreate = Osal_TaskCreate;
  osalHandler.TaskDestroy = Osal_TaskDestroy;
  osalHandler.TaskSleepMs = Osal_TaskSleepMs;
  osalHandler.MutexCreate = Osal_MutexCreate;
  osalHandler.MutexDestroy = Osal_MutexDestroy;
  osalHandler.MutexLock = Osal_MutexLock;
  osalHandler.MutexUnlock = Osal_MutexUnlock;
  osalHandler.SemaphoreCreate = Osal_SemaphoreCreate;
  osalHandler.SemaphoreDestroy = Osal_SemaphoreDestroy;
  osalHandler.SemaphoreWait = Osal_SemaphoreWait;
  osalHandler.SemaphoreTimedWait = Osal_SemaphoreTimedWait;
  osalHandler.SemaphorePost = Osal_SemaphorePost;
  osalHandler.Malloc = Osal_Malloc;
  osalHandler.Free = Osal_Free;
  osalHandler.GetTimeMs = Osal_GetTimeMs;
  osalHandler.GetTimeUs = Osal_GetTimeUs;

  uartHandler.UartInit = HalUart_Init;
  uartHandler.UartDeInit = HalUart_DeInit;
  uartHandler.UartWriteData = HalUart_WriteData;
  uartHandler.UartReadData = HalUart_ReadData;
  uartHandler.UartGetStatus = HalUart_GetStatus;

  usbBulkHandler.UsbBulkInit = HalUsbBulk_Init;
  usbBulkHandler.UsbBulkDeInit = HalUsbBulk_DeInit;
  usbBulkHandler.UsbBulkWriteData = HalUsbBulk_WriteData;
  usbBulkHandler.UsbBulkReadData = HalUsbBulk_ReadData;
  usbBulkHandler.UsbBulkGetDeviceInfo = HalUsbBulk_GetDeviceInfo;

  fileSystemHandler.FileOpen = Osal_FileOpen,
  fileSystemHandler.FileClose = Osal_FileClose,
  fileSystemHandler.FileWrite = Osal_FileWrite,
  fileSystemHandler.FileRead = Osal_FileRead,
  fileSystemHandler.FileSync = Osal_FileSync,
  fileSystemHandler.FileSeek = Osal_FileSeek, fileSystemHandler.DirOpen = Osal_DirOpen,
  fileSystemHandler.DirClose = Osal_DirClose, fileSystemHandler.DirRead = Osal_DirRead,
  fileSystemHandler.Mkdir = Osal_Mkdir, fileSystemHandler.Unlink = Osal_Unlink,
  fileSystemHandler.Rename = Osal_Rename, fileSystemHandler.Stat = Osal_Stat,

  returnCode = DjiPlatform_RegOsalHandler(&osalHandler);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Register OSAL handler error");
  }

  returnCode = DjiPlatform_RegHalUartHandler(&uartHandler);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Register HAL handler error");
  }

#if (HARDWARE_CONNECTION == DJI_USE_UART_AND_USB_BULK_DEVICE)
  returnCode = DjiPlatform_RegHalUsbBulkHandler(&usbBulkHandler);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Register HAL USB BULK handler error");
  }
#elif (HARDWARE_CONNECTION == DJI_USE_UART_AND_NETWORK_DEVICE)
  returnCode = DjiPlatform_RegHalNetworkHandler(&networkHandler);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Register HAL Network handler error");
  }
#elif (HARDWARE_CONNECTION == DJI_USE_ONLY_UART)
  /*!< Attention: Only use uart hardware connection.
   */
#endif

  // Attention: if you want to use camera stream view function, please uncomment it.
  returnCode = DjiPlatform_RegSocketHandler(&socketHandler);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Register OSAL SOCKET handler error");
  }

  returnCode = DjiPlatform_RegFileSystemHandler(&fileSystemHandler);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
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
  }
}

}  // namespace umd_psdk