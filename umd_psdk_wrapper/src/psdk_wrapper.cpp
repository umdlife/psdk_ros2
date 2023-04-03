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
  if (!set_environment()) {
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

  if (!init(&user_info)) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  activate_ros_elements();

  if (!init_telemetry()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  subscribe_psdk_topics();

  if(!init_camera_manager()){
    return nav2_util::CallbackReturn::FAILURE;
  }
  if(!init_liveview_manager()){
    return nav2_util::CallbackReturn::FAILURE;
  }
  if(!init_gimbal_manager()){
    return nav2_util::CallbackReturn::FAILURE;
  }
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PSDKWrapper::on_deactivate(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "Deactivating PSDKWrapper");
  (void)state;
  deactivate_ros_elements();

  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PSDKWrapper::on_cleanup(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up PSDKWrapper");
  (void)state;
  clean_ros_elements();

  unsubscribe_psdk_topics();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PSDKWrapper::on_shutdown(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "Shutting down PSDKWrapper");

  if (DjiCore_DeInit() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  if (DjiFcSubscription_DeInit() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    return nav2_util::CallbackReturn::FAILURE;
  }

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
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Register OSAL handler error");
    return false;
  }
  RCLCPP_INFO(get_logger(), "Registered OSAL handler");

  return_code = DjiPlatform_RegHalUartHandler(&uart_handler);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Register HAL handler error");
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
  if (DjiPlatform_RegHalUsbBulkHandler(&usb_bulk_handler) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Register HAL USB BULK handler error");
    return false;
  }
#elif (HARDWARE_CONNECTION == DJI_USE_UART_AND_NETWORK_DEVICE)
  RCLCPP_INFO(get_logger(), "Using DJI_USE_UART_AND_NETWORK_DEVICE");
  T_DjiHalNetworkHandler network_handler;
  network_handler.NetworkInit = HalNetWork_Init;
  network_handler.NetworkDeInit = HalNetWork_DeInit;
  network_handler.NetworkGetDeviceInfo = HalNetWork_GetDeviceInfo;
  if (DjiPlatform_RegHalNetworkHandler(&network_handler) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Register HAL Network handler error");
  }
#elif (HARDWARE_CONNECTION == DJI_USE_ONLY_UART)
  RCLCPP_INFO(get_logger(), "Using DJI_USE_ONLY_UART");
  /*!< Attention: Only use uart hardware connection.
   */
#endif

  // Attention: if you want to use camera stream view function, please uncomment it.
  return_code = DjiPlatform_RegSocketHandler(&socket_handler);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Register OSAL SOCKET handler error");
    return false;
  }

  return_code = DjiPlatform_RegFileSystemHandler(&file_system_handler);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Register OSAL filesystem handler error");
    throw std::runtime_error("Register osal filesystem handler error.");
    return false;
  }
  RCLCPP_INFO(get_logger(), "Environment has been set!");
  return true;
}

void
PSDKWrapper::load_parameters()
{
  RCLCPP_INFO(get_logger(), "Loading parameters");
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
  RCLCPP_INFO(get_logger(), "Baudrate: %s", params_.baudrate.c_str());
  if (!get_parameter("hardware_connection", params_.hardware_connection)) {
    RCLCPP_ERROR(get_logger(), "hardware_connection param not defined");
    exit(-1);
  }
#define HARDWARE_CONNECTION params_.hardware_connection;
  RCLCPP_INFO(get_logger(), "Hardware connection: %s",
              params_.hardware_connection.c_str());

  if (!get_parameter("uart_dev_1", params_.uart_dev_1)) {
    RCLCPP_ERROR(get_logger(), "uart_dev_1 param not defined");
    exit(-1);
  }
  const char *name = "UART_DEV_1";
  setenv(name, params_.uart_dev_1.c_str(), 1);
  RCLCPP_INFO(get_logger(), "Uart dev 1: %s", params_.uart_dev_1.c_str());

  if (!get_parameter("uart_dev_2", params_.uart_dev_2)) {
    RCLCPP_ERROR(get_logger(), "uart_dev_2 param not defined");
    exit(-1);
  }
  name = "UART_DEV_2";
  setenv(name, params_.uart_dev_2.c_str(), 1);
  RCLCPP_INFO(get_logger(), "Uart dev 2: %s", params_.uart_dev_2.c_str());

  // Get data frequency
  int frequency = 0;
  if (!get_parameter("data_frequency.timestamp", frequency)) {
    RCLCPP_ERROR(get_logger(), "timestamp param not defined");
    exit(-1);
  }
  set_topic_frequency(&telemetry_.timestamp_topics, frequency);

  if (!get_parameter("data_frequency.attitude", frequency)) {
    RCLCPP_ERROR(get_logger(), "attitude param not defined");
    exit(-1);
  }
  set_topic_frequency(&telemetry_.attitude_topics, frequency);

  if (!get_parameter("data_frequency.acceleration", frequency)) {
    RCLCPP_ERROR(get_logger(), "acceleration param not defined");
    exit(-1);
  }
  set_topic_frequency(&telemetry_.acceleration_topics, frequency);

  if (!get_parameter("data_frequency.velocity", frequency)) {
    RCLCPP_ERROR(get_logger(), "velocity param not defined");
    exit(-1);
  }
  set_topic_frequency(&telemetry_.velocity_topics, frequency);

  if (!get_parameter("data_frequency.angular_velocity", frequency)) {
    RCLCPP_ERROR(get_logger(), "angular_velocity param not defined");
    exit(-1);
  }
  set_topic_frequency(&telemetry_.angular_velocity_topics, frequency);

  if (!get_parameter("data_frequency.position", frequency)) {
    RCLCPP_ERROR(get_logger(), "position param not defined");
    exit(-1);
  }
  set_topic_frequency(&telemetry_.position_topics, frequency);

  if (!get_parameter("data_frequency.gps_data", frequency)) {
    RCLCPP_ERROR(get_logger(), "gps_data param not defined");
    exit(-1);
  }
  set_topic_frequency(&telemetry_.gps_data_topics, frequency);

  if (!get_parameter("data_frequency.rtk_data", frequency)) {
    RCLCPP_ERROR(get_logger(), "rtk_data param not defined");
    exit(-1);
  }
  set_topic_frequency(&telemetry_.rtk_data_topics, frequency);

  if (!get_parameter("data_frequency.magnetometer", frequency)) {
    RCLCPP_ERROR(get_logger(), "magnetometer param not defined");
    exit(-1);
  }
  set_topic_frequency(&telemetry_.magnetometer_topics, frequency);

  if (!get_parameter("data_frequency.rc_channels_data", frequency)) {
    RCLCPP_ERROR(get_logger(), "rc_channels_data param not defined");
    exit(-1);
  }
  set_topic_frequency(&telemetry_.rc_channel_topics, frequency);

  if (!get_parameter("data_frequency.gimbal_data", frequency)) {
    RCLCPP_ERROR(get_logger(), "gimbal_data param not defined");
    exit(-1);
  }
  set_topic_frequency(&telemetry_.gimbal_topics, frequency);

  if (!get_parameter("data_frequency.flight_status", frequency)) {
    RCLCPP_ERROR(get_logger(), "flight_status param not defined");
    exit(-1);
  }
  set_topic_frequency(&telemetry_.flight_status_topics, frequency);

  if (!get_parameter("data_frequency.battery_level", frequency)) {
    RCLCPP_ERROR(get_logger(), "battery_level param not defined");
    exit(-1);
  }
  set_topic_frequency(&telemetry_.battery_status_topics, frequency);

  if (!get_parameter("data_frequency.control_information", frequency)) {
    RCLCPP_ERROR(get_logger(), "control_information param not defined");
    exit(-1);
  }
  set_topic_frequency(&telemetry_.control_topics, frequency);
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
  RCLCPP_INFO(get_logger(), "Init DJI Core...");
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



void PSDKWrapper::initialize_ros_elements()
{
//// Camera

// Actions
camera_start_shoot_single_photo_action_ = 
    std::make_unique<nav2_util::SimpleActionServer<CameraStartShootSinglePhoto>>(
          shared_from_this(), "camera_start_shoot_single_photo",
          std::bind(&PSDKWrapper::camera_start_shoot_single_photo_callback_, this));
camera_start_shoot_burst_photo_action_ = 
    std::make_unique<nav2_util::SimpleActionServer<CameraStartShootBurstPhoto>>(
          shared_from_this(), "camera_start_shoot_burst_photo",
          std::bind(&PSDKWrapper::camera_start_shoot_burst_photo_callback_, this));
camera_start_shoot_aeb_photo_action_ =
    std::make_unique<nav2_util::SimpleActionServer<CameraStartShootAEBPhoto>>(
          shared_from_this(), "camera_start_shoot_aeb_photo",
          std::bind(&PSDKWrapper::camera_start_shoot_aeb_photo_callback_, this));
camera_start_shoot_interval_photo_action_ =
    std::make_unique<nav2_util::SimpleActionServer<CameraStartShootIntervalPhoto>>(
          shared_from_this(), "camera_start_shoot_interval_photo",
          std::bind(&PSDKWrapper::camera_start_shoot_interval_photo_callback_, this));
camera_stop_shoot_photo_action_ =
    std::make_unique<nav2_util::SimpleActionServer<CameraStopShootPhoto>>(
          shared_from_this(), "camera_stop_shoot_photo",
          std::bind(&PSDKWrapper::camera_stop_shoot_photo_callback_, this));
camera_record_video_action_ =
    std::make_unique<nav2_util::SimpleActionServer<CameraRecordVideo>>(
          shared_from_this(), "camera_record_video",
          std::bind(&PSDKWrapper::camera_record_video_callback_, this));
camera_get_laser_ranging_info_action_ = 
    std::make_unique<nav2_util::SimpleActionServer<CameraGetLaserRangingInfo>>(
          shared_from_this(), "camera_get_laser_ranging_info",
          std::bind(&PSDKWrapper::camera_get_laser_ranging_info_callback_, this));
camera_download_file_list_action_ = 
    std::make_unique<nav2_util::SimpleActionServer<CameraDownloadFileList>>(
          shared_from_this(), "camera_download_file_list",
          std::bind(&PSDKWrapper::camera_download_file_list_callback_, this));
camera_download_file_by_index_action_ =
    std::make_unique<nav2_util::SimpleActionServer<CameraDownloadFileByIndex>>(
          shared_from_this(), "camera_download_file_by_index",
          std::bind(&PSDKWrapper::camera_download_file_by_index_callback_, this));
camera_delete_file_by_index_action_ =
    std::make_unique<nav2_util::SimpleActionServer<CameraDeleteFileByIndex>>(
          shared_from_this(), "camera_delete_file_by_index",
          std::bind(&PSDKWrapper::camera_delete_file_by_index_callback_, this));
camera_streaming_action_ = 
    std::make_unique<nav2_util::SimpleActionServer<CameraStreaming>>(
          shared_from_this(), "camera_streaming",
          std::bind(&PSDKWrapper::camera_streaming_callback_, this));
// Services
camera_get_type_service_ = create_service<CameraGetType>(
          "camera_get_type",
          std::bind(&PSDKWrapper::camera_get_type_callback_, this, _1, _2), qos_profile_);
camera_set_ev_service_ = create_service<CameraSetEV>(
          "camera_set_ev",
          std::bind(&PSDKWrapper::camera_set_ev_callback_, this, _1, _2), qos_profile_);
camera_get_ev_service_ = create_service<CameraGetEV>(
          "camera_get_ev",
          std::bind(&PSDKWrapper::camera_get_ev_callback_, this, _1, _2), qos_profile_);
camera_set_shutter_speed_service_ = create_service<CameraSetShutterSpeed>(
          "camera_set_shutter_speed",
          std::bind(&PSDKWrapper::camera_set_shutter_speed_callback_, this, _1, _2), qos_profile_);
camera_get_shutter_speed_service_ = create_service<CameraGetShutterSpeed>(
          "camera_get_shutter_speed",
          std::bind(&PSDKWrapper::camera_get_shutter_speed_callback_, this, _1, _2), qos_profile_);
camera_set_iso_service_ = create_service<CameraSetISO>(
          "camera_set_iso",
          std::bind(&PSDKWrapper::camera_set_iso_callback_, this, _1, _2), qos_profile_);
camera_get_iso_service_ = create_service<CameraGetISO>(
          "camera_get_iso",
          std::bind(&PSDKWrapper::camera_get_iso_callback_, this, _1, _2), qos_profile_);
camera_set_focus_target_service_ = create_service<CameraSetFocusTarget>(
          "camera_set_focus_target",
          std::bind(&PSDKWrapper::camera_set_focus_target_callback_, this, _1, _2), qos_profile_);
camera_get_focus_target_service_ = create_service<CameraGetFocusTarget>(
          "camera_get_focus_target",
          std::bind(&PSDKWrapper::camera_get_focus_target_callback_, this, _1, _2), qos_profile_);
camera_set_focus_mode_service_ = create_service<CameraSetFocusMode>(
          "camera_set_focus_mode",
          std::bind(&PSDKWrapper::camera_set_focus_mode_callback_, this, _1, _2), qos_profile_);
camera_get_focus_mode_service_ = create_service<CameraGetFocusMode>(
          "camera_get_focus_mode",
          std::bind(&PSDKWrapper::camera_get_focus_mode_callback_, this, _1, _2), qos_profile_);
camera_set_optical_zoom_service_ = create_service<CameraSetOpticalZoom>(
          "camera_set_optical_zoom",
          std::bind(&PSDKWrapper::camera_set_optical_zoom_callback_, this, _1, _2), qos_profile_);
camera_get_optical_zoom_service_ = create_service<CameraGetOpticalZoom>(
          "camera_get_optical_zoom",
          std::bind(&PSDKWrapper::camera_get_optical_zoom_callback_, this, _1, _2), qos_profile_);
camera_set_infrared_zoom_service_= create_service<CameraSetInfraredZoom>(
          "camera_set_infrared_zoom",
          std::bind(&PSDKWrapper::camera_set_infrared_zoom_callback_, this, _1, _2), qos_profile_);

//// Gimbal
// Services
gimbal_set_mode_service_ = create_service<GimbalSetMode>(
    "gimbal_set_mode",
    std::bind(&PSDKWrapper::gimbal_set_mode_callback_, this, _1, _2), qos_profile_);
gimbal_reset_service_ = create_service<GimbalReset>(
    "gimbal_reset",
    std::bind(&PSDKWrapper::gimbal_reset_callback_, this, _1, _2), qos_profile_);
// Actions
gimbal_rotation_action_ = 
    std::make_unique<nav2_util::SimpleActionServer<GimbalRotation>>(
          shared_from_this(), "gimbal_rotation",
          std::bind(&PSDKWrapper::gimbal_rotation_callback_, this));
//// Telemetry
// Publishers
  attitude_pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>(
      "dji_psdk_ros/attitude", 10);
  acceleration_ground_pub_ = create_publisher<geometry_msgs::msg::AccelStamped>(
      "dji_psdk_ros/acceleration_ground", 10);
  acceleration_body_pub_ = create_publisher<geometry_msgs::msg::AccelStamped>(
      "dji_psdk_ros/acceleration_body", 10);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("dji_psdk_ros/imu", 10);
  velocity_ground_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "dji_psdk_ros/velocity_ground", 10);
  flight_status_pub_ =
      create_publisher<std_msgs::msg::UInt8>("dji_psdk_ros/fligh_status", 10);
  altitude_pub_ =
      create_publisher<umd_psdk_interfaces::msg::Altitude>("dji_psdk_ros/altitude", 10);
  relative_height_pub_ =
      create_publisher<std_msgs::msg::Float32>("dji_psdk_ros/relative_height", 10);
  gps_position_pub_ =
      create_publisher<sensor_msgs::msg::NavSatFix>("dji_psdk_ros/gps_position", 10);
  rtk_position_pub_ =
      create_publisher<sensor_msgs::msg::NavSatFix>("dji_psdk_ros/rtk_position", 10);
  magnetometer_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(
      "dji_psdk_ros/magnetometer", 10);
  rc_pub_ = create_publisher<sensor_msgs::msg::Joy>("dji_psdk_ros/rc", 10);
  gimbal_angles_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "dji_psdk_ros/gimbal_angles", 10);
  gimbal_status_pub_ = create_publisher<umd_psdk_interfaces::msg::GimbalStatus>(
      "dji_psdk_ros/gimbal_status", 10);
  aircraft_status_pub_ = create_publisher<umd_psdk_interfaces::msg::AircraftStatus>(
      "dji_psdk_ros/aircraft_status", 10);
  battery_pub_ =
      create_publisher<umd_psdk_interfaces::msg::Battery>("dji_psdk_ros/battery", 10);
  flight_anomaly_pub_ = create_publisher<umd_psdk_interfaces::msg::FlightAnomaly>(
      "dji_psdk_ros/flight_anomaly", 10);
  position_fused_pub_ = create_publisher<umd_psdk_interfaces::msg::PositionFused>(
      "dji_psdk_ros/position_fused", 10);
  relative_obstacle_info_pub_ =
      create_publisher<umd_psdk_interfaces::msg::RelativeObstacleInfo>(
          "dji_psdk_ros/relative_obstacle_info", 10);
  home_position_pub_ = create_publisher<umd_psdk_interfaces::msg::HomePosition>(
      "dji_psdk_ros/home_position", 10);
}

void PSDKWrapper::activate_ros_elements()
{
  // Camera
  camera_start_shoot_single_photo_action_->activate();
  camera_start_shoot_burst_photo_action_->activate();
  camera_start_shoot_aeb_photo_action_->activate();
  camera_start_shoot_interval_photo_action_->activate();
  camera_stop_shoot_photo_action_->activate();
  camera_record_video_action_->activate();
  camera_get_laser_ranging_info_action_->activate();
  camera_download_file_list_action_->activate();
  camera_download_file_by_index_action_->activate();
  camera_delete_file_by_index_action_->activate();
  camera_streaming_action_->activate();
  // Gimbal
  gimbal_rotation_action_->activate();
  // Telemetry
  attitude_pub_->on_activate();
  acceleration_ground_pub_->on_activate();
  acceleration_body_pub_->on_activate();
  imu_pub_->on_activate();
  velocity_ground_pub_->on_activate();
  flight_status_pub_->on_activate();
  altitude_pub_->on_activate();
  relative_height_pub_->on_activate();
  gps_position_pub_->on_activate();
  rtk_position_pub_->on_activate();
  magnetometer_pub_->on_activate();
  rc_pub_->on_activate();
  gimbal_angles_pub_->on_activate();
  gimbal_status_pub_->on_activate();
  aircraft_status_pub_->on_activate();
  battery_pub_->on_activate();
  flight_anomaly_pub_->on_activate();
  position_fused_pub_->on_activate();
  relative_obstacle_info_pub_->on_activate();
  home_position_pub_->on_activate();
  
}

void PSDKWrapper::deactivate_ros_elements()
{
  // Camera
  camera_start_shoot_single_photo_action_->deactivate();
  camera_start_shoot_burst_photo_action_->deactivate();
  camera_start_shoot_aeb_photo_action_->deactivate();
  camera_start_shoot_interval_photo_action_->deactivate();
  camera_stop_shoot_photo_action_->deactivate();
  camera_record_video_action_->deactivate();
  camera_get_laser_ranging_info_action_->deactivate();
  camera_download_file_list_action_->deactivate();
  camera_download_file_by_index_action_->deactivate();
  camera_delete_file_by_index_action_->deactivate();
  camera_streaming_action_->deactivate();
  // Gimbal
  gimbal_rotation_action_->deactivate();
  // Telemetry
  attitude_pub_->on_deactivate();
  acceleration_ground_pub_->on_deactivate();
  acceleration_body_pub_->on_deactivate();
  imu_pub_->on_deactivate();
  velocity_ground_pub_->on_deactivate();
  flight_status_pub_->on_deactivate();
  altitude_pub_->on_deactivate();
  relative_height_pub_->on_deactivate();
  gps_position_pub_->on_deactivate();
  rtk_position_pub_->on_deactivate();
  magnetometer_pub_->on_deactivate();
  rc_pub_->on_deactivate();
  gimbal_angles_pub_->on_deactivate();
  gimbal_status_pub_->on_deactivate();
  aircraft_status_pub_->on_deactivate();
  battery_pub_->on_deactivate();
  flight_anomaly_pub_->on_deactivate();
  position_fused_pub_->on_deactivate();
  relative_obstacle_info_pub_->on_deactivate();
  home_position_pub_->on_deactivate();
}

void PSDKWrapper::clean_ros_elements()
{
  //Camera
  camera_start_shoot_single_photo_action_.reset();
  camera_start_shoot_burst_photo_action_.reset();
  camera_start_shoot_aeb_photo_action_.reset();
  camera_start_shoot_interval_photo_action_.reset();
  camera_stop_shoot_photo_action_.reset();
  camera_record_video_action_.reset();
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
  camera_get_laser_ranging_info_action_.reset();
  camera_download_file_list_action_.reset();
  camera_download_file_by_index_action_.reset();
  camera_delete_file_by_index_action_.reset();
  camera_streaming_action_.reset();
  // Gimbal
  gimbal_set_mode_service_.reset();
  gimbal_reset_service_.reset();
  gimbal_rotation_action_.reset();
  // Telemetry
  attitude_pub_.reset();
  acceleration_ground_pub_.reset();
  acceleration_body_pub_.reset();
  imu_pub_.reset();
  velocity_ground_pub_.reset();
  flight_status_pub_.reset();
  altitude_pub_.reset();
  relative_height_pub_.reset();
  gps_position_pub_.reset();
  rtk_position_pub_.reset();
  magnetometer_pub_.reset();
  rc_pub_.reset();
  gimbal_angles_pub_.reset();
  gimbal_status_pub_.reset();
  aircraft_status_pub_.reset();
  battery_pub_.reset();
  flight_anomaly_pub_.reset();
  position_fused_pub_.reset();
  relative_obstacle_info_pub_.reset();
  home_position_pub_.reset();
}

    
}  // namespace umd_psdk
