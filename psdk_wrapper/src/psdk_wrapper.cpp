/*
 * Copyright (C) 2023 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file psdk_wrapper.cpp
 *
 * @brief Implementation of the PSDKWrapper class. This class is the main
 * interface between the Payload SDK and ROS 2. It initializes the modules and
 * the environment.
 *
 * @author Bianca Bendris
 * Contact: bianca@unmanned.life
 *
 */

#include "psdk_wrapper/psdk_wrapper.hpp"

std::shared_ptr<psdk_ros2::TelemetryModule> psdk_ros2::global_telemetry_ptr_;
std::shared_ptr<psdk_ros2::CameraModule> psdk_ros2::global_camera_ptr_;
std::shared_ptr<psdk_ros2::LiveviewModule> psdk_ros2::global_liveview_ptr_;
std::shared_ptr<psdk_ros2::HmsModule> psdk_ros2::global_hms_ptr_;
std::shared_ptr<psdk_ros2::PerceptionModule> psdk_ros2::global_perception_ptr_;

using namespace std::placeholders;  // NOLINT

namespace psdk_ros2
{
PSDKWrapper::PSDKWrapper(const std::string &node_name)
    : rclcpp_lifecycle::LifecycleNode(
          node_name, "",
          rclcpp::NodeOptions().use_intra_process_comms(true).arguments(
              {"--ros-args", "-r",
               node_name + ":" + std::string("__node:=") + node_name}))
{
  RCLCPP_INFO(get_logger(), "Creating Constructor PSDKWrapper");
  declare_parameter("app_name", rclcpp::ParameterValue(""));
  declare_parameter("app_id", rclcpp::ParameterValue(""));
  declare_parameter("app_key", rclcpp::ParameterValue(""));
  declare_parameter("app_license", rclcpp::ParameterValue(""));
  declare_parameter("developer_account", rclcpp::ParameterValue(""));
  declare_parameter("baudrate", rclcpp::ParameterValue(""));
  declare_parameter("link_config_file_path", rclcpp::ParameterValue(""));
  declare_parameter("mandatory_modules.telemetry",
                    rclcpp::ParameterValue(true));
  declare_parameter("mandatory_modules.flight_control",
                    rclcpp::ParameterValue(true));
  declare_parameter("mandatory_modules.camera", rclcpp::ParameterValue(true));
  declare_parameter("mandatory_modules.gimbal", rclcpp::ParameterValue(true));
  declare_parameter("mandatory_modules.liveview", rclcpp::ParameterValue(true));
  declare_parameter("mandatory_modules.hms", rclcpp::ParameterValue(true));
  declare_parameter("mandatory_modules.perception",
                    rclcpp::ParameterValue(true));
  declare_parameter("tf_frame_prefix", rclcpp::ParameterValue(""));
  declare_parameter("imu_frame", rclcpp::ParameterValue("psdk_imu_link"));
  declare_parameter("body_frame", rclcpp::ParameterValue("psdk_base_link"));
  declare_parameter("map_frame", rclcpp::ParameterValue("psdk_map_enu"));
  declare_parameter("gimbal_base_frame",
                    rclcpp::ParameterValue("psdk_gimbal_base_link"));
  declare_parameter("gimbal_frame", rclcpp::ParameterValue("psdk_gimbal_link"));
  declare_parameter("camera_frame", rclcpp::ParameterValue("psdk_camera_link"));
  declare_parameter("perception_camera_frame",
                    rclcpp::ParameterValue("psdk_perception_camera_link"));
  declare_parameter("publish_transforms", rclcpp::ParameterValue(true));
  declare_parameter("hms_return_codes_path", rclcpp::ParameterValue(""));
  declare_parameter("file_path", rclcpp::ParameterValue("/logs/media/"));

  declare_parameter("data_frequency.imu", 1);
  declare_parameter("data_frequency.timestamp", 1);
  declare_parameter("data_frequency.attitude", 1);
  declare_parameter("data_frequency.acceleration", 1);
  declare_parameter("data_frequency.velocity", 1);
  declare_parameter("data_frequency.angular_velocity", 1);
  declare_parameter("data_frequency.position", 1);
  declare_parameter("data_frequency.altitude", 1);
  declare_parameter("data_frequency.gps_fused_position", 1);
  declare_parameter("data_frequency.gps_data", 1);
  declare_parameter("data_frequency.rtk_data", 1);
  declare_parameter("data_frequency.magnetometer", 1);
  declare_parameter("data_frequency.rc_channels_data", 1);
  declare_parameter("data_frequency.gimbal_data", 1);
  declare_parameter("data_frequency.flight_status", 1);
  declare_parameter("data_frequency.battery_level", 1);
  declare_parameter("data_frequency.control_information", 1);
  declare_parameter("data_frequency.esc_data_frequency", 1);
  declare_parameter("num_of_initialization_retries", 1);

  get_parameter("mandatory_modules.telemetry", is_telemetry_module_mandatory_);
  get_parameter("mandatory_modules.flight_control",
                is_flight_control_module_mandatory_);
  get_parameter("mandatory_modules.camera", is_camera_module_mandatory_);
  get_parameter("mandatory_modules.gimbal", is_gimbal_module_mandatory_);
  get_parameter("mandatory_modules.liveview", is_liveview_module_mandatory_);
  get_parameter("mandatory_modules.hms", is_hms_module_mandatory_);
  get_parameter("mandatory_modules.perception",
                is_perception_module_mandatory_);

  create_module(is_telemetry_module_mandatory_, telemetry_module_,
                telemetry_thread_, "telemetry_node",
                psdk_ros2::global_telemetry_ptr_);
  create_module(is_flight_control_module_mandatory_, flight_control_module_,
                flight_control_thread_, "flight_control_node");
  create_module(is_camera_module_mandatory_, camera_module_, camera_thread_,
                "camera_node", psdk_ros2::global_camera_ptr_);
  create_module(is_gimbal_module_mandatory_, gimbal_module_, gimbal_thread_,
                "gimbal_node");
  create_module(is_liveview_module_mandatory_, liveview_module_,
                liveview_thread_, "liveview_node",
                psdk_ros2::global_liveview_ptr_);
  create_module(is_hms_module_mandatory_, hms_module_, hms_thread_, "hms_node",
                psdk_ros2::global_hms_ptr_);
  create_module(is_perception_module_mandatory_, perception_module_,
                perception_thread_, "perception_node",
                psdk_ros2::global_perception_ptr_);
}

PSDKWrapper::~PSDKWrapper()
{
  RCLCPP_INFO(get_logger(), "Destroying PSDKWrapper");
  rclcpp_lifecycle::State state;
  PSDKWrapper::on_shutdown(state);
}

PSDKWrapper::CallbackReturn
PSDKWrapper::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Configuring PSDKWrapper");

  load_parameters();
  if (!set_environment())
  {
    rclcpp::shutdown();
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

PSDKWrapper::CallbackReturn
PSDKWrapper::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating PSDKWrapper");

  T_DjiUserInfo user_info;
  set_user_info(&user_info);

  if (!init(&user_info) || !initialize_psdk_modules())
  {
    RCLCPP_INFO(get_logger(), "IFAILED HERE1");
    rclcpp::shutdown();
    return CallbackReturn::FAILURE;
  }

  telemetry_module_->set_aircraft_base(aircraft_base_info_);

  if (camera_module_)
  {
    telemetry_module_->set_camera_type(
        camera_module_->get_attached_camera_type());
  }

  // Delay the initialization of some modules due to dependencies
  if (!flight_control_module_->init(telemetry_module_->get_current_gps()) &&
      is_flight_control_module_mandatory_)
  {
    RCLCPP_INFO(get_logger(), "IFAILED HERE2");
    rclcpp::shutdown();
    return CallbackReturn::FAILURE;
  }

  if (!transition_modules_to_state(LifecycleState::CONFIGURE) ||
      !transition_modules_to_state(LifecycleState::ACTIVATE))
  {
    RCLCPP_INFO(get_logger(), "IFAILED HERE3");
    rclcpp::shutdown();
    return CallbackReturn::FAILURE;
  }
  telemetry_module_->subscribe_psdk_topics();

  return CallbackReturn::SUCCESS;
}

PSDKWrapper::CallbackReturn
PSDKWrapper::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Deactivating PSDKWrapper");
  telemetry_module_->unsubscribe_psdk_topics();

  // Deactivate the modules
  if (!transition_modules_to_state(LifecycleState::DEACTIVATE))
  {
    rclcpp::shutdown();
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

PSDKWrapper::CallbackReturn
PSDKWrapper::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaning up PSDKWrapper");
  if (!transition_modules_to_state(LifecycleState::CLEANUP))
  {
    rclcpp::shutdown();
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

PSDKWrapper::CallbackReturn
PSDKWrapper::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutting down PSDKWrapper");

  // Deinitialize all modules if they are initialized
  if ((is_telemetry_module_mandatory_ && telemetry_module_ &&
       !telemetry_module_->deinit()) ||
      (is_flight_control_module_mandatory_ && flight_control_module_ &&
       !flight_control_module_->deinit()) ||
      (is_camera_module_mandatory_ && camera_module_ &&
       !camera_module_->deinit()) ||
      (is_gimbal_module_mandatory_ && gimbal_module_ &&
       !gimbal_module_->deinit()) ||
      (is_liveview_module_mandatory_ && liveview_module_ &&
       !liveview_module_->deinit()) ||
      (is_hms_module_mandatory_ && hms_module_ && !hms_module_->deinit()) ||
      (is_perception_module_mandatory_ && perception_module_ &&
       !perception_module_->deinit()))
  {
    RCLCPP_ERROR(get_logger(), "Failed to deinitialize one or more modules.");
    return CallbackReturn::FAILURE;
  }

  // Deinitialize the core
  auto deinit_result = DjiCore_DeInit();
  if (deinit_result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "DJI core could not be deinitialized. Error code is: %ld",
                 deinit_result);
    return CallbackReturn::FAILURE;
  }

  if (!transition_modules_to_state(LifecycleState::SHUTDOWN))
  {
    return CallbackReturn::FAILURE;
  }

  stop_and_destroy_module(is_telemetry_module_mandatory_, telemetry_module_,
                          telemetry_thread_);
  stop_and_destroy_module(is_flight_control_module_mandatory_,
                          flight_control_module_, flight_control_thread_);
  stop_and_destroy_module(is_camera_module_mandatory_, camera_module_,
                          camera_thread_);
  stop_and_destroy_module(is_gimbal_module_mandatory_, gimbal_module_,
                          gimbal_thread_);
  stop_and_destroy_module(is_liveview_module_mandatory_, liveview_module_,
                          liveview_thread_);
  stop_and_destroy_module(is_hms_module_mandatory_, hms_module_, hms_thread_);
  stop_and_destroy_module(is_perception_module_mandatory_, perception_module_,
                          perception_thread_);

  rclcpp::shutdown();
  return CallbackReturn::SUCCESS;
}

bool
PSDKWrapper::set_environment()
{
  RCLCPP_INFO(get_logger(), "Setting environment");
  T_DjiReturnCode return_code;
  T_DjiOsalHandler osal_handler = {0};
  T_DjiHalUartHandler uart_handler = {0};
  T_DjiFileSystemHandler file_system_handler = {0};
  T_DjiSocketHandler socket_handler{0};
  T_DjiUserLinkConfig linkConfig;

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
  osal_handler.GetRandomNum = Osal_GetRandomNum;

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

  return_code = DjiUserConfigManager_LoadConfiguration(
      params_.link_config_file_path.c_str());
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Configuration file could not be loaded. Error code is: %ld",
                 return_code);
    return false;
  }
  RCLCPP_INFO(get_logger(), "Loaded configuration file");
  DjiUserConfigManager_GetLinkConfig(&linkConfig);
  if (linkConfig.type == DJI_USER_LINK_CONFIG_USE_UART_AND_USB_BULK_DEVICE)
  {
    RCLCPP_INFO(get_logger(), "Using DJI_USE_UART_USB_BULK_DEVICE");
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
  }
  else if (linkConfig.type == DJI_USER_LINK_CONFIG_USE_UART_AND_NETWORK_DEVICE)
  {
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
      return false;
    }
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Using DJI_USE_ONLY_UART");
  }

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

template <typename ModuleType>
void
PSDKWrapper::create_module(bool is_mandatory,
                           std::shared_ptr<ModuleType> &module_ptr,
                           std::unique_ptr<utils::NodeThread> &thread_ptr,
                           const std::string &node_name,
                           std::shared_ptr<ModuleType> &global_ptr)
{
  if (is_mandatory)
  {
    module_ptr = std::make_shared<ModuleType>(node_name);
    global_ptr = module_ptr;
    thread_ptr = std::make_unique<utils::NodeThread>(module_ptr);
  }
}

template <typename ModuleType>
void
PSDKWrapper::create_module(bool is_mandatory,
                           std::shared_ptr<ModuleType> &module_ptr,
                           std::unique_ptr<utils::NodeThread> &thread_ptr,
                           const std::string &node_name)
{
  if (is_mandatory)
  {
    module_ptr = std::make_shared<ModuleType>(node_name);
    thread_ptr = std::make_unique<utils::NodeThread>(module_ptr);
  }
}

template <typename ModuleType>
void
PSDKWrapper::stop_and_destroy_module(
    bool is_mandatory, std::shared_ptr<ModuleType> &module_ptr,
    std::unique_ptr<utils::NodeThread> &thread_ptr)
{
  if (is_mandatory && module_ptr)
  {
    // Stop the thread
    thread_ptr.reset();

    // Destroy the module
    module_ptr.reset();
  }
}

template <typename ModuleType>
bool
PSDKWrapper::initialize_module(bool is_mandatory,
                               std::shared_ptr<ModuleType> &module_ptr)
{
  if (is_mandatory && module_ptr)
  {
    auto init_func = std::bind(&ModuleType::init, module_ptr);
    return init_func();
  }
  return true;
}

void
PSDKWrapper::load_parameters()
{
  RCLCPP_INFO(get_logger(), "Loading parameters");
  get_mandatory_param("app_name", params_.app_name);
  RCLCPP_INFO(get_logger(), "App name: %s", params_.app_name.c_str());
  get_mandatory_param("app_id", params_.app_id);
  RCLCPP_INFO(get_logger(), "App id: %s", params_.app_id.c_str());
  get_mandatory_param("app_key", params_.app_key);
  RCLCPP_INFO(get_logger(), "App key: %s", params_.app_key.c_str());
  get_mandatory_param("app_license", params_.app_license);
  get_mandatory_param("developer_account", params_.developer_account);
  get_mandatory_param("baudrate", params_.baudrate);
  RCLCPP_INFO(get_logger(), "Baudrate: %s", params_.baudrate.c_str());

  get_non_mandatory_param("link_config_file_path",
                          params_.link_config_file_path);
  RCLCPP_INFO(get_logger(), "Using connection configuration file: %s",
              params_.link_config_file_path.c_str());

  get_parameter("num_of_initialization_retries",
                num_of_initialization_retries_);

  if (is_perception_module_mandatory_)
  {
    get_non_mandatory_param(
        "perception_camera_frame",
        perception_module_->params_.perception_camera_frame);
  }
  if (is_hms_module_mandatory_)
  {
    get_non_mandatory_param("hms_return_codes_path",
                            hms_module_->hms_return_codes_path_);
  }
  if (is_camera_module_mandatory_)
  {
    get_non_mandatory_param("file_path",
                            camera_module_->default_path_to_download_media_);
  }
  if (is_telemetry_module_mandatory_)
  {
    get_non_mandatory_param("tf_frame_prefix",
                            telemetry_module_->params_.tf_frame_prefix);
    get_non_mandatory_param("imu_frame", telemetry_module_->params_.imu_frame);
    get_non_mandatory_param("body_frame",
                            telemetry_module_->params_.body_frame);
    get_non_mandatory_param("map_frame", telemetry_module_->params_.map_frame);
    get_non_mandatory_param("gimbal_frame",
                            telemetry_module_->params_.gimbal_frame);
    get_non_mandatory_param("gimbal_base_frame",
                            telemetry_module_->params_.gimbal_base_frame);
    get_non_mandatory_param("camera_frame",
                            telemetry_module_->params_.camera_frame);
    get_parameter("publish_transforms",
                  telemetry_module_->params_.publish_transforms);
    // Get data frequency
    get_and_validate_frequency("data_frequency.imu",
                               telemetry_module_->params_.imu_frequency,
                               IMU_TOPIC_MAX_FREQ);
    get_and_validate_frequency("data_frequency.attitude",
                               telemetry_module_->params_.attitude_frequency,
                               ATTITUDE_TOPICS_MAX_FREQ);
    get_and_validate_frequency(
        "data_frequency.acceleration",
        telemetry_module_->params_.acceleration_frequency,
        ACCELERATION_TOPICS_MAX_FREQ);
    get_and_validate_frequency("data_frequency.velocity",
                               telemetry_module_->params_.velocity_frequency,
                               VELOCITY_TOPICS_MAX_FREQ);
    get_and_validate_frequency(
        "data_frequency.angular_velocity",
        telemetry_module_->params_.angular_rate_frequency,
        ANGULAR_VELOCITY_TOPICS_MAX_FREQ);
    get_and_validate_frequency("data_frequency.position",
                               telemetry_module_->params_.position_frequency,
                               POSITION_TOPICS_MAX_FREQ);
    get_and_validate_frequency("data_frequency.altitude",
                               telemetry_module_->params_.altitude_frequency,
                               ALTITUDE_TOPICS_MAX_FREQ);
    get_and_validate_frequency(
        "data_frequency.gps_fused_position",
        telemetry_module_->params_.gps_fused_position_frequency,
        GPS_FUSED_POSITION_TOPICS_MAX_FREQ);
    get_and_validate_frequency("data_frequency.gps_data",
                               telemetry_module_->params_.gps_data_frequency,
                               GPS_DATA_TOPICS_MAX_FREQ);
    get_and_validate_frequency("data_frequency.rtk_data",
                               telemetry_module_->params_.rtk_data_frequency,
                               RTK_DATA_TOPICS_MAX_FREQ);
    get_and_validate_frequency(
        "data_frequency.magnetometer",
        telemetry_module_->params_.magnetometer_frequency,
        MAGNETOMETER_TOPICS_MAX_FREQ);
    get_and_validate_frequency(
        "data_frequency.rc_channels_data",
        telemetry_module_->params_.rc_channels_data_frequency,
        RC_CHANNELS_TOPICS_MAX_FREQ);
    get_and_validate_frequency("data_frequency.esc_data_frequency",
                               telemetry_module_->params_.esc_data_frequency,
                               ESC_DATA_TOPICS_FREQ);
    get_and_validate_frequency("data_frequency.gimbal_data",
                               telemetry_module_->params_.gimbal_data_frequency,
                               GIMBAL_DATA_TOPICS_MAX_FREQ);
    get_and_validate_frequency(
        "data_frequency.flight_status",
        telemetry_module_->params_.flight_status_frequency,
        FLIGHT_STATUS_TOPICS_MAX_FREQ);
    get_and_validate_frequency(
        "data_frequency.battery_level",
        telemetry_module_->params_.battery_level_frequency,
        BATTERY_STATUS_TOPICS_MAX_FREQ);
    get_and_validate_frequency(
        "data_frequency.control_information",
        telemetry_module_->params_.control_information_frequency,
        CONTROL_DATA_TOPICS_MAX_FREQ);
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
  if (is_core_initialized_)
  {
    RCLCPP_INFO(get_logger(), "DJI Core already initialized, skipping.");
    return true;
  }
  RCLCPP_INFO(get_logger(), "Init DJI Core...");
  int current_num_retries = 0;
  T_DjiReturnCode result;
  while (current_num_retries <= num_of_initialization_retries_)
  {
    result = DjiCore_Init(user_info);
    if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "DJI core could not be initiated. Error code is: %ld. "
                   "Retrying for %d time. ",
                   result, current_num_retries);
      current_num_retries++;
      std::this_thread::sleep_for(std::chrono::milliseconds(5000));
      continue;
    }
    break;
  }

  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    return false;
  }

  if (DjiAircraftInfo_GetBaseInfo(&aircraft_base_info_) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not get aircraft information.");
    return false;
  }

  if (aircraft_base_info_.mountPosition != DJI_MOUNT_POSITION_EXTENSION_PORT)
  {
    RCLCPP_ERROR(get_logger(), "Please use the extension port");
    return false;
  }

  if (DjiCore_SetAlias("PSDK_App") != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not set alias.");
    return false;
  }

  if (DjiCore_ApplicationStart() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not start application.");
    return false;
  }
  is_core_initialized_ = true;
  return true;
}

bool
PSDKWrapper::initialize_psdk_modules()
{
  if (!initialize_module(is_telemetry_module_mandatory_, telemetry_module_) ||
      !initialize_module(is_camera_module_mandatory_, camera_module_) ||
      !initialize_module(is_gimbal_module_mandatory_, gimbal_module_) ||
      !initialize_module(is_liveview_module_mandatory_, liveview_module_) ||
      !initialize_module(is_hms_module_mandatory_, hms_module_) ||
      !initialize_module(is_perception_module_mandatory_, perception_module_))
  {
    return false;
  }

  return true;
}

void
PSDKWrapper::get_and_validate_frequency(const std::string &param_name,
                                        int &frequency, const int max_frequency)
{
  if (!get_parameter(param_name, frequency))
  {
    RCLCPP_ERROR(get_logger(), "%s param not defined", param_name.c_str());
    exit(-1);
  }
  if (frequency > max_frequency)
  {
    RCLCPP_WARN(get_logger(),
                "Frequency defined for %s is higher than the maximum allowed "
                "%d. The maximum value is set",
                param_name.c_str(), max_frequency);
    frequency = max_frequency;
  }
}

void
PSDKWrapper::get_non_mandatory_param(const std::string &param_name,
                                     std::string &param_string)
{
  if (!get_parameter(param_name, param_string))
  {
    RCLCPP_WARN(get_logger(), "%s param not defined, using default one: %s",
                param_name.c_str(), param_string.c_str());
  }
}

void
PSDKWrapper::get_mandatory_param(const std::string &param_name,
                                 std::string &param_string)
{
  if (!get_parameter(param_name, param_string))
  {
    RCLCPP_ERROR(get_logger(), "%s param not defined", param_name.c_str());
    exit(-1);
  }
}

bool
PSDKWrapper::transition_modules_to_state(LifecycleState state)
{
  std::function<CallbackReturn(
      std::shared_ptr<rclcpp_lifecycle::LifecycleNode>)>
      transition;

  // Define the transition function based on the desired lifecycle state
  switch (state)
  {
    case LifecycleState::ACTIVATE:
      transition = [](std::shared_ptr<rclcpp_lifecycle::LifecycleNode> module)
      { return module->on_activate(rclcpp_lifecycle::State()); };
      break;
    case LifecycleState::DEACTIVATE:
      transition = [](std::shared_ptr<rclcpp_lifecycle::LifecycleNode> module)
      { return module->on_deactivate(rclcpp_lifecycle::State()); };
      break;
    case LifecycleState::CONFIGURE:
      transition = [](std::shared_ptr<rclcpp_lifecycle::LifecycleNode> module)
      { return module->on_configure(rclcpp_lifecycle::State()); };
      break;
    case LifecycleState::CLEANUP:
      transition = [](std::shared_ptr<rclcpp_lifecycle::LifecycleNode> module)
      { return module->on_cleanup(rclcpp_lifecycle::State()); };
      break;
    case LifecycleState::SHUTDOWN:
      transition = [](std::shared_ptr<rclcpp_lifecycle::LifecycleNode> module)
      { return module->on_shutdown(rclcpp_lifecycle::State()); };
      break;
    default:
      return false;
  }

  // Check and transition only the mandatory modules
  bool all_transitions_successful = true;

  auto transition_if_mandatory =
      [&](bool is_mandatory,
          const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &module)
  {
    if (is_mandatory && module && transition(module) != CallbackReturn::SUCCESS)
    {
      all_transitions_successful = false;
    }
  };

  transition_if_mandatory(is_telemetry_module_mandatory_, telemetry_module_);
  transition_if_mandatory(is_flight_control_module_mandatory_,
                          flight_control_module_);
  transition_if_mandatory(is_camera_module_mandatory_, camera_module_);
  transition_if_mandatory(is_liveview_module_mandatory_, liveview_module_);
  transition_if_mandatory(is_gimbal_module_mandatory_, gimbal_module_);
  transition_if_mandatory(is_hms_module_mandatory_, hms_module_);
  transition_if_mandatory(is_perception_module_mandatory_, perception_module_);

  return all_transitions_successful;
}

}  // namespace psdk_ros2
