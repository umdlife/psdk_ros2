/* Copyright (C) 2023 Unmanned Life - All Rights Reserved
 *
 * This file is part of the `umd_psdk_wrapper` source code package and is subject to
 * the terms and conditions defined in the file LICENSE.txt contained therein.
 */
/**
 * @file camera.cpp
 *
 * @brief
 *
 * @author Lidia de la Torre Vazquez
 * Contact: lidia@unmanned.life
 *
 */

#include "umd_psdk_wrapper/psdk_wrapper.hpp"
#include "umd_psdk_wrapper/psdk_wrapper_utils.hpp"

namespace umd_psdk {

using namespace std::placeholders; 

void PSDKWrapper::initialize_ros_camera_services()
{

// init_camera_manager_service_ = create_service<std_srvs::srv::Empty>(
//     "init_camera_manager",
//     std::bind(&PSDKWrapper::init_camera_manager_callback_, this, _1, _2), qos_profile_);
camera_start_shoot_single_photo_service_ = create_service<CameraStartShootSinglePhoto>(
    "camera_start_shoot_single_photo",
    std::bind(&PSDKWrapper::camera_start_shoot_single_photo_callback_, this, _1, _2), qos_profile_);

}

bool PSDKWrapper::init_camera_manager()
{
RCLCPP_INFO(get_logger(), "Initiating camera manager...");
  if (DjiCameraManager_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not initialize camera manager.");
    return false;
  }
  return true;

}


void
PSDKWrapper::clean_ros_services()
{
    RCLCPP_INFO(get_logger(), "Cleaning ROS services related to camera");
    init_camera_manager_service_.reset();
    camera_start_shoot_single_photo_service_.reset();
}

bool PSDKWrapper::init_camera_manager_callback_(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request, 
    const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    RCLCPP_INFO(get_logger(), "Initiating camera manager...");
    T_DjiReturnCode returnCode;
    returnCode = DjiCameraManager_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Could not initialize Camera Manager");
      return false;
      // TODO(@lidiadltv): Add the exitCameraModule action from the PSDK samples?
    }
    else if(returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
        RCLCPP_ERROR(get_logger(), "It worked!");
    }
    return true;
}

bool PSDKWrapper::camera_start_shoot_single_photo_callback_(
    const std::shared_ptr<CameraStartShootSinglePhoto::Request> request,
    const std::shared_ptr<CameraStartShootSinglePhoto::Response> response)
{
    RCLCPP_INFO(get_logger(), "Calling Camera shoot single photo");
    // TODO(@lidiadltv): Should I put all the code below in another method as Onboard-SDK does?
    response->result = false;
    T_DjiReturnCode returnCode;
    E_DjiMountPosition index = static_cast<E_DjiMountPosition>(request->payload_index);
    /*!< set camera work mode as shoot photo */
    returnCode = DjiCameraManager_SetMode(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        std::cout << "0" << std::endl;
        RCLCPP_INFO(get_logger(),"set mounted position %d camera's work mode as shoot-photo mode failed,"
                       " error code :0x%08X", request->payload_index, returnCode);
        return response->result;
    }
    std::cout << "1" << std::endl;
    /*!< set shoot-photo mode */
    returnCode = DjiCameraManager_SetShootPhotoMode(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
            std::cout << "2" << std::endl;
        RCLCPP_INFO(get_logger(),"set mounted position %d camera's shoot photo mode as single-photo mode failed,"
                       " error code :0x%08X", request->payload_index, returnCode);
        return response->result;
    }
    std::cout << "3" << std::endl;
    // TODO(@lidiadltv): Do I have to add a sleep like in the Payload-SDK examples??

    /*!< start to shoot single photo */
    returnCode = DjiCameraManager_StartShootPhoto(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        // USER_LOG_ERROR("Mounted position %d camera shoot photo failed, "
        //                "error code :0x%08X", request->payload_index, returnCode);
        std::cout << "4" << std::endl;
        return response->result;
    }
    std::cout << "5" << std::endl;
    response->result = true;
    return response->result;
}

}