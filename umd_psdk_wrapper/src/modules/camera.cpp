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
camera_start_shoot_single_photo_action_ = 
    std::make_unique<nav2_util::SimpleActionServer<CameraStartShootSinglePhoto>>(
          shared_from_this(), "camera_start_shoot_single_photo",
          std::bind(&PSDKWrapper::camera_start_shoot_single_photo_callback_, this));

camera_start_shoot_burst_photo_action_ = 
    std::make_unique<nav2_util::SimpleActionServer<CameraStartShootBurstPhoto>>(
          shared_from_this(), "camera_start_shoot_burst_photo",
          std::bind(&PSDKWrapper::camera_start_shoot_burst_photo_callback_, this));
std::cout << "I'm initializing the actions" << std::endl;
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
PSDKWrapper::activate_ros_actions()
{
    RCLCPP_INFO(get_logger(), "Cleaning ROS services related to camera");
    // TODO(@lidiadltv): Add this to on_activate and deactivate
    camera_start_shoot_single_photo_action_->activate();
    camera_start_shoot_burst_photo_action_->activate();
}

void
PSDKWrapper::deactivate_ros_actions()
{
    RCLCPP_INFO(get_logger(), "Cleaning ROS services related to camera");
    // TODO(@lidiadltv): Add this to on_activate and deactivate
    camera_start_shoot_single_photo_action_->deactivate();
    camera_start_shoot_burst_photo_action_->deactivate();
}


void
PSDKWrapper::clean_ros_actions()
{
    RCLCPP_INFO(get_logger(), "Cleaning ROS services related to camera");
    // TODO(@lidiadltv): Add this to on_activate and deactivate
    camera_start_shoot_single_photo_action_.reset();
    camera_start_shoot_burst_photo_action_.reset();
}

void PSDKWrapper::camera_start_shoot_single_photo_callback_()
{
    RCLCPP_INFO(get_logger(), "Calling Camera shoot single photo");
    auto current_goal = camera_start_shoot_single_photo_action_->get_current_goal();
    auto action_result = std::make_shared<CameraStartShootSinglePhoto::Result>();
    // TODO(@lidiadltv): Should I put all the code below in another method as Onboard-SDK does?
    action_result->result = false;
    T_DjiReturnCode returnCode;
    E_DjiMountPosition index = static_cast<E_DjiMountPosition>(current_goal->payload_index);
    /*!< set camera work mode as shoot photo */
    returnCode = DjiCameraManager_SetMode(index, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        RCLCPP_INFO(get_logger(),"set mounted position %d camera's work mode as shoot-photo mode failed,"
                       " error code :0x%08X", current_goal->payload_index, returnCode);
        camera_start_shoot_single_photo_action_->terminate_current(action_result);
    }
    /*!< set shoot-photo mode */
    returnCode = DjiCameraManager_SetShootPhotoMode(index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        RCLCPP_INFO(get_logger(),"set mounted position %d camera's shoot photo mode as single-photo mode failed,"
                       " error code :0x%08X", current_goal->payload_index, returnCode);
        camera_start_shoot_single_photo_action_->terminate_current(action_result);
    }
    // TODO(@lidiadltv): Do I have to add a sleep like in the Payload-SDK examples??
    /*!< start to shoot single photo */
    returnCode = DjiCameraManager_StartShootPhoto(index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        camera_start_shoot_single_photo_action_->terminate_current(action_result);
    }
    else{
        action_result->result = true;
        camera_start_shoot_single_photo_action_->succeeded_current(action_result);
    }
    
}

void PSDKWrapper::camera_start_shoot_burst_photo_callback_()
{
    RCLCPP_INFO(get_logger(), "Calling Camera shoot burst photo");
    auto current_goal = camera_start_shoot_burst_photo_action_->get_current_goal();
    auto action_result = std::make_shared<CameraStartShootBurstPhoto::Result>();
    action_result->result = false;
    T_DjiReturnCode returnCode;

    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    E_DjiMountPosition index = static_cast<E_DjiMountPosition>(current_goal->payload_index);
    E_DjiCameraBurstCount burst_count = static_cast<E_DjiCameraBurstCount>(current_goal->photo_burst_count);

    /*!< set camera work mode as shoot photo */
    returnCode = DjiCameraManager_SetMode(index, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_INFO(get_logger(), "set mounted position %d camera's work mode as shoot photo mode failed,"
                       " error code :0x%08X.", index, returnCode);
        camera_start_shoot_burst_photo_action_->terminate_current(action_result);
        return;
    }
    /*!< set shoot-photo mode */
    returnCode = DjiCameraManager_SetShootPhotoMode(index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_BURST);
    if (returnCode == DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        RCLCPP_INFO(get_logger(), "Not supported command for camera mounted in position %d ", index);
        camera_start_shoot_burst_photo_action_->terminate_current(action_result);
        return;
    }
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_INFO(get_logger(),"set mounted position %d camera's shoot photo mode as burst-photo mode failed,"
                       " error code :0x%08X", index, returnCode);
        camera_start_shoot_burst_photo_action_->terminate_current(action_result);
        return;
    }
    /*! wait the APP change the shoot-photo mode display */
    osalHandler->TaskSleepMs(500);
    /*!< set shoot-photo mode parameter */
    returnCode = DjiCameraManager_SetPhotoBurstCount(index, burst_count);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        RCLCPP_INFO(get_logger(),"set mounted position %d camera's burst count(%d) failed,"
                       " error code :0x%08X.", index, burst_count, returnCode);
        camera_start_shoot_burst_photo_action_->terminate_current(action_result);
        return;
    }
    /*!< start to shoot single photo */
    returnCode = DjiCameraManager_StartShootPhoto(index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_BURST);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_INFO(get_logger(), "Mounted position %d camera shoot photo failed, "
                       "error code :0x%08X.", index, returnCode);
        camera_start_shoot_burst_photo_action_->terminate_current(action_result);
        return;
    }
    else{
        action_result->result = true;
        camera_start_shoot_burst_photo_action_->succeeded_current(action_result);
    }

}

}