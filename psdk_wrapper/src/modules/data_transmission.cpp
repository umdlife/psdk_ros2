#include <string>
#include "psdk_wrapper/psdk_wrapper.hpp"

E_DjiChannelAddress channelAddress = DJI_CHANNEL_ADDRESS_MASTER_RC_APP;

namespace psdk_ros2
{

T_DjiReturnCode c_ReceiveDataFromMobileCallback(const uint8_t* data, uint16_t len) {
  return global_ptr_->publish_data_from_mobile(data, len);
}
bool
PSDKWrapper::init_data_transmission()
{
  RCLCPP_INFO(get_logger(), "Initiating data_transmission module...");
  if (DjiLowSpeedDataChannel_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not initialize data_transmission module.");
    return false;
  }

  T_DjiReturnCode return_code = DjiLowSpeedDataChannel_RegRecvDataCallback(channelAddress,
                                                                           c_ReceiveDataFromMobileCallback);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not initialize receive data callback.");
    return false;
  }
  return true;
}

bool PSDKWrapper::deinit_data_transmission()
{
  RCLCPP_INFO(get_logger(), "Deinitializing data_transmission module...");
  if (DjiLowSpeedDataChannel_DeInit() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not deinitialize the data_transmission module.");
    return false;
  }
  return true;
}



T_DjiReturnCode PSDKWrapper::publish_data_from_mobile(const uint8_t* data,
                                           uint16_t len)
{
  auto trans_data = std::make_unique<psdk_interfaces::msg::TransmissionData>();
  // The +1 appears to be neccessary, as otherwise there is an additional byte before the
  // data starts, that appears to be the length of the data.
  std::string string_data { (char*) (data+1), len-1 };
  trans_data->data = string_data;
  trans_data->header.stamp = this->get_clock()->now();
  trans_data->header.frame_id = get_optical_frame_id();
  low_speed_transmission_data_pub_->publish(std::move(trans_data));
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

}
