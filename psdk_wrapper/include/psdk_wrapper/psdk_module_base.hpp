#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace psdk_ros2
{
class PSDKModuleBase
{
 public:
  PSDKModuleBase(const std::string& name,
                 rclcpp_lifecycle::LifecycleNode::SharedPtr node)
      : name_(name), node_(node)
  {
  }

  virtual ~PSDKModuleBase() {}

  virtual bool init() = 0;
  virtual bool deinit() = 0;

 protected:
  std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::mutex module_mutex_;
};
}  // namespace psdk_ros2