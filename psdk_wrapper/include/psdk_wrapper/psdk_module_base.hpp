#include <mutex>
#include <rclcpp/rclcpp.hpp>

class PSDKModuleBase
{
 public:
  PSDKModuleBase(const std::string& name, rclcpp::LifecycleNode::SharedPtr node)
      : name_(name), node_(node)
  {
  }

  virtual ~PSDKModuleBase() {}

  virtual void run() = 0;
  virtual bool init() = 0;
  virtual bool deinit() = 0;

 protected:
  std::string name_;
  rclcpp::LifecycleNode::SharedPtr node_;
  std::mutex module_mutex_;
};
