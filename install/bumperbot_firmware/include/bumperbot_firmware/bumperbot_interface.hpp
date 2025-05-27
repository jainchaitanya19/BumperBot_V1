#ifndef BUMPERBOT_HARDWARE_HPP
#define BUMPERBOT_HARDWARE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <vector>
#include <string>

using namespace std;

namespace bumperbot_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class BumperbotHardware: public hardware_interface::SystemInterface
{
public:
BumperbotHardware();
  virtual ~BumperbotHardware();

  // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  // Implementing hardware_interface::SystemInterface
  virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  virtual vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  virtual hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  LibSerial::SerialPort arduino_;
  std::string port_;
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  rclcpp::Time last_run_;
};
}  // namespace bumperbot_firmware


#endif  // BUMPERBOT_INTERFACE_HPP
