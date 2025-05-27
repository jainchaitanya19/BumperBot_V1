#include "bumperbot_firmware/bumperbot_interface.hpp"
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>  


namespace bumperbot_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using hardware_interface::HardwareInfo;
using hardware_interface::SystemInterface;
using hardware_interface::StateInterface;
using hardware_interface::CommandInterface;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using rclcpp_lifecycle::State;
using namespace rclcpp;

BumperbotHardware::BumperbotHardware()
{
}

BumperbotHardware::~BumperbotHardware()
{
    if (arduino_.IsOpen())
    {
        try
        {
            arduino_.Close();
        }
        catch (...)
        {
            RCLCPP_FATAL(rclcpp::get_logger("BumperbotHardware"),
                         "Something went wrong while closing the port: %s", port_.c_str());
        }
    }
}
/*This function job iThis function’s main job is to initialize the custom hardware interface using data provided in the URDF 
(robot description) and the hardware configuration parameters.s to initialize the hardware interface 
based on infomation provided in the URDF and configuration file */
CallbackReturn BumperbotHardware::on_init(const HardwareInfo &hardware_info)
{
    CallbackReturn result = SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    try
    {
        port_ = info_.hardware_parameters.at("port");
    }
    catch (...)
    {
        RCLCPP_FATAL(get_logger("BumperbotHardware"), "No serial port provided! Aborting.");
        return CallbackReturn::FAILURE;
    }

    velocity_commands_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());
    velocity_states_.reserve(info_.joints.size());

    return CallbackReturn::SUCCESS;
}
/* This function exports state interfaces for each joint to ROS 2 control framework.
It creates and returns a vector of StateInterface objects, each linking a joint name 
and state type (e.g., position or velocity) to a corresponding memory location.
This allows ROS 2 controllers to read real-time hardware states from the robot. */

vector<StateInterface> BumperbotHardware::export_state_interfaces()
{
  vector<StateInterface> state_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // Register a state interface for each joint with position data.
// This allows the controller to read the joint's position from hardware.
// emplace_back creates the StateInterface directly in the vector to avoid copying.
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}

/*This function exports command interfaces for each joint to the ROS 2 control framework.
It creates and returns a vector of CommandInterface objects, each linking a joint name 
and command type (e.g., velocity) to a memory location where controller commands will be written.
This allows ROS 2 controllers to send commands to the robot’s actuators. */
vector<CommandInterface> BumperbotHardware::export_command_interfaces()
{
  vector<CommandInterface> command_interfaces;

  // Provide only a velocity Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // Register a command interface for each joint with velocity command access.
// This lets the controller send velocity commands to each joint during control cycles.
// emplace_back constructs the CommandInterface directly inside the vector.
    command_interfaces.emplace_back(CommandInterface(
        info_.joints[i].name, HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}
// This function is called when the lifecycle transitions from "inactive" to "active".
// It performs the following:
// 1. Resets velocity and position state variables to initial values.
// 2. Opens a serial connection with the Arduino using the configured port and baud rate.
// 3. If the port is successfully opened, the hardware interface is activated and ready to send commands.
// 4. If there is any error opening the port, it returns FAILURE to prevent unsafe activation.

CallbackReturn BumperbotHardware::on_activate(const State &)
{
  RCLCPP_INFO(rclcpp::get_logger("BumperbotHardware"), "Starting robot hardware ...");

  // Reset commands and states
  
  velocity_commands_ = { 0.0, 0.0 };
  position_states_ = { 0.0, 0.0 };
  velocity_states_ = { 0.0, 0.0 };

  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotHardware"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("BumperbotHardware"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


// This function safely deactivates the robot hardware when the lifecycle node transitions from active to inactive.
// It closes the serial connection to the Arduino if it is open and logs the operation status.
// Useful for halting motion and cleaning up before shutdown or reconfiguration.
CallbackReturn BumperbotHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("BumperbotHardware"), "Stopping robot hardware ...");

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotHardware"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("BumperbotHardware"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}
// This function reads sensor data from the Arduino over serial connection.
// It parses velocity values for the left and right wheels and updates both velocity and position states.
// The positions are updated using numerical integration: pos += vel × dt.

hardware_interface::return_type BumperbotHardware::read(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  // Interpret the string
  if(arduino_.IsDataAvailable())
  {
    auto dt = (rclcpp::Clock().now() - last_run_).seconds();
    std::string message;
    arduino_.ReadLine(message);
    std::stringstream ss(message);
    std::string res;
    int multiplier = 1;
    while(std::getline(ss, res, ','))
    {
      multiplier = res.at(1) == 'p' ? 1 : -1;

      if(res.at(0) == 'r')
      {
        velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
        position_states_.at(0) += velocity_states_.at(0) * dt;
      }
      else if(res.at(0) == 'l')
      {
        velocity_states_.at(1) = multiplier * std::stod(res.substr(2, res.size()));
        position_states_.at(1) += velocity_states_.at(1) * dt;
      }
    }
    last_run_ = rclcpp::Clock().now();
  }
  return hardware_interface::return_type::OK;
}



hardware_interface::return_type BumperbotHardware::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  // Implement communication protocol with the Arduino
  std::stringstream message_stream;
  char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
  char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';
  std::string compensate_zeros_right = "";
  std::string compensate_zeros_left = "";
  if(std::abs(velocity_commands_.at(0)) < 10.0)
  {
    compensate_zeros_right = "0";
  }
  else
  {
    compensate_zeros_right = "";
  }
  if(std::abs(velocity_commands_.at(1)) < 10.0)
  {
    compensate_zeros_left = "0";
  }
  else
  {
    compensate_zeros_left = "";
  }
  
  message_stream << std::fixed << std::setprecision(2) << 
    "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_.at(0)) << 
    ",l" <<  left_wheel_sign << compensate_zeros_left << std::abs(velocity_commands_.at(1)) << ",";

  try
  {
    arduino_.Write(message_stream.str());
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("BumperbotInterface"),
                        "Something went wrong while sending the message "
                            << message_stream.str() << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}


} // namespace bumperbot_hardware

//PLUGINLIB_EXPORT_CLASS(Namespace_name, inside the namspace- name of the class )
PLUGINLIB_EXPORT_CLASS(bumperbot_hardware::BumperbotHardware, hardware_interface::SystemInterface)