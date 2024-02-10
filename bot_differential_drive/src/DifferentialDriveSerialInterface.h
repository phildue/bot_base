//
// Created by phil on 22.03.20.
//

#ifndef ACTUATOR_INTERFACE_VELOCITY_H
#define ACTUATOR_INTERFACE_VELOCITY_H
#include "SerialProtocol.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_control.h"
#include <thread>
namespace bot_differential_drive {
class DifferentialDriveSerialInterface
    : public hardware_interface::SystemInterface {
public:
  virtual ~DifferentialDriveSerialInterface() = default;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  DIFFDRIVE_ARDUINO_PUBLIC
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  DIFFDRIVE_ARDUINO_PUBLIC
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  static constexpr int LEFT = 0;
  static constexpr int RIGHT = 1;

protected:
  SerialProtocol _serial;
  std::string _deviceName;
  std::vector<double> _jointEffort, _jointPosition, _jointVelocity;
  std::vector<double> _jointVelocityCommand, _jointVelocityCommandExecuted;
  int _numJoints;
  std::vector<std::string> _wheelNames;
  double _loopHz;
  double p_error_, v_error_, e_error_;
  std::shared_ptr<SerialPort> _port;
};
} // namespace bot_differential_drive
#endif // ACTUATOR_INTERFACE_VELOCITY_H
