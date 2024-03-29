//
// Created by phil on 22.03.20.
//

#include "DifferentialDriveSerialInterface.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <chrono>
namespace bot_differential_drive {

hardware_interface::CallbackReturn DifferentialDriveSerialInterface::on_init(
    const hardware_interface::HardwareInfo &info) {

  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  _wheelNames.resize(2);
  _jointEffort.resize(2);
  _jointPosition.resize(2);
  _jointVelocity.resize(2);
  _jointVelocityCommand.resize(2);
  _jointVelocityCommandExecuted.resize(2);
  _wheelNames[LEFT] = info.hardware_parameters.at("left_wheel_name");
  _wheelNames[RIGHT] = info.hardware_parameters.at("right_wheel_name");
  _loopHz = std::stof(info.hardware_parameters.at("loop_rate"));
  _deviceName = info.hardware_parameters.at("device_name");
  _timeout = std::stoi(info.hardware_parameters.at("timeout_ms"));
  _port = std::make_shared<SerialPort>(_deviceName,
                                       mn::CppLinuxSerial::BaudRate::B_9600);
  _port->SetTimeout(_timeout);
  _port->Open();

  for (const hardware_interface::ComponentInfo &joint : info.joints) {
    // DiffBotSystem has exactly two states and one command interface on each
    // joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(rclcpp::get_logger("DiffDriveInterface"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_ERROR(
          rclcpp::get_logger("DiffDriveInterface"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_ERROR(rclcpp::get_logger("DiffDriveInterface"),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_ERROR(
          rclcpp::get_logger("DiffDriveInterface"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_ERROR(
          rclcpp::get_logger("DiffDriveInterface"),
          "Joint '%s' have '%s' as second state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface>
DifferentialDriveSerialInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      _wheelNames[LEFT], hardware_interface::HW_IF_POSITION,
      &_jointPosition[LEFT]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      _wheelNames[LEFT], hardware_interface::HW_IF_VELOCITY,
      &_jointVelocity[LEFT]));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      _wheelNames[RIGHT], hardware_interface::HW_IF_POSITION,
      &_jointPosition[RIGHT]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      _wheelNames[RIGHT], hardware_interface::HW_IF_VELOCITY,
      &_jointVelocity[RIGHT]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DifferentialDriveSerialInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      _wheelNames[LEFT], hardware_interface::HW_IF_VELOCITY,
      &_jointVelocityCommand[LEFT]));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      _wheelNames[RIGHT], hardware_interface::HW_IF_VELOCITY,
      &_jointVelocityCommand[RIGHT]));

  return command_interfaces;
}

hardware_interface::CallbackReturn
DifferentialDriveSerialInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"),
              "Configuring ...please wait...");
  _port->Close();
  _port->SetDevice(_deviceName);
  _port->SetBaudRate(mn::CppLinuxSerial::BaudRate::B_9600);
  _port->SetTimeout(_timeout);
  _port->Open();
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"),
              "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DifferentialDriveSerialInterface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"),
              "Cleaning up ...please wait...");
  _port->Close();

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"),
              "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DifferentialDriveSerialInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"),
              "Activating ...please wait...");
  if (!_port->isOpen()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // TODO: configuration message
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"),
              "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DifferentialDriveSerialInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"),
              "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"),
              "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
DifferentialDriveSerialInterface::read(const rclcpp::Time & /*time*/,
                                       const rclcpp::Duration &period) {
  if (!_port->isOpen()) {
    return hardware_interface::return_type::ERROR;
  }
  _port->Write(serial_protocol::MsgQueryState().strSerial());
  auto raw = _port->ReadUntil('\n', 100);

  if (!raw.empty() &&
      serial_protocol::parseType(raw) == serial_protocol::MsgType::STATE) {
    try {
      serial_protocol::MsgStateVelocityAndPosition msg{raw};
      _jointVelocity[LEFT] = msg._vl;
      _jointVelocity[RIGHT] = msg._vr;
      _jointPosition[LEFT] = msg._pl;
      _jointPosition[RIGHT] = msg._pr;
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"),
                  "Received STATE: \n%s ", msg.str().c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("DiffDriveInterface"),
                   "Error during reading from serial port. Received: %s ",
                   raw.c_str());
    }

  } else {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveInterface"),
                 "Could not read state from serial port. Received: %s ",
                 raw.c_str());
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
DifferentialDriveSerialInterface::write(const rclcpp::Time & /*time*/,
                                        const rclcpp::Duration & /*period*/) {
  if (!_port->isOpen()) {
    return hardware_interface::return_type::ERROR;
  }
  if (_jointVelocityCommand[LEFT] != 0 || _jointVelocityCommand[RIGHT] != 0) {
    _port->Write(serial_protocol::MsgCmdVel(_jointVelocityCommand[LEFT],
                                            _jointVelocityCommand[RIGHT], 0)
                     .strSerial());
  }
  return hardware_interface::return_type::OK;
}
} // namespace bot_differential_drive
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bot_differential_drive::DifferentialDriveSerialInterface,
                       hardware_interface::SystemInterface)
