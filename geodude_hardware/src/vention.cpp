// Copyright 2024 Personal Robotics Lab, University of Washington
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// Authors: Atharva Pradhan

#include "geodude_hardware/vention.hpp"

#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// ROS
#include <hardware_interface/types/hardware_interface_type_values.hpp>

using namespace std::chrono_literals;
namespace geodude_hardware
{

// Ensure Shutdown
VentionHW::~VentionHW()
{
  // Ignore SIGTERM (buys us 10s)
  signal(SIGTERM, SIG_IGN);
  // If the controller manager is shutdown via Ctrl + C the on_deactivate methods won't be called.
  // We therefore need to make sure to actually deactivate the communication
  on_deactivate(rclcpp_lifecycle::State());
  on_cleanup(rclcpp_lifecycle::State());
  on_shutdown(rclcpp_lifecycle::State());
}

// Init: Read info and configure command/state buffers
hardware_interface::CallbackReturn VentionHW::on_init(const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  vention_states_positions_.resize(VENTION_DOF, 0.0);   // Do not set default values to NaN
  vention_velocity_commands_.resize(VENTION_DOF, 0.0);  // Do not set default values to NaN
  vention_states_velocities_.resize(VENTION_DOF, std::numeric_limits<double>::quiet_NaN());
  vention_position_commands_.resize(VENTION_DOF, std::numeric_limits<double>::quiet_NaN());
  vention_velocity_commands_old_.resize(VENTION_DOF, std::numeric_limits<double>::quiet_NaN());
  vention_states_positions_old_.resize(VENTION_DOF, std::numeric_limits<double>::quiet_NaN());
  vention_states_velocities_old_.resize(VENTION_DOF, std::numeric_limits<double>::quiet_NaN());
  vention_states_accelerations_.resize(VENTION_DOF, std::numeric_limits<double>::quiet_NaN());

  vention_async_thread_shutdown_ = false;  // changes to true on deactivate
  vention_control_level_ = control_integration_level_::vUNDEFINED;

  // Checking for state interfaces
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // VentionHW has exactly 2 state interfaces
    // and 1 command interface on each joint
    if (joint.state_interfaces.size() != VENTION_DOF) {
      RCLCPP_FATAL(
        rclcpp::get_logger(info_.name), "Joint '%s' has %zu state interfaces. 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const hardware_interface::InterfaceInfo & interface_info : joint.state_interfaces) {
      if (!(interface_info.name == hardware_interface::HW_IF_POSITION ||
            interface_info.name == hardware_interface::HW_IF_VELOCITY)) {
        RCLCPP_FATAL(
          rclcpp::get_logger(info_.name), "Joint '%s' has %s state interface. Expected %s or %s",
          joint.name.c_str(), interface_info.name.c_str(), hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // Checking for command interfaces per joint
    if (!(joint.command_interfaces.size() == 1)) {
      RCLCPP_FATAL(
        rclcpp::get_logger(info_.name),
        "Joint '%s' has %zu command interfaces. Only Velocity expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const hardware_interface::InterfaceInfo & interface_info : joint.command_interfaces) {
      if (!(interface_info.name == hardware_interface::HW_IF_POSITION ||
            interface_info.name == hardware_interface::HW_IF_VELOCITY)) {
        RCLCPP_FATAL(
          rclcpp::get_logger(info_.name), "Joint '%s' has %s command interface. Expected %s or %s.",
          joint.name.c_str(), interface_info.name.c_str(), hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VentionHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (std::size_t i = 0; i < VENTION_DOF; i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &vention_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vention_states_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VentionHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < VENTION_DOF; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &vention_position_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vention_velocity_commands_[i]));
  }

  return command_interfaces;
}

// Mode Switching
hardware_interface::return_type VentionHW::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  /*
  Without this check, the BarrettHW component will call this medthod which will
  lead to the controller_manager node to error out because 
  VentionHW::prepare_command_mode_switch/this function is not implemented to handle 
  the BarrettHW's command mode switch request, neither does it have the
  necessary joints. Hence, we only call prepare_command_mode_switch for 
  the following cases.
  */
  if (
    // Vention controller activation call:
    (start_interfaces.size() == VENTION_DOF && stop_interfaces.empty()) ||
    // Vention controller deactivation call:
    (start_interfaces.empty() && stop_interfaces.size() == VENTION_DOF) ||
    // Vention controller switch call (Curretnly, there is only velocity control mode, implying this will never be the case):
    (start_interfaces.size() == VENTION_DOF && stop_interfaces.size() == VENTION_DOF)) {
    // Only one mode switch at a time
    const std::lock_guard<std::mutex> lock(mMutex);
    // Prepare stopping command modes
    std::vector<control_integration_level_> old_modes = {};
    for (const std::string & key : stop_interfaces) {
      for (std::size_t i = 0; i < VENTION_DOF; i++) {
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
          old_modes.push_back(control_integration_level_::vVELOCITY);
          RCLCPP_INFO(
            rclcpp::get_logger(info_.name), "Stopping %s/velocity joint command interface",
            info_.joints[i].name.c_str());
        }
      }
    }

    // Handle Stop
    if (old_modes.size() > 0) {
      // Criterion: All joints must have the same command mode
      if (!std::all_of(
            old_modes.begin() + 1, old_modes.end(),
            [&](control_integration_level_ mode) { return mode == old_modes[0]; })) {
        RCLCPP_ERROR(
          rclcpp::get_logger(info_.name), "All joints to be stopped must be in the same mode.");
        return hardware_interface::return_type::ERROR;
      }

      // Set control level to undefined in case of a controller deactivation call
      if (old_modes.size() == VENTION_DOF) {
        vention_control_level_ = control_integration_level_::vUNDEFINED;
      }
    }

    // Prepare starting command modes
    std::vector<control_integration_level_> new_modes = {};
    for (const std::string & key : start_interfaces) {
      for (std::size_t i = 0; i < VENTION_DOF; i++) {
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
          new_modes.push_back(control_integration_level_::vVELOCITY);
          RCLCPP_INFO(
            rclcpp::get_logger(info_.name), "Starting %s/velocity joint command interface",
            info_.joints[i].name.c_str());
        }
      }
    }

    // Handle Start
    if (new_modes.size() > 0) {
      // Criterion: All joints must have the same command mode
      if (!std::all_of(
            new_modes.begin() + 1, new_modes.end(),
            [&](control_integration_level_ mode) { return mode == new_modes[0]; })) {
        RCLCPP_ERROR(
          rclcpp::get_logger(info_.name),
          "All joints to be started must be the same command mode.");
        return hardware_interface::return_type::ERROR;
      }
    }

    vention_control_level_ =
      new_modes.size() > 0 ? new_modes[0] : control_integration_level_::vUNDEFINED;
  }

  return hardware_interface::return_type::OK;
}

// Configure: init api, make sure joint number matches robot
hardware_interface::CallbackReturn VentionHW::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  bool connection_established_ = false;
  bool recieve_buffer_flushed_ = false;
  bool flush_start_up_buffer = false;

  connection_established_ = vention.telnetConnect(ip_address);
  if (!connection_established_) {
    RCLCPP_ERROR(
      rclcpp::get_logger(info_.name),
      "Vention connection not established. Please ensure that you are providing the right IP "
      "address.");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  flush_start_up_buffer = vention.telnetRecv(ignore, true);
  if (!flush_start_up_buffer) {
    RCLCPP_ERROR(
      rclcpp::get_logger(info_.name),
      "Start up failure: Receive buffer not flushed. See Vention API's telnetRecv function for "
      "more information.");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  recieve_buffer_flushed_ = vention.isReady();
  if (!recieve_buffer_flushed_) {
    RCLCPP_ERROR(
      rclcpp::get_logger(info_.name),
      "Vention not yet ready to accept command because last recieve buffer not flushed, Consider "
      "pressing the reset button on the Vention control pendant.");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  while (!vention.isMotionCompleted()) {
    std::this_thread::sleep_for(1s);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VentionHW::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "on_activate call successful");
  for (std::size_t i = 0; i < VENTION_DOF; i++) {
    vention_states_positions_old_[i] = vention_states_positions_[i];
  }

  vention.immediateAbsoluteMove({ VentionAPI::ARM_LEFT, VentionAPI::ARM_RIGHT }, { 200.0, 200.0 }, true);

  commandSent = true;
  vention_async_thread_ =
    std::make_shared<std::thread>(&VentionHW::vention_async_thread_callback, this);

  return hardware_interface::CallbackReturn::SUCCESS;
}

void VentionHW::vention_async_thread_callback()
{
  while (!vention_async_thread_shutdown_) {
    checkVentionReadWrite();
    // Slow down the thread to 10 Hz i.e. the control frequency of the Vention HW interface
    std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }
}

void VentionHW::checkVentionReadWrite()
{
  /*
  The order of read and write is important. Calling the write operation before read 
  can cause the actuators to move to an undesired position. This is because the write
  operation expects valid states to be read before it can send the commands.
  */

  // Read first
  double left_vention_pos =
    vention.getPosition(VentionAPI::ARM_LEFT) / 1000.0;  // Convert to meters
  double right_vention_pos =
    vention.getPosition(VentionAPI::ARM_RIGHT) / 1000.0;  // Convert to meters

  // Update states if the read values are not NaN
  if (!(std::isnan(left_vention_pos) && std::isnan(right_vention_pos))) {
    vention_states_positions_[1] = right_vention_pos;
    vention_states_positions_[0] = left_vention_pos;
  }

  // Calculate velocities and accelerations
  for (std::size_t i = 0; i < VENTION_DOF; i++) {
    vention_states_velocities_[i] =
      (vention_states_positions_[i] - vention_states_positions_old_[i]) / dt;
    vention_states_accelerations_[i] =
      (vention_states_velocities_[i] - vention_states_velocities_old_[i]) / dt;
    vention_states_positions_old_[i] = vention_states_positions_[i];
    vention_states_velocities_old_[i] = vention_states_velocities_[i];
  }

  // Write next
  if (!(std::isnan(vention_velocity_commands_old_[0]) &&
        std::isnan(vention_velocity_commands_old_[1]))) {
    if (
      // If either of the actuators are to be commanded
      std::abs(vention_velocity_commands_old_[0] - vention_velocity_commands_[0]) >
        VENTION_VELOCITY_COMMAND_THRESHOLD ||
      std::abs(vention_velocity_commands_old_[1] - vention_velocity_commands_[1]) >
        VENTION_VELOCITY_COMMAND_THRESHOLD) {
      commandSent = false;
    } else if (
      // If both the actuators are to be commanded
      std::abs(vention_velocity_commands_old_[0] - vention_velocity_commands_[0]) >
        VENTION_VELOCITY_COMMAND_THRESHOLD &&
      std::abs(vention_velocity_commands_old_[1] - vention_velocity_commands_[1]) >
        VENTION_VELOCITY_COMMAND_THRESHOLD) {
      commandSent = false;
    }
  }

  // Write velocity commands into a buffer for commandSent check
  for (std::size_t i = 0; i < VENTION_DOF; i++) {
    vention_velocity_commands_old_[i] = vention_velocity_commands_[i];
  }
  if (commandSent == false) {
    double scaled_vel_left = vention_velocity_commands_[1] * 1000;   // Convert to mm/s
    double scaled_vel_right = vention_velocity_commands_[0] * 1000;  // Convert to mm/s

    /* 
    Limit the velocity commands to be under 85 mm/s. This is the maximum velocity that the actuators can handle
    under the 35 kgs payload (27 kg arm + 1 kg hand + 5 kg mounting plate.)
    */
    if (std::abs(scaled_vel_left) > VENTION_MAX_VELOCITY) {
      scaled_vel_left = scaled_vel_left / std::abs(scaled_vel_left) * VENTION_MAX_VELOCITY;
    }
    if (std::abs(scaled_vel_right) > VENTION_MAX_VELOCITY) {
      scaled_vel_right = scaled_vel_right / std::abs(scaled_vel_right) * VENTION_MAX_VELOCITY;
    }

    // Send velocity commands to the actuators. Velocity commands are in mm/s and acceleration/deceleration is in mm/s^2
    vention.sendVelocityCommand(VentionAPI::ARM_LEFT, scaled_vel_left, 800.0, false);
    vention.sendVelocityCommand(VentionAPI::ARM_RIGHT, scaled_vel_right, 800.0, false);

    commandSent = true;
  }
}

hardware_interface::CallbackReturn VentionHW::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  /* Change the control level flag to undefined, stop the while loop by 
  setting the shutdown flag to true and join the thread */
  vention_control_level_ = control_integration_level_::vUNDEFINED;
  vention_async_thread_shutdown_ = true;
  vention_async_thread_->join();

  // Stop the lead screw actuator by commanding 0.0 mm/s velocity and 400 mm/s^2 deceleration
  vention.sendVelocityCommand(VentionAPI::ARM_LEFT, 0.0, 400.0, false);
  vention.sendVelocityCommand(VentionAPI::ARM_RIGHT, 0.0, 400.0, false);
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "on_deactivate call successful");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VentionHW::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "on_cleanup call successful");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Shutdown: Disconnect telnet
hardware_interface::CallbackReturn VentionHW::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  vention.telnetDisconnect();
  while (!vention.telnetDisconnect()) {
    std::this_thread::sleep_for(1s);
  }
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "on_shutdown call successful");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Error: Deactivate, cleanup, shutdown
hardware_interface::CallbackReturn VentionHW::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  on_deactivate(rclcpp_lifecycle::State());
  on_cleanup(rclcpp_lifecycle::State());
  on_shutdown(rclcpp_lifecycle::State());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type VentionHW::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // This is a placeholder for the async thread started in on_activate
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VentionHW::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // This is a placeholder for the async thread started in on_activate
  return hardware_interface::return_type::OK;
}

};  // namespace geodude_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(geodude_hardware::VentionHW, hardware_interface::SystemInterface)
