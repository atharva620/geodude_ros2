// Copyright 2023 Personal Robotics Lab, University of Washington
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
// Author: Atharva Pradhan

#ifndef GEODUDE_HARDWARE_VENTIONHW_H_
#define GEODUDE_HARDWARE_VENTIONHW_H_

#include <Eigen/Core>
#include <cmath>
#include <memory>
#include <mutex>

// ROS
#include "geodude_hardware/vention_api.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

namespace geodude_hardware
{
// 2 DOF Vention Actuator
constexpr std::size_t VENTION_DOF = 2;

/* Do not increase this value beyond 85.0 mm/s if the Vention actuator is
  configured as a *custom* type. */
constexpr double VENTION_MAX_VELOCITY = 85.0;  // unit: mm/s

/* If difference (m/s) between two successive velocities read by the controller
  is greater than 0.0001 m/s, only then command the actuators to move. This is 
  implemented to ensure that insignificant commands are not sent to the actuators.
  Sending redundant commands can also block the 500 Hz high frequency control loop */
constexpr double VENTION_VELOCITY_COMMAND_THRESHOLD = 0.0001;  // unit: m/s

/* 
This hardware interface abstracts the Vention API into ros2_control. 
The Vention API is a Telnet interface that handles low-level send and 
recieve commands to and from the MachineMotionV2/Vention control machine
over the Ethernet.
*/
class VentionHW : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(VentionHW)

  virtual ~VentionHW();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void vention_async_thread_callback();

private:
  // If this IP address changes in the future, log onto the router update this IP address
  std::string ip_address = "192.168.1.152";
  bool commandSent = false;

  enum control_integration_level_ : std::uint8_t {
    vUNDEFINED = 0,
    vVELOCITY = 1,
  };

  control_integration_level_ vention_control_level_;

  bool vention_async_thread_shutdown_;
  static constexpr double dt = 0.1;
  std::shared_ptr<std::thread> vention_async_thread_;
  void checkVentionReadWrite();

  std::mutex mMutex;
  std::string ignore;
  VentionAPI vention;

  std::vector<double> vention_states_positions_;
  std::vector<double> vention_states_velocities_;
  std::vector<double> vention_position_commands_;
  std::vector<double> vention_velocity_commands_;
  std::vector<double> vention_velocity_commands_old_;
  std::vector<double> vention_states_positions_old_;
  std::vector<double> vention_states_velocities_old_;
  std::vector<double> vention_states_accelerations_;
};  // End class VentionHW

};  // End namespace geodude_hardware

#endif
