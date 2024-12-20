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
// Author: Ethan K. Gordon, Atharva Pradhan

#ifndef GEODUDE_HARDWARE_BARRETTHW_H_
#define GEODUDE_HARDWARE_BARRETTHW_H_

#include <Eigen/Core>
#include <cmath>
#include <memory>
#include <mutex>

// ROS
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

// libbarrett
#include <barrett/exception.h>
#include <barrett/products/force_torque_sensor.h>
#include <barrett/products/hand.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems/exposed_output.h>
#include <barrett/systems/rate_limiter.h>
#include <barrett/systems/wam.h>
#include <barrett/units.h>
namespace geodude_hardware
{
// Number of joints in the Barrett WAM
constexpr std::size_t WAM_DOF = 7;
  
// Number of joints in the Barrett Hand
constexpr std::size_t HAND_DOF = 4;
  
// Common lower joint limit (rads) for all 4 joints of the Barrett Hand
constexpr double hand_lower_joint_limit = 0.0000;

// Barrett Hand spread joint limit (rads) for j00
constexpr double spread_joint_limit = 3.14159265359;

// Barrett Hand finger joint limits (rads) for j01, j11, j21
constexpr double finger_joint_limit = 2.44346095279;

class BarrettHW : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BarrettHW)
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(WAM_DOF);

  virtual ~BarrettHW();

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

  void hand_read_write_async_thread_();
  void hand_temp_async_thread_();

private:
  void move_joints_to_home_position();
  // enum defining at which control level we are
  // Dumb way of maintaining the command_interface type per joint.
  enum integration_level_t : std::uint8_t {
    kUNDEFINED = 0,
    kIDLE = 1,
    kPOSITION = 2,
    kVELOCITY = 3,
    kEFFORT = 4,
  };
  integration_level_t arm_control_level_;
  integration_level_t hand_control_level_;
  std::string integrationLevelToString(integration_level_t arm_control_level_);

  std::mutex mMutex;

  // Declaring a separate thread for the hand
  bool async_thread_shutdown_;
  std::shared_ptr<std::thread> async_thread_;
  std::shared_ptr<std::thread> puck_temp_async_thread_;

  // Hand read and write callback function
  void checkHandAsyncIO();
  // Hand temperature check callback function
  void checkHandTempAsync();

  // Flag to start hand reading and writing after arm read and write
  bool hand_reading_started_;
  bool hand_writing_started_;

  // Parameters
  bool use_gravcomp_ = false;

  // Depending on the launch argument it points to
  // geodude_hardware/config/left_wam7 or right_wam7
  std::string wam_config_file_;

  // Barrett Storage
  std::shared_ptr<::barrett::ProductManager> hw_product_manager_{nullptr};
  ::barrett::systems::Wam<WAM_DOF> * hw_wam7_ptr_{nullptr};
  ::barrett::Hand * hw_hand_ptr_{nullptr};
  ::barrett::systems::ExposedOutput<jt_type> jt_output_;
  ::barrett::systems::ExposedOutput<jv_type> jv_output_;
  ::barrett::systems::ExposedOutput<jp_type> jp_output_;

  // State buffers for arm and hand
  std::vector<double> arm_states_positions_;
  std::vector<double> arm_states_velocities_;
  std::vector<double> arm_states_efforts_;
  std::vector<double> hand_states_positions_;
  std::vector<double> hand_states_velocities_;
  std::vector<double> hand_states_efforts_;
  int hand_puck_temp_[HAND_DOF];

  // Command buffers for arm and hand
  jp_type arm_commands_positions_;
  jv_type arm_commands_velocities_;
  jt_type arm_commands_efforts_;
  ::barrett::Hand::jp_type hand_commands_positions_;
  ::barrett::Hand::jp_type hand_jnt_pos_prev_;
  ::barrett::Hand::jt_type hand_commands_efforts_;

};  // End class BarrettHW

};  // End namespace geodude_hardware

#endif
