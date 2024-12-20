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
// Authors: Ethan K. Gordon, Atharva Pradhan

#include "geodude_hardware/barrett.hpp"

#include <barrett/detail/stl_utils.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/time.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <csignal>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// ROS
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace geodude_hardware
{

// Destructor: ensure clean shutdown
BarrettHW::~BarrettHW()
{
  // Ignore SIGTERM (buys us 10s to home the robot)
  signal(SIGTERM, SIG_IGN);
  /* If the controller manager is shutdown via Ctrl + C the on_deactivate methods won't be called.
  We therefore need to make sure to actually deactivate the communication
  WARNING: Press ctrl + c only once while the robot is away from home position
  and you intend to home the robot and exit the HW interface. Pressing ctrl + c
  more than once has shown to skip the execution of three methods above, causing
  uncontrolled falling */
  on_deactivate(rclcpp_lifecycle::State());  // locks position before homing
  on_cleanup(rclcpp_lifecycle::State());     // homes the robot
  on_shutdown(rclcpp_lifecycle::State());    // transitions from active to idle
}

// Init: info parameter is passed from the barrett.ros2_control.urdf.xacro
// All command and state buffers are initialized here
hardware_interface::CallbackReturn BarrettHW::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Setting real-time priority");
  struct sched_param sp = {.sched_priority = 50};
  if (sched_setscheduler(0, SCHED_RR, &sp) != 0) {
    RCLCPP_WARN(rclcpp::get_logger(info_.name), "Unable to set priority");
  }

  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger(info_.name), "Initializing joint command and joint state buffers ...");

  arm_states_positions_.resize(WAM_DOF, std::numeric_limits<double>::quiet_NaN());
  arm_states_velocities_.resize(WAM_DOF, std::numeric_limits<double>::quiet_NaN());
  arm_states_efforts_.resize(WAM_DOF, std::numeric_limits<double>::quiet_NaN());

  hand_states_positions_.resize(HAND_DOF, std::numeric_limits<double>::quiet_NaN());
  hand_states_velocities_.resize(HAND_DOF, std::numeric_limits<double>::quiet_NaN());
  hand_states_efforts_.resize(HAND_DOF, std::numeric_limits<double>::quiet_NaN());

  for (size_t i = 0; i < WAM_DOF; i++) {
    arm_commands_positions_[i] = std::numeric_limits<double>::quiet_NaN();
    arm_commands_velocities_[i] = 0.0;
  }
  for (size_t i = 0; i < HAND_DOF; i++) {
    hand_commands_positions_[i] = std::numeric_limits<double>::quiet_NaN();
    hand_commands_efforts_[i] = std::numeric_limits<double>::quiet_NaN();
    hand_jnt_pos_prev_[i] = 0.0;
  }

  hand_control_level_ = integration_level_t::kIDLE;
  arm_control_level_ = integration_level_t::kIDLE;
  async_thread_shutdown_ = false;  // changes to true on deactivate
  hand_reading_started_ = false;
  hand_writing_started_ = false;

  // Checking for state interfaces
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Checking for state interfaces from URDF...");

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    /* BarrettHW has exactly 3 state interfaces
    and 3 command interfaces on each joint */
    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger(info_.name), "Joint '%s' has %zu state interfaces. 3 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const hardware_interface::InterfaceInfo & interface_info : joint.state_interfaces) {
      if (!(interface_info.name == hardware_interface::HW_IF_POSITION ||
            interface_info.name == hardware_interface::HW_IF_VELOCITY ||
            interface_info.name == hardware_interface::HW_IF_EFFORT)) {
        RCLCPP_FATAL(
          rclcpp::get_logger(info_.name),
          "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
          interface_info.name.c_str(), hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    /* This logic is to accomodate 3 command interfaces for the arm and 2 for the hand
    Checking for command interfaces */
    RCLCPP_INFO(rclcpp::get_logger(info_.name), "Checking for command interfaces from URDF...");
    if (joint.command_interfaces.size() > 3 || joint.command_interfaces.size() < 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger(info_.name),
        "Joint '%s' has %zu command interfaces. Position OR position & velocity expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const hardware_interface::InterfaceInfo & interface_info : joint.command_interfaces) {
      if (!(interface_info.name == hardware_interface::HW_IF_POSITION ||
            interface_info.name == hardware_interface::HW_IF_VELOCITY ||
            interface_info.name == hardware_interface::HW_IF_EFFORT)) {
        RCLCPP_FATAL(
          rclcpp::get_logger(info_.name),
          "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
          interface_info.name.c_str(), hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  // Read other parameters
  // TODO: Verify that this parameter can be passed as a launch argument and default it to true
  if (info_.hardware_parameters.count("use_gravcomp")) {
    use_gravcomp_ = (info_.hardware_parameters.at("use_gravcomp") == "true") ||
                    (info_.hardware_parameters.at("use_gravcomp") == "True");
  }
  if (info_.hardware_parameters.count("wam_config_file")) {
    wam_config_file_ = info_.hardware_parameters.at("wam_config_file");
  }
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Info from URDF read successfully!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/* This function is responsible for exporting the joint states populated in
BarrettHW::read (accessed by the joint state broadcaster) */
std::vector<hardware_interface::StateInterface> BarrettHW::export_state_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Exporting arm joint state interfaces ...");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < WAM_DOF; i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &arm_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &arm_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &arm_states_efforts_[i]));
  }
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Exporting hand joint state interfaces ...");
  for (std::size_t i = WAM_DOF; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &hand_states_positions_[i - WAM_DOF]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &hand_states_velocities_[i - WAM_DOF]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hand_states_efforts_[i - WAM_DOF]));
  }

  return state_interfaces;
}

/* This function is responsible for exporting the command interfaces populated in
BarrettHW::write (accessed by the user/controllers) */
std::vector<hardware_interface::CommandInterface> BarrettHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Exporting arm command interfaces ...");
  for (std::size_t i = 0; i < WAM_DOF; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &arm_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &arm_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &arm_commands_efforts_[i]));
  }
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Exporting hand command interfaces ...");
  for (std::size_t i = WAM_DOF; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &hand_commands_positions_[i - WAM_DOF]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
      &hand_commands_efforts_[i - WAM_DOF]));
  }

  return command_interfaces;
}

/* Control mode switching - This function is called whenever a request to activate/
unload the controllers is received typically through the ros2 control's CLI
or through controller manager's services (eg. /switch_controller)
The logic in this function is extremely important because this function sets
the value of hand/arm_control_level which is used for enabling/disabling
the hand read and write (blocking!) operations through the asynchronous thread */
hardware_interface::return_type BarrettHW::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  /* 
  Without this check, the VentionHW component will call this method
  which would lead to the controller_manager node to error out because 
  BarrettHW::prepare_command_mode_switch is not implemented to handle 
  the VentionHW's command mode switch request, neither does it have the
  necessary joints. Hence, we only call prepare_command_mode_switch for 
  the following cases. 
  */
  if (
    // arm controller activation call:
    (start_interfaces.size() == WAM_DOF && stop_interfaces.empty()) ||
    // arm controller deactivation call:
    (start_interfaces.empty() && stop_interfaces.size() == WAM_DOF) ||
    // hand controller activation call:
    (start_interfaces.size() == HAND_DOF && stop_interfaces.empty()) ||
    // hand controller deactivation call:
    (start_interfaces.empty() && stop_interfaces.size() == HAND_DOF) ||
    // arm controller activation and hand controller deactivation call:
    (start_interfaces.size() == WAM_DOF && stop_interfaces.size() == HAND_DOF) ||
    // hand controller activation and arm controller deactivation call:
    (start_interfaces.size() == HAND_DOF && stop_interfaces.size() == WAM_DOF)) {
    // Only one mode switch at a time
    const std::lock_guard<std::mutex> lock(mMutex);

    RCLCPP_INFO(rclcpp::get_logger(info_.name), "Request to switch joint command interfaces ...");
    // Prepare stopping command modes
    std::vector<integration_level_t> old_modes = {};
    for (const std::string & key : stop_interfaces) {
      for (std::size_t i = 0; i < info_.joints.size(); i++) {
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
          old_modes.push_back(integration_level_t::kPOSITION);
          RCLCPP_INFO(
            rclcpp::get_logger(info_.name), "Stopping %s/position joint command interface",
            info_.joints[i].name.c_str());
        }
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
          old_modes.push_back(integration_level_t::kVELOCITY);
          RCLCPP_INFO(
            rclcpp::get_logger(info_.name), "Stopping %s/velocity joint command interface",
            info_.joints[i].name.c_str());
        }
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
          old_modes.push_back(integration_level_t::kEFFORT);
          RCLCPP_INFO(
            rclcpp::get_logger(info_.name), "Stopping %s/effort joint command interface",
            info_.joints[i].name.c_str());
        }
      }
    }

    // Handle Stop
    if (old_modes.size() > 0) {
      bool stop_arm = false;
      bool stop_hand = false;

      // Criterion: All joints (hand, arm, or both) must be stopped at the same time
      if (old_modes.size() == WAM_DOF) {
        stop_arm = true;
        RCLCPP_INFO(rclcpp::get_logger(info_.name), "Stopping %zu arm joints", old_modes.size());
      } else if (old_modes.size() == HAND_DOF) {
        stop_hand = true;
        RCLCPP_INFO(rclcpp::get_logger(info_.name), "Stopping %zu hand joints", old_modes.size());
      } else if (old_modes.size() == WAM_DOF + HAND_DOF) {
        stop_arm = stop_hand = true;
        RCLCPP_INFO(
          rclcpp::get_logger(info_.name), "Stopping %zu arm + hand joints", old_modes.size());
      } else {
        RCLCPP_ERROR(rclcpp::get_logger(info_.name), "Must stop all joints simultaneously.");
        return hardware_interface::return_type::ERROR;
      }

      // Criterion: All joints must have the same command mode
      if (!std::all_of(old_modes.begin() + 1, old_modes.end(), [&](integration_level_t mode) {
            return mode == (stop_arm ? arm_control_level_ : hand_control_level_);
          })) {
        RCLCPP_ERROR(
          rclcpp::get_logger(info_.name), "All stopped joints must be in the same mode.");
        return hardware_interface::return_type::ERROR;
      }

      if (stop_arm) {
        arm_control_level_ = integration_level_t::kIDLE;
        RCLCPP_INFO(rclcpp::get_logger(info_.name), "Arm joints stopped, no controller active.");
      }
      if (stop_hand) {
        hand_control_level_ = integration_level_t::kIDLE;
        RCLCPP_INFO(rclcpp::get_logger(info_.name), "Hand joints stopped, no controller active.");
      }
    }

    // Prepare starting command modes
    std::vector<integration_level_t> new_modes = {};

    // Set the arm in IDLE/grav comp mode in case of an arm controller deactivation call
    if (start_interfaces.empty() && stop_interfaces.size() == WAM_DOF) {
      for (std::size_t i = 0; i < WAM_DOF; i++) {
        RCLCPP_INFO(
          rclcpp::get_logger(info_.name),
          "Start interfaces are empty, setting arm control level to IDLE");
        new_modes.push_back(integration_level_t::kIDLE);
      }
    }

    for (const std::string & key : start_interfaces) {
      for (std::size_t i = 0; i < info_.joints.size(); i++) {
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
          new_modes.push_back(integration_level_t::kPOSITION);
          RCLCPP_INFO(
            rclcpp::get_logger(info_.name), "Starting %s/position joint command interface",
            info_.joints[i].name.c_str());
        }
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
          new_modes.push_back(integration_level_t::kVELOCITY);
          RCLCPP_INFO(
            rclcpp::get_logger(info_.name), "Starting %s/velocity joint command interface",
            info_.joints[i].name.c_str());
        }
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
          new_modes.push_back(integration_level_t::kEFFORT);
          RCLCPP_INFO(
            rclcpp::get_logger(info_.name), "Starting %s/effort joint command interface",
            info_.joints[i].name.c_str());
        }
      }
    }

    // Handle Start
    if (new_modes.size() > 0) {
      bool start_arm = false;
      bool start_hand = false;

      // Criterion: All joints (hand, arm, or both) must be started at the same time
      if (new_modes.size() == WAM_DOF) {
        start_arm = true;
        RCLCPP_INFO(rclcpp::get_logger(info_.name), "Starting %zu arm joints", new_modes.size());
      } else if (new_modes.size() == HAND_DOF) {
        start_hand = true;
        RCLCPP_INFO(rclcpp::get_logger(info_.name), "Starting %zu hand joints", new_modes.size());
      } else if (new_modes.size() == WAM_DOF + HAND_DOF) {
        start_arm = start_hand = true;
        RCLCPP_INFO(
          rclcpp::get_logger(info_.name), "Starting %zu arm + hand joints", new_modes.size());
      } else {
        RCLCPP_ERROR(rclcpp::get_logger(info_.name), "Must start all joints simultaneously.");
        return hardware_interface::return_type::ERROR;
      }

      // Criterion: All joints must have the same command mode
      if (!std::all_of(new_modes.begin() + 1, new_modes.end(), [&](integration_level_t mode) {
            return mode == new_modes[0];
          })) {
        RCLCPP_ERROR(rclcpp::get_logger(info_.name), "All joints must be the same command mode.");
        return hardware_interface::return_type::ERROR;
      }

      // Criterion: only start hand dof if hand is active
      if (!hw_hand_ptr_ && start_hand) {
        RCLCPP_ERROR(rclcpp::get_logger(info_.name), "Hand currently inactive.");
        return hardware_interface::return_type::ERROR;
      }

      // Criterion: Joints must not be in use.
      bool inUse = (start_arm && (arm_control_level_ != integration_level_t::kIDLE));
      if (inUse) {
        RCLCPP_ERROR(rclcpp::get_logger(info_.name), "Arm joints already in use.");
        return hardware_interface::return_type::ERROR;
      }
      inUse = (start_hand && (hand_control_level_ != integration_level_t::kIDLE));
      if (inUse) {
        RCLCPP_ERROR(rclcpp::get_logger(info_.name), "Hand joints already in use.");
        return hardware_interface::return_type::ERROR;
      }

      if (start_arm) {
        arm_control_level_ = new_modes[0];
        RCLCPP_INFO(
          rclcpp::get_logger(info_.name), "Arm joints started in %s mode.",
          BarrettHW::integrationLevelToString(new_modes[0]).c_str());
      }
      if (start_hand) {
        hand_control_level_ = new_modes[0];
        RCLCPP_INFO(
          rclcpp::get_logger(info_.name), "Hand joints started in %s mode.",
          BarrettHW::integrationLevelToString(new_modes[0]).c_str());
      }
    }

    // Stop the robot and track the correct signal
    if (arm_control_level_ == integration_level_t::kVELOCITY) {
      RCLCPP_INFO(
        rclcpp::get_logger(info_.name), "Starting arm velocity control with zero velocities.");
      for (std::size_t i = 0; i < arm_states_velocities_.size(); i++) {
        arm_commands_velocities_[i] = 0.0;
      }
      jv_output_.setValue(jv_type(0.0));
      hw_wam7_ptr_->trackReferenceSignal(jv_output_.output);
    } else if (arm_control_level_ == integration_level_t::kEFFORT) {
      RCLCPP_INFO(rclcpp::get_logger(info_.name), "Starting arm effort control with zero efforts.");
      for (std::size_t i = 0; i < arm_states_efforts_.size(); i++) {
        arm_commands_efforts_[i] = 0.0;
      }
      jt_output_.setValue(jt_type(0.0));
      hw_wam7_ptr_->trackReferenceSignal(jt_output_.output);
    } else if (
      arm_control_level_ == integration_level_t::kPOSITION ||
      arm_control_level_ == integration_level_t::kUNDEFINED) {
      RCLCPP_INFO(
        rclcpp::get_logger(info_.name), "Starting arm position control with current positions.");
      for (std::size_t i = 0; i < arm_states_positions_.size(); i++) {
        arm_commands_positions_[i] = arm_states_positions_[i];
      }
      jp_output_.setValue(arm_commands_positions_);
      hw_wam7_ptr_->trackReferenceSignal(jp_output_.output);
    } else if (arm_control_level_ == integration_level_t::kIDLE) {
      RCLCPP_INFO(
        rclcpp::get_logger(info_.name),
        "Control level IDLE, setting WAM in free-drive/gravity compensation mode.");
      hw_wam7_ptr_->idle();  // Detaches the supervisory controller from tracking a reference signal
    }

    if (hand_control_level_ == integration_level_t::kPOSITION) {
      RCLCPP_INFO(
        rclcpp::get_logger(info_.name), "Starting hand position control with current positions.");
      for (std::size_t i = 0; i < hand_states_positions_.size(); i++) {
        hand_commands_positions_[i] = hand_states_positions_[i];
      }
      hw_hand_ptr_->setProperty(
        ::barrett::Hand::WHOLE_HAND, ::barrett::Puck::HOLD, ::barrett::Hand::v_type(1.0));
      hw_hand_ptr_->setProperty(
        ::barrett::Hand::WHOLE_HAND, ::barrett::Puck::TSTOP, ::barrett::Hand::v_type(30.0));
      hw_hand_ptr_->setPositionMode();
    } else if (hand_control_level_ == integration_level_t::kEFFORT) {
      RCLCPP_INFO(
        rclcpp::get_logger(info_.name), "Starting hand effort control with zero efforts.");
      for (int i = 0; i < hand_commands_efforts_.size(); i++) {
        hand_commands_efforts_[i] = 0.0;
      }
      /* Setting the HOLD value to 1.0 ensures that the motors stay engaged. Otherwise
    the hand motors will be idled after TSTOP milliseconds. This means setting HOLD to 
    0.0 calls hand.idle() in case of a motor stall condition */
      hw_hand_ptr_->setProperty(
        ::barrett::Hand::WHOLE_HAND, ::barrett::Puck::HOLD, ::barrett::Hand::v_type(1.0));
      /* FIXME: Setting TSTOP to non-zero value will cause finger commands to be ignored when sent 
      after more than 3000 milliseconds of controller activation. This can be observred when 
      the user launches moveit2 and tries to manually command the fingers to a position.
      This bug is not encountered when controller activation and finger commands are sent 
      within the 3000 milliseconds window/spontaneously. For instance, through a script */
      hw_hand_ptr_->setProperty(
        ::barrett::Hand::WHOLE_HAND, ::barrett::Puck::TSTOP, ::barrett::Hand::v_type(300.0));
      /* Uncomment these to fix the Barrett Hand pre-mature breakaway issue for the specific finger/whole hand*/
      // hw_hand_ptr_->setProperty(::barrett::Hand::F2, ::barrett::Puck::OT, 0.0);
      // hw_hand_ptr_->setProperty(::barrett::Hand::F2, ::barrett::Puck::IVEL, 300.0);
      // hw_hand_ptr_->setProperty(::barrett::Hand::F2, ::barrett::Puck::IOFF, 50.0);
      // hw_hand_ptr_->setProperty(::barrett::Hand::F2, ::barrett::Puck::IHIT, 2.0);
      hw_hand_ptr_->setTorqueMode();
    }
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger(info_.name),
      "Ignoring command mode switch request because combination of start and stop interfaces is "
      "not supported by BarrettHW.");
    return hardware_interface::return_type::OK;
  }
}

/* Configure: One time initializations before activation like checking for safety
module, whether or not WAM is zeroed, wake all pucks/motor controllers, initialzing
virtual WAM and Hand objects, hand async thread declaration */
hardware_interface::CallbackReturn BarrettHW::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Get Pretty Stack Traces
  ::barrett::installExceptionHandler();

  // Create ProductManager
  hw_product_manager_.reset(
    new ::barrett::ProductManager(wam_config_file_.empty() ? NULL : wam_config_file_.c_str()));

  // Check Safety Module
  if (!hw_product_manager_->foundSafetyModule()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(info_.name),
      "SafetyModule not found. Is WAM connected to the right port?");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // Check that mode is IDLE
  hw_product_manager_->getSafetyModule()->setMode(::barrett::SafetyModule::IDLE);
  if (hw_product_manager_->getSafetyModule()->getMode() != ::barrett::SafetyModule::IDLE) {
    RCLCPP_ERROR(rclcpp::get_logger(info_.name), "WAM not IDLE. Shift-IDLE and re-configure.");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  hw_product_manager_->getSafetyModule()->setWamZeroed();
  // Check that arm is zero-ed
  if (!hw_product_manager_->getSafetyModule()->wamIsZeroed()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(info_.name),
      "WAM not zero-ed. Run ros2 run geodude_hardware zero_barrett first!");
    return hardware_interface::CallbackReturn::FAILURE;
  }
  hw_product_manager_->wakeAllPucks();

  // Load WAM and (optionally) hand object/s
  bool wam_exists = false;
  bool hand_exists = false;
  switch (info_.joints.size()) {
    case WAM_DOF + HAND_DOF:
      // Check that hand exists
      hand_exists = hw_product_manager_->foundHand();
      if (!hand_exists) {
        RCLCPP_ERROR(rclcpp::get_logger(info_.name), "Requested %ld-DOF HAND not found.", HAND_DOF);
        return hardware_interface::CallbackReturn::FAILURE;
      }
      hw_hand_ptr_ = hw_product_manager_->getHand();
      __attribute__((fallthrough));
    case WAM_DOF:
      // Check that WAM exists
      wam_exists = hw_product_manager_->foundWam7();
      // true to wait for Shift-Activate
      RCLCPP_WARN(rclcpp::get_logger(info_.name), "Shift-Activate the WAM Now");
      if (wam_exists) {
        hw_wam7_ptr_ = hw_product_manager_->getWam7(true);
      } else {
        RCLCPP_ERROR(rclcpp::get_logger(info_.name), "Requested %ld-DOF WAM not found.", WAM_DOF);
        return hardware_interface::CallbackReturn::FAILURE;
      }
      break;
    default:
      RCLCPP_FATAL(
        rclcpp::get_logger(info_.name), "Unsupported Joint Size %ld, options are: 7",
        info_.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize Hand
  if (hand_exists) {
    RCLCPP_INFO(rclcpp::get_logger(info_.name), "Activated, initializing hand...");
    hw_hand_ptr_->initialize();
    RCLCPP_INFO(rclcpp::get_logger(info_.name), "...Hand Initialized!");

    async_thread_ = std::make_shared<std::thread>(&BarrettHW::hand_read_write_async_thread_, this);
    puck_temp_async_thread_ =
      std::make_shared<std::thread>(&BarrettHW::hand_temp_async_thread_, this);

    RCLCPP_INFO(
      rclcpp::get_logger(info_.name), "Starting async thread for hand read/write operations...");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Activate: One time intializations after pressing shift-activate
hardware_interface::CallbackReturn BarrettHW::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // User should have shift-activated the WAM by now
  if (hw_product_manager_->getSafetyModule()->getMode() != ::barrett::SafetyModule::ACTIVE) {
    RCLCPP_ERROR(
      rclcpp::get_logger(info_.name), "WAM not ACTIVE. Shift-Activate before calling on_activate.");

    return hardware_interface::CallbackReturn::FAILURE;
  }

  if (info_.hardware_parameters.count("move_to_initial_joint_position")) {
    std::string param = info_.hardware_parameters.at("move_to_initial_joint_position");
    bool move_to_initial_joint_position = (param == "true") || (param == "True");
    if (move_to_initial_joint_position) {
      move_joints_to_home_position();
    }
  }

  // TODO: Do we need this?
  // Initialize Default Values
  RCLCPP_INFO(
    rclcpp::get_logger(info_.name),
    "On Activate call successful, initializing default values by querying the encoders ...");
  auto jnt_vel = hw_wam7_ptr_->getJointVelocities();
  auto jnt_pos = hw_wam7_ptr_->getJointPositions();
  for (std::size_t i = 0; i < arm_states_velocities_.size(); i++) {
    arm_states_positions_[i] = jnt_pos[i];
    arm_commands_positions_[i] = jnt_pos[i];
    arm_states_velocities_[i] = jnt_vel[i];
    arm_commands_velocities_[i] = 0.0;
  }

  if (hw_hand_ptr_) {
    auto hand_jnt_pos = hw_hand_ptr_->getInnerLinkPosition();
    for (std::size_t i = 0; i < hand_states_velocities_.size(); i++) {
      hand_states_positions_[i] = hand_jnt_pos[i];
      hand_commands_positions_[i] = hand_jnt_pos[i];
      hand_states_velocities_[i] = 0.0;
      hand_commands_efforts_[i] = 0.0;
    }

    /* Enable TSTOP: TIME-TO-STOP (default: 30 milliseconds) This is the time
    after which the hand motors are considered stopped/dont exert torques.
    If this parameter is zero, this means that the motors will continue to
    exert torques even when they are stalled due to an obstacle or joint limits,
    before they start overheating and cause hardware damage */
    hw_hand_ptr_->setProperty(
      ::barrett::Hand::WHOLE_HAND, ::barrett::Puck::TSTOP, ::barrett::Hand::v_type(30.0));

    /* Enable HOLD: values for fingers should be set to 1.0 when using position control
    for the hand to ensure that the motors stay engaged or else you need to call setPositionMode 
    everytime before call setPositionCommand. This can cause a clicking sound, resetting the 
    internal PID loop everytime you call setPositionMode. */

    // Make sure motors stay engaged
    // hw_hand_ptr_->setProperty(
    // ::barrett::Hand::WHOLE_HAND, ::barrett::Puck::HOLD, ::barrett::Hand::v_type(1.0));
    // hw_hand_ptr_->setPositionMode();

    /* Command the fingers to a positive, non-zero value on activate because sometimes finger 
    joints show a finite negative value (say -0.00001) causing MoveIt2 to error out */
    // hw_hand_ptr_->setPositionCommand(::barrett::Hand::jp_type(0.025), ::barrett::Hand::WHOLE_HAND);
    hw_hand_ptr_->getProperty(::barrett::Puck::THERM, hand_puck_temp_, false);
    for (std::size_t i = 0; i < HAND_DOF; i++) {
      RCLCPP_INFO(
        rclcpp::get_logger(info_.name), "Hand puck %ld temperature: %u", i, hand_puck_temp_[i]);
    }
  }

  // Set up gravity compensation
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Setting up gravity compensation ...");
  hw_wam7_ptr_->gravityCompensate(use_gravcomp_);
  if (hw_wam7_ptr_->isGravityCompensated() != use_gravcomp_) {
    /* If the WAM is not gravity compensated, ask the supervisory controller to hold position
    to prevent the arm from uncontrolled falling */
    RCLCPP_INFO(
      rclcpp::get_logger(info_.name), "WAM not gravity compensated. Locking position ...");
    jp_output_.setValue(jnt_pos);
    hw_wam7_ptr_->trackReferenceSignal(jp_output_.output);
  }

  // NOTE: This is useful if the controllers are launched right after the robot is activated, before launching moveit2
  // if (arm_control_level_ == integration_level_t::kVELOCITY) {
  //   jv_output_.setValue(jv_type(0.0));
  //   hw_wam7_ptr_->trackReferenceSignal(jv_output_.output);
  // }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// First call when user presses ctrl + c
hardware_interface::CallbackReturn BarrettHW::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  /* Locking position on deactivate because we still want the supervisory
  controller (in libbarrett) to hold position or the arm yields to own weight. */
  if (hw_wam7_ptr_) {
    RCLCPP_INFO(rclcpp::get_logger(info_.name), "Locking position on deactivate ...");
    jp_output_.setValue(hw_wam7_ptr_->getJointPositions());
    hw_wam7_ptr_->trackReferenceSignal(jp_output_.output);
  }

  // Reset the hand parameters to default to ensure clean shutdown
  if (hw_hand_ptr_) {
    hw_hand_ptr_->setProperty(
      ::barrett::Hand::WHOLE_HAND, ::barrett::Puck::HOLD, ::barrett::Hand::v_type(1.0));
    hw_hand_ptr_->setProperty(
      ::barrett::Hand::WHOLE_HAND, ::barrett::Puck::TSTOP, ::barrett::Hand::v_type(30.0));
    hw_hand_ptr_->setPositionMode();
    hw_hand_ptr_->idle();
  }

  RCLCPP_INFO(rclcpp::get_logger(info_.name), "on_deactivate call successful");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Homes the WAM and shuts down the hand async thread
hardware_interface::CallbackReturn BarrettHW::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (hw_product_manager_ && hw_wam7_ptr_) {
    if (hw_product_manager_->getSafetyModule()->getMode() != ::barrett::SafetyModule::ACTIVE) {
      RCLCPP_ERROR(
        rclcpp::get_logger(info_.name), "WAM not ACTIVE. Shift-Activate to home on_cleanup.");
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    RCLCPP_INFO(rclcpp::get_logger(info_.name), "Homing the WAM ...");
    hw_wam7_ptr_->moveHome(true, 0.5);  // Move Home
    /* Moving home with lower velocities means lower torque produced. Hence, the WAM might
    not always home successfully if it is too far away from the home position. If we increase the
    velocity, however, the WAM might jump home with higher torque, which is a trade-off. */
    if (async_thread_) {
      async_thread_shutdown_ = true;
      async_thread_->join();
      async_thread_.reset();
      puck_temp_async_thread_->join();
      puck_temp_async_thread_.reset();
    }
  }
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "on_cleanup call successful");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/* TIP: You can always programmatically go from a higher state to a lower state
on the control pendant (eg. Active to Idle) but not vice-versa for safety reasons.
This function transitions the control pendant to shift-idle */
hardware_interface::CallbackReturn BarrettHW::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (hw_product_manager_) {
    hw_product_manager_->getSafetyModule()->setMode(::barrett::SafetyModule::IDLE);
  }
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "on_shutdown call successful");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// TODO: How can this be utilized?
hardware_interface::CallbackReturn BarrettHW::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  on_deactivate(previous_state);
  on_cleanup(previous_state);
  on_shutdown(previous_state);

  return hardware_interface::CallbackReturn::SUCCESS;
}

void BarrettHW::hand_temp_async_thread_()
{
  /* This thread is joined when on_deactivate is called and doesn't stop before ctrl+c
  because we ideally want the hand thread to always keep runnning asynchronously
  and continuously evaluate the puck temperature based on the hand control level */
  while (!async_thread_shutdown_) {
    // if (hand_control_level_ == integration_level_t::kEFFORT || hand_control_level_ == integration_level_t::kPOSITION) {
    //   checkHandTempAsync();
    // }
    /* Always monitor puck temperatures at start-up. Sometimes, when the hand is 
    opened with higher torques, fingers tend to get stuck at the joint stops. This can
    conflict with the fingers trying to move to an initial position (as done in on_activate)
    If the controller is unable to move the fingers, it might overt exert the motor torques
    and cause the motors to heat up. This can be avoided by monitoring the puck temperatures.*/
    checkHandTempAsync();
    std::this_thread::sleep_for(std::chrono::nanoseconds(2000000000));  // 2 seconds
  }
}

void BarrettHW::hand_read_write_async_thread_()
{
  /* This thread is joined when on_deactivate is called and doesn't stop before ctrl+c
  because we ideally want the hand thread to always keep runnning asynchronously
  and continuously evaluate the hand control level condition as a switch to enable/disable
  hand reading and writing. Leaving this thread running until shutdown wouldn't block
  the arm because it is asynchronous */
  while (!async_thread_shutdown_) {
    if (hand_reading_started_ && hand_writing_started_) {
      checkHandAsyncIO();
    }
    if (hand_control_level_ == integration_level_t::kEFFORT) {
      // Recommended frequency for effort control: 200 Hz
      std::this_thread::sleep_for(std::chrono::nanoseconds(5000000));
    } else {  // hand_control_level_ == Position or undefined
      std::this_thread::sleep_for(std::chrono::nanoseconds(20000000));
    }
  }
}

// Hand read and write callback function
void BarrettHW::checkHandAsyncIO()
{
  if (!hand_reading_started_) {
    return;
  }
  // Hand read operation
  if (hw_hand_ptr_) {
    if (hand_control_level_ != integration_level_t::kIDLE) {
      if (hand_control_level_ == integration_level_t::kPOSITION) {
        auto hand_read_start = std::chrono::high_resolution_clock::now();  // Start time
        hw_hand_ptr_->update(
          ::barrett::Hand::S_FINGERTIP_TORQUE | ::barrett::Hand::S_POSITION, true);
        auto hand_jnt_eff = hw_hand_ptr_->getFingertipTorque();
        auto hand_jnt_pos = hw_hand_ptr_->getInnerLinkPosition();
        auto hand_read_end = std::chrono::high_resolution_clock::now();  //   End time
        auto hand_read_duration =
          std::chrono::duration_cast<std::chrono::nanoseconds>(hand_read_end - hand_read_start);
        for (std::size_t i = 0; i < HAND_DOF; i++) {
          hand_states_positions_[i] = hand_jnt_pos[i];
          hand_states_velocities_[i] = (hand_jnt_pos[i] - hand_jnt_pos_prev_[i]) /
                                       (hand_read_duration.count() * 1e-9);  // dx/dt
          hand_states_efforts_[i] = hand_jnt_eff[i];
        }
        hand_jnt_pos_prev_ = hand_jnt_pos;
      } else if (hand_control_level_ == integration_level_t::kEFFORT) {
        auto hand_read_start = std::chrono::high_resolution_clock::now();  // Start time
        hw_hand_ptr_->update(
          ::barrett::Hand::S_FINGERTIP_TORQUE | ::barrett::Hand::S_POSITION, true);
        auto hand_jnt_eff = hw_hand_ptr_->getFingertipTorque();
        auto hand_jnt_pos = hw_hand_ptr_->getInnerLinkPosition();
        auto hand_read_end = std::chrono::high_resolution_clock::now();  //   End time
        auto hand_read_duration =
          std::chrono::duration_cast<std::chrono::nanoseconds>(hand_read_end - hand_read_start);
        for (std::size_t i = 0; i < HAND_DOF; i++) {
          double lower_limit = hand_lower_joint_limit;
          double upper_limit = (i < HAND_DOF - 1) ? finger_joint_limit : spread_joint_limit;
          hand_states_positions_[i] = std::clamp(hand_jnt_pos[i], lower_limit, upper_limit);
          hand_states_velocities_[i] = (hand_jnt_pos[i] - hand_jnt_pos_prev_[i]) /
                                       (hand_read_duration.count() * 1e-9);  // dx/dt
          hand_states_efforts_[i] = hand_jnt_eff[i];
        }
        hand_jnt_pos_prev_ = hand_jnt_pos;
      }
    }
  }
  if (!hand_writing_started_) {
    return;
  }
  // Hand write operation
  if (hw_hand_ptr_) {
    if (hand_control_level_ != integration_level_t::kIDLE) {
      /* TODO: Maybe only command the fingers if previous command - this command > some threshold
      to conserve write latency because write operation demands CANbus bandwidth. In addition, 
      this might also help with the FIXME issue in prepare_command_mode_switch */
      if (hand_control_level_ == integration_level_t::kPOSITION) {
        // RCLCPP_INFO(rclcpp::get_logger(info_.name), "Writing hand positions ...");
        for (size_t i = 0; i < HAND_DOF; i++) {
          if (hand_commands_positions_[i] < 0.0) {
            RCLCPP_INFO(
              rclcpp::get_logger(info_.name),
              "hand joint positions need to be greater equals zero!");
            hand_commands_positions_[i] = 0.0;
          }
        }
        hw_hand_ptr_->setPositionCommand(hand_commands_positions_, ::barrett::Hand::WHOLE_HAND);
      } else if (hand_control_level_ == integration_level_t::kEFFORT) {
        hw_hand_ptr_->setTorqueCommand(hand_commands_efforts_, ::barrett::Hand::WHOLE_HAND);
      }
    }
  }
}

void BarrettHW::checkHandTempAsync()
{
  if (!hand_reading_started_) {
    return;
  }
  // Read hand puck temperatures
  // TODO: Make this a service call
  hw_hand_ptr_->getProperty(::barrett::Puck::THERM, hand_puck_temp_, false);
  for (std::size_t i = 0; i < HAND_DOF; i++) {
    if (hand_puck_temp_[i] > 50) {
      RCLCPP_WARN(
        rclcpp::get_logger(info_.name),
        "Puck %ld temperature: %d, if increasing, press ctrl+c on this window", i,
        hand_puck_temp_[i]);
    }
    // RCLCPP_INFO(
    //   rclcpp::get_logger(info_.name), "Hand puck %ld temperature: %d", i, hand_puck_temp_[i]);
    if (hand_puck_temp_[i] > 60) {
      RCLCPP_ERROR(
        rclcpp::get_logger(info_.name), "Puck %ld temperature: %d", i, hand_puck_temp_[i]);
      RCLCPP_ERROR(
        rclcpp::get_logger(info_.name),
        "Puck %ld temperature exceeded 60 degrees, deactivating hand ...", i);
      hand_control_level_ = integration_level_t::kIDLE;
      hw_hand_ptr_->idle();
    }
  }
}

/* Arm write operation. This function is always called by iteratively at frequency
set by the controller manager (eg. 500Hz) in a read-update-write loop by the
controller manager for the entire duration that the HW interface is running/active */

/* WARNING: Real-time function. DO NOT BLOCK. For debugging and printing, comment out
everything and all the trackReferenceSignal() calls but the return statement */

hardware_interface::return_type BarrettHW::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  hand_writing_started_ = true;

  if (arm_control_level_ == integration_level_t::kVELOCITY) {
    jv_output_.setValue(arm_commands_velocities_);
  } else if (arm_control_level_ == integration_level_t::kEFFORT) {
    jt_output_.setValue(arm_commands_efforts_);
  } else if (arm_control_level_ == integration_level_t::kPOSITION) {
    jp_output_.setValue(arm_commands_positions_);
  }
  return hardware_interface::return_type::OK;
}

/* Arm read operation. This function is always called the controller manager
in a read-update-write loop for the entire duration that the HW interface is running
WARNING: Real-time function. DO NOT BLOCK. */

hardware_interface::return_type BarrettHW::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  hand_reading_started_ = true;
  auto jnt_pos = hw_wam7_ptr_->getJointPositions();
  auto jnt_vel = hw_wam7_ptr_->getJointVelocities();
  auto jnt_eff = hw_wam7_ptr_->getJointTorques();

  for (std::size_t i = 0; i < arm_states_positions_.size(); i++) {
    arm_states_positions_[i] = jnt_pos[i];
    arm_states_velocities_[i] = jnt_vel[i];
    arm_states_efforts_[i] = jnt_eff[i];
  }

  return hardware_interface::return_type::OK;
}

void BarrettHW::move_joints_to_home_position()
{
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Moving joints to home position");

  std::string package_path;
  try {
    package_path = ament_index_cpp::get_package_share_directory("geodude_hardware");
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger(info_.name), "Unable to get package path for geodude_hardware");
    return;
  }

  std::ifstream f(package_path + "/config/joint_poses.json");
  if (!f.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger(info_.name), "Unable to located file.");
    return;
  }
  json json_data = json::parse(f);

  std::string debug = "Loaded: " + json_data.dump(4);
  RCLCPP_INFO(rclcpp::get_logger(info_.name), debug.c_str());

  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Moving.");

  if (hw_wam7_ptr_) {
    auto data = json_data["arm_joint_angles"];
    jp_type arm_target_joint_positions;  // = data;
    for (size_t i = 0; i < WAM_DOF; i++) {
      arm_target_joint_positions[i] = data[i];
    }
    // joint poses, blocking, velocity, acceleration
    hw_wam7_ptr_->moveTo(arm_target_joint_positions, true, 0.2, 0.2);
  }

  if (hw_hand_ptr_) {
    auto data = json_data["finger_joint_angles"];
    barrett::Hand::jp_type finger_target_joint_positions;
    for (size_t i = 0; i < HAND_DOF; i++) {
      finger_target_joint_positions[i] = data[i];
    }
    hw_hand_ptr_->trapezoidalMove(finger_target_joint_positions, ::barrett::Hand::WHOLE_HAND, true);
  }
}

std::string BarrettHW::integrationLevelToString(integration_level_t arm_control_level_)
{
  switch (arm_control_level_) {
    case integration_level_t::kIDLE:
      return "IDLE";
    case integration_level_t::kVELOCITY:
      return "VELOCITY";
    case integration_level_t::kEFFORT:
      return "EFFORT";
    case integration_level_t::kPOSITION:
      return "POSITION";
    case integration_level_t::kUNDEFINED:
      return "UNDEFINED";
    default:
      return "UNKNOWN";
  }
}

};  // namespace geodude_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(geodude_hardware::BarrettHW, hardware_interface::SystemInterface)
