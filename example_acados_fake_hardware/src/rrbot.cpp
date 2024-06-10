// Copyright 2022, ICube Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "example_acados_fake_hardware/rrbot.hpp"
#include "rclcpp/rclcpp.hpp"

namespace example_acados_fake_hardware
{
// ------------------------------------------------------------------------------------------
CallbackReturn RRbot::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Allocate memory
  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_acceleration_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Check description compatibility
  // rrbot has exactly 2 joints
  if (info_.joints.size() != 2) {
    RCLCPP_FATAL(
      rclcpp::get_logger("RRbot"),
      "RRbot description has %li joints. 2 expected.", info_.joints.size());
    return CallbackReturn::ERROR;
  }
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // rrbot has currently exactly 3 state and 1 command interface on each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRbot"),
        "Joint '%s' has %li command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRbot"),
        "Joint '%s' has %s command interfaces. '%s' expected", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRbot"),
        "Joint '%s' has %li state interface. 3 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRbot"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRbot"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRbot"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }

  // initialize states
  for (uint i = 0; i < info_.joints.size(); i++) {
    hw_states_position_[i] = std::stod(info_.joints[i].state_interfaces[0].initial_value);
    hw_states_acceleration_[i] = 0.0;
    hw_states_velocity_[i] = 0.0;
  }
  q_pos_ << hw_states_position_[0], hw_states_position_[1];
  q_vel_ << 0.0, 0.0,
  q_acc_ << 0.0, 0.0;
  return CallbackReturn::SUCCESS;
}
// ------------------------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
RRbot::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
  }
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));
  }

  return state_interfaces;
}
// ------------------------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
RRbot::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]));
  }

  return command_interfaces;
}
// ------------------------------------------------------------------------------------------
hardware_interface::return_type RRbot::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  hw_states_position_[0] = q_pos_(0);
  hw_states_position_[1] = q_pos_(1);

  hw_states_velocity_[0] = q_vel_(0);
  hw_states_velocity_[1] = q_vel_(1);

  hw_states_effort_[0] = tau_(0);
  hw_states_effort_[1] = tau_(1);

  return hardware_interface::return_type::OK;
}
// ------------------------------------------------------------------------------------------
hardware_interface::return_type RRbot::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  tau_(0) = (std::isnan(hw_commands_effort_[0])) ? 0.0 : hw_commands_effort_[0];
  tau_(1) = (std::isnan(hw_commands_effort_[1])) ? 0.0 : hw_commands_effort_[1];

  bmat_(
    0,
    0) = m1_ * lc1_ * lc1_ + m2_ * (l1_ * l1_ + lc2_ * lc2_ + 2 * l1_ * lc2_ * cos(q_pos_(1))) +
    i1_ + i2_;
  bmat_(0, 1) = m2_ * (lc2_ * lc2_ + l1_ * lc2_ * cos(q_pos_(1))) + i2_;
  bmat_(1, 0) = m2_ * (lc2_ * lc2_ + l1_ * lc2_ * cos(q_pos_(1))) + i2_;
  bmat_(1, 1) = m2_ * lc2_ * lc2_ + i2_;

  cvec_(0) = (-m2_ * l1_ * lc2_ * q_vel_(1) * sin(q_pos_(1))) * q_vel_(0) +
    (-m2_ * l1_ * lc2_ * (q_vel_(0) + q_vel_(1)) * sin(q_pos_(1))) * q_vel_(1);
  cvec_(1) = (m2_ * l1_ * lc2_ * q_vel_(0) * sin(q_pos_(1))) * q_vel_(0);

  gvec_(0) =
    (m1_ * lc1_ * cos(q_pos_(0)) + m2_ *
    (l1_ * cos(q_pos_(0)) + lc2_ * cos(q_pos_(0) + q_pos_(1)))) * g_;
  gvec_(1) = m2_ * lc2_ * cos(q_pos_(0) + q_pos_(1)) * g_;

  // Add small viscous friction
  auto tau_viscosity = 0.1 * q_vel_;

  // Compute acceleration
  q_acc_ = bmat_.inverse() * (tau_ - cvec_ - gvec_ - tau_viscosity);

  // Update velocity and position
  q_vel_ = q_vel_ + q_acc_ * (period.nanoseconds() * 1e-9);
  q_pos_ = q_pos_ + q_vel_ * (period.nanoseconds() * 1e-9);

  return hardware_interface::return_type::OK;
}

}  // namespace example_acados_fake_hardware
// ---------------------------------------------------------------------------
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  example_acados_fake_hardware::RRbot, hardware_interface::SystemInterface)
