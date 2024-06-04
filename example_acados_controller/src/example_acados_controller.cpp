// Copyright 2024, ICube Laboratory, University of Strasbourg
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

#include "example_acados_controller/example_acados_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace example_acados_controller
{
using hardware_interface::LoanedCommandInterface;

ExampleAcadosController::ExampleAcadosController()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  joints_command_subscriber_(nullptr)
{
}

CallbackReturn ExampleAcadosController::on_init()
{
  try {
    // Declare parameters
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());

  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ExampleAcadosController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // getting the names of the joints to be controlled
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return CallbackReturn::FAILURE;
  }

  if (joint_names_.size() != 2) {
    RCLCPP_ERROR(get_node()->get_logger(), "expected exactly 2 joints, got %li", joint_names_.size());
    return CallbackReturn::FAILURE;
  }

  // the desired joint trajectory is queried from the commands topic
  // and passed to update via a rt pipe
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/reference", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) {rt_command_ptr_.writeFromNonRT(msg);});

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ExampleAcadosController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_ACCELERATION);
  }
  return conf;
}
// The controller requires the current position and velocity states. For this reason
// it can be directly defined here without the need of getting as parameters.
// The state interface is then deployed to all targeted joints.
controller_interface::InterfaceConfiguration
ExampleAcadosController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return conf;
}

// Fill ordered_interfaces with references to the matching interfaces
// in the same order as in joint_names
template<typename T>
bool get_ordered_interfaces(
  std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
  const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
  for (const auto & joint_name : joint_names) {
    for (auto & command_interface : unordered_interfaces) {
      if (command_interface.get_name() == joint_name + "/" + interface_type) {
        ordered_interfaces.push_back(std::ref(command_interface));
      }
    }
  }

  return joint_names.size() == ordered_interfaces.size();
}

CallbackReturn ExampleAcadosController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::vector<std::reference_wrapper<LoanedCommandInterface>> ordered_interfaces;
  if (
    !get_ordered_interfaces(
      command_interfaces_, joint_names_, hardware_interface::HW_IF_ACCELERATION, ordered_interfaces) ||
    command_interfaces_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu acceleration command interfaces, got %zu",
      joint_names_.size(),
      ordered_interfaces.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  
  // Reset command
  q_acc_cmd_.setZero();

  return CallbackReturn::SUCCESS;
}

CallbackReturn ExampleAcadosController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn ExampleAcadosController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn ExampleAcadosController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn ExampleAcadosController::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}
// main control loop function getting the state interface and writing to the command interface
controller_interface::return_type ExampleAcadosController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // get the data from the subscriber using the rt pipe
  auto reference_joint_trajectory = rt_command_ptr_.readFromRT();

  // retrieve current robot state
  q_pos_(0) = state_interfaces_[0].get_value();
  q_vel_(0) = state_interfaces_[1].get_value();
  q_pos_(1) = state_interfaces_[2].get_value();
  q_vel_(1) = state_interfaces_[3].get_value();

  // retrieve reference position and velocity  
  if (!reference_joint_trajectory || !(*reference_joint_trajectory)) {
    // no command received yet
    q_pos_ref_ = q_pos_;
    q_vel_ref_.setZero();
  } else {
    //checking proxy data validity
    if (((*reference_joint_trajectory)->joint_names.size() != joint_names_.size()) ||
        ((*reference_joint_trajectory)->points[0].positions.size() != joint_names_.size()) ||
        ((*reference_joint_trajectory)->points[0].velocities.size() != joint_names_.size()) ||
        ((*reference_joint_trajectory)->points[0].accelerations.size() != joint_names_.size())
      ){
        RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(), 1000, "command size does not match number of interfaces");
        return controller_interface::return_type::ERROR;
    }
    q_pos_ref_(0) = (*reference_joint_trajectory)->points[0].positions[0];
    q_pos_ref_(1) = (*reference_joint_trajectory)->points[0].positions[1];
    q_vel_ref_(0) = (*reference_joint_trajectory)->points[0].velocities[0];
    q_vel_ref_(1) = (*reference_joint_trajectory)->points[0].velocities[1];
  }

  // Set nmpc initial state and parameters

  // TODO

  // Solve NMPC optimization problem

  // TODO

  // Send command to robot
  command_interfaces_[0].set_value(q_acc_cmd_(0));
  command_interfaces_[1].set_value(q_acc_cmd_(1));
  
  return controller_interface::return_type::OK;
}

}  // namespace example_acados_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  example_acados_controller::ExampleAcadosController, controller_interface::ControllerInterface)
