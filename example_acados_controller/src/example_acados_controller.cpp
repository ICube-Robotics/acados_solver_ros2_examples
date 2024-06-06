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
#include <utility>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// Utils for Acados solver plugins (to adjust cost, constraints, etc.)
#include "acados_solver_base/acados_solver_utils.hpp"

namespace example_acados_controller
{
using hardware_interface::LoanedCommandInterface;

ExampleAcadosController::ExampleAcadosController()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  reference_subscriber_(nullptr)
{
}

CallbackReturn ExampleAcadosController::on_init()
{
  try {
    // Declare parameters
    // this->declare_parameter("update_rate");
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::string>("nmpc.plugin_name", std::string());
    auto_declare<int>("nmpc.N", 10);
    auto_declare<double>("nmpc.Ts", -1);
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ExampleAcadosController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // get parameters
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  auto nmpc_plugin_name = get_node()->get_parameter("nmpc.plugin_name").as_string();
  auto N = get_node()->get_parameter("nmpc.N").as_int();
  auto Ts = get_node()->get_parameter("nmpc.Ts").as_double();

  // Check parameters
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return CallbackReturn::FAILURE;
  }

  if (joint_names_.size() != 2) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "expected exactly 2 joints, got %li",
      joint_names_.size());
    return CallbackReturn::FAILURE;
  }

  if (Ts <= 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "'nmpc.Ts' parameter must be positive");
    return CallbackReturn::FAILURE;
  }
  if (N <= 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "'nmpc.N' parameter must be positive");
    return CallbackReturn::FAILURE;
  }
  if (nmpc_plugin_name.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'nmpc_plugin_name' parameter was empty");
    return CallbackReturn::FAILURE;
  }

  // Initialize NMPC solver
  RCLCPP_INFO(
    get_node()->get_logger(), "Loading Acados solver plugin '%s'...",
    nmpc_plugin_name.c_str()
  );

  acados_solver_loader_ =
    std::make_shared<pluginlib::ClassLoader<acados::AcadosSolver>>(
    "acados_solver_base", "acados::AcadosSolver");
  acados_solver_ = std::unique_ptr<acados::AcadosSolver>(
    acados_solver_loader_->createUnmanagedInstance(nmpc_plugin_name));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Initializing Acados solver with N = %li and Ts = %f", N, Ts);

  if (0 != acados_solver_->init(N, Ts)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure the Acados solver!");
    return CallbackReturn::FAILURE;
  }
  if (0 != acados_solver_->reset()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to reset the Acados solver!");
    return CallbackReturn::FAILURE;
  }

  // the desired joint trajectory is queried from the commands topic
  // and passed to update via a rt pipe
  reference_subscriber_ = get_node()->create_subscription<CmdType>(
    "/cartesian_reference", rclcpp::SystemDefaultsQoS(),
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
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
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
      command_interfaces_, joint_names_, hardware_interface::HW_IF_EFFORT, ordered_interfaces) ||
    command_interfaces_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu acceleration command interfaces, got %zu",
      joint_names_.size(),
      ordered_interfaces.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Reset the NMPC solver
  if (0 != acados_solver_->reset()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to reset the Acados solver!");
    return CallbackReturn::ERROR;
  }

  // Reset command
  tau_cmd_.setZero();
  is_first_itr_ = true;


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
  // Robot parameters
  double l0 = 2.0;
  double l1 = 1.0;
  double l2 = 1.0;
  double m1 = 1.0;
  double m2 = 1.0;

  // get the data from the subscriber using the rt pipe
  auto reference_cartesian_pose = rt_command_ptr_.readFromRT();

  // retrieve current robot state
  q_pos_(0) = state_interfaces_[0].get_value();
  q_vel_(0) = state_interfaces_[1].get_value();
  q_pos_(1) = state_interfaces_[2].get_value();
  q_vel_(1) = state_interfaces_[3].get_value();

  if (is_first_itr_) {
    // Initialize reference position and velocity
    // Note: this is just an example, the actual values should be set based on the application
    // requirements. Also, in practice, use the URDF to retrieve a forward kinematics model...
    p_ref_(0) = l1 * cos(q_pos_(0)) + l2 * cos(q_pos_(0) + q_pos_(1));
    p_ref_(1) = l1 * sin(q_pos_(0)) + l2 * sin(q_pos_(0) + q_pos_(1));
    p_dot_ref_.setZero();
  }

  // retrieve reference position and velocity
  if (!reference_cartesian_pose || !(*reference_cartesian_pose)) {
    // no command received yet, use last position values and set velocitty ref to zero
    p_dot_ref_.setZero();
  } else {
    // checking proxy data validity
    if ((*reference_cartesian_pose)->data.size() != 4) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(), 1000,
        "command size does not match number of interfaces (should be [q1, q2, q_dot1, q_dot2])");
      return controller_interface::return_type::ERROR;
    }
    p_ref_(0) = (*reference_cartesian_pose)->data[0];
    p_ref_(1) = (*reference_cartesian_pose)->data[1];
    p_dot_ref_(0) = (*reference_cartesian_pose)->data[2];
    p_dot_ref_(1) = (*reference_cartesian_pose)->data[3];
  }
  // RCLCPP_INFO(get_node()->get_logger(), "p_ref =[ %f, %f]", p_ref_(0), p_ref_(1));

  // Set nmpc initial state
  acados::ValueMap x_values_map;
  x_values_map["q"] = std::vector(&q_pos_[0], q_pos_.data() + q_pos_.size());
  x_values_map["q_dot"] = std::vector(&q_vel_[0], q_vel_.data() + q_vel_.size());

  bool all_ok = true;
  if (is_first_itr_) {  // this is the first iteration
    // Set initial state values for all stages of the NMPC problem
    all_ok &= (0 == acados_solver_->initialize_state_values(x_values_map));
    // Update the first iteration flag
    is_first_itr_ = false;
  }
  // Set initial state values for the first stage of the NMPC problem
  all_ok &= (0 == acados_solver_->set_initial_state_values(x_values_map));

  if (!all_ok) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to set NMPC initial state values!");
    return controller_interface::return_type::ERROR;
  }

  // Set nmpc runtime parameters
  std::vector<double> Q_pos_diag = get_node()->get_parameter("nmpc.Q_pos_diag").as_double_array();
  std::vector<double> Q_vel_diag = get_node()->get_parameter("nmpc.Q_vel_diag").as_double_array();
  std::vector<double> R_diag = get_node()->get_parameter("nmpc.R_diag").as_double_array();

  acados::ValueMap p_values_map;
  p_values_map["l0"] = std::vector{l0};
  p_values_map["l1"] = std::vector{l1};
  p_values_map["l2"] = std::vector{l2};
  p_values_map["m1"] = std::vector{m1};
  p_values_map["m2"] = std::vector{m2};
  p_values_map["p_ref"] = std::vector(&p_ref_[0], p_ref_.data() + p_ref_.size());
  p_values_map["p_dot_ref"] = std::vector(&p_dot_ref_[0], p_dot_ref_.data() + p_dot_ref_.size());
  p_values_map["Q_pos_diag"] = Q_pos_diag;
  p_values_map["Q_vel_diag"] = Q_vel_diag;
  p_values_map["R_diag"] = R_diag;

  if (0 != acados_solver_->set_runtime_parameters(p_values_map)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to set NMPC runtime parameters!");
    return controller_interface::return_type::ERROR;
  }

  // Solve NMPC optimization problem
  if (0 != acados_solver_->solve()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to solve the NMPC SQP problem!");
    all_ok = false;
  } else {
    // Get optimal control input
    acados::ValueMap u_values_map = acados_solver_->get_control_values_as_map(0);
    tau_cmd_(0) = u_values_map["tau"][0];
    tau_cmd_(1) = u_values_map["tau"][1];
  }

  // Retrieve algebraic state values if needed
  // acados::ValueMap z_values_map = acados_solver_->get_algebraic_state_values_as_map(0);
  // std::cout << "p: " << z_values_map["p"][0] << ", " << z_values_map["p"][1] << std::endl  << std::endl;

  // Send command to robot
  if (!all_ok) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error during update()! Setting controls to zero.");
    tau_cmd_.setZero();
    command_interfaces_[0].set_value(0.0);
    command_interfaces_[1].set_value(0.0);
    return controller_interface::return_type::ERROR;
  }

  command_interfaces_[0].set_value(tau_cmd_(0));
  command_interfaces_[1].set_value(tau_cmd_(1));
  return controller_interface::return_type::OK;
}

}  // namespace example_acados_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  example_acados_controller::ExampleAcadosController, controller_interface::ControllerInterface)
