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
//
// Author: Thibault Poignonec (tpoignonec@unistra.fr)

#ifndef EXAMPLE_ACADOS_CONTROLLER__EXAMPLE_ACADOS_CONTROLLER_HPP_
#define EXAMPLE_ACADOS_CONTROLLER__EXAMPLE_ACADOS_CONTROLLER_HPP_

#include <Eigen/Dense>
#include <Eigen/QR>

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "example_acados_controller/visibility_control.h"

// Acados
#include "pluginlib/class_loader.hpp"
#include "acados_solver_base/acados_solver.hpp"

namespace example_acados_controller
{
using CmdType = std_msgs::msg::Float64MultiArray;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ExampleAcadosController : public controller_interface::ControllerInterface
{
public:
  EXAMPLE_ACADOS_CONTROLLER_PUBLIC
  ExampleAcadosController();

  EXAMPLE_ACADOS_CONTROLLER_PUBLIC
  CallbackReturn on_init() override;

  EXAMPLE_ACADOS_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  EXAMPLE_ACADOS_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  EXAMPLE_ACADOS_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  EXAMPLE_ACADOS_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  EXAMPLE_ACADOS_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  EXAMPLE_ACADOS_CONTROLLER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  EXAMPLE_ACADOS_CONTROLLER_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  EXAMPLE_ACADOS_CONTROLLER_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  EXAMPLE_ACADOS_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:
  // Basic controller setup
  std::vector<std::string> joint_names_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr reference_subscriber_;
  std::string logger_name_;

  /// Acados solver pluginlib loader
  std::shared_ptr<pluginlib::ClassLoader<acados::AcadosSolver>> acados_solver_loader_;

  /// Acados solver
  std::unique_ptr<acados::AcadosSolver> acados_solver_;
  bool is_first_itr_;

  // Joint state
  Eigen::Vector2d q_pos_;
  Eigen::Vector2d q_vel_;

  // Cartesian reference
  Eigen::Vector2d p_ref_;
  Eigen::Vector2d p_dot_ref_;

  // Controls
  Eigen::Vector2d tau_cmd_;
};

}  // namespace example_acados_controller

#endif  // EXAMPLE_ACADOS_CONTROLLER__EXAMPLE_ACADOS_CONTROLLER_HPP_
