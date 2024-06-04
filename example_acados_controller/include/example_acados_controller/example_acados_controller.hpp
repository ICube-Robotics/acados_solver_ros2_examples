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

#ifndef EXAMPLE_CONTROLLER__EXAMPLE_CONTROLLER_HPP_
#define EXAMPLE_CONTROLLER__EXAMPLE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/QR>

#include "controller_interface/controller_interface.hpp"
#include "example_acados_controller/visibility_control.h"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace example_acados_controller
{
using CmdType = trajectory_msgs::msg::JointTrajectory;
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
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  EXAMPLE_ACADOS_CONTROLLER_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

  EXAMPLE_ACADOS_CONTROLLER_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

  EXAMPLE_ACADOS_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:
  std::vector<std::string> joint_names_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;

  std::string logger_name_;

  Eigen::Vector2d q_pos_ref_;
  Eigen::Vector2d q_vel_ref_;
};

}  // namespace example_acados_controller

#endif  // EXAMPLE_ACADOS_CONTROLLER