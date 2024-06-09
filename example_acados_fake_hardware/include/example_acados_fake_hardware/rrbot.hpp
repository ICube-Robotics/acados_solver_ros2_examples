// Copyright 2023, ICube Laboratory, University of Strasbourg
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
// Author: Maciej Bednarczyk (macbednarczyk@gmail.com)

#ifndef EXAMPLE_ACADOS_FAKE_HARDWARE__RRBOT_HPP_
#define EXAMPLE_ACADOS_FAKE_HARDWARE__RRBOT_HPP_

#include <Eigen/Dense>
#include <Eigen/QR>

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"
#include "example_acados_fake_hardware/visibility_control.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace example_acados_fake_hardware
{
class RRbot : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RRbot);

  EXAMPLE_ACADOS_FAKE_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  EXAMPLE_ACADOS_FAKE_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  EXAMPLE_ACADOS_FAKE_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  EXAMPLE_ACADOS_FAKE_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  EXAMPLE_ACADOS_FAKE_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_commands_effort_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_acceleration_;
  std::vector<double> hw_states_effort_;

  Eigen::Vector2d q_pos_;
  Eigen::Vector2d q_vel_;
  Eigen::Vector2d q_acc_;
  Eigen::Vector2d tau_;

  Eigen::Matrix2d bmat_;
  Eigen::Vector2d cvec_;
  Eigen::Vector2d gvec_;

  double m1_ = 1.0;
  double l1_ = 1.0;
  double lc1_ = l1_ / 2;
  double i1_ = m1_ / 12 * l1_ * l1_;
  double m2_ = 1.0;
  double l2_ = 1.0;
  double lc2_ = l2_ / 2;
  double i2_ = m2_ / 12 * l2_ * l2_;
  double g_ = 9.8;
};

}  // namespace example_acados_fake_hardware

#endif  // EXAMPLE_ACADOS_FAKE_HARDWARE__RRBOT_HPP_
