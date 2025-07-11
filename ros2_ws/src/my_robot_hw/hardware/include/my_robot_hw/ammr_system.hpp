// Copyright 2021 ros2_control Development Team
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

#ifndef MY_ROBOT_HW__AMMR_SYSTEM_HPP_
#define MY_ROBOT_HW__AMMR_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "melcli.h"

namespace my_robot_hw
{

class AMMRSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AMMRSystemHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  int initMelcli();

  std::vector<double> hw_commands_pos_;
  std::vector<double> hw_commands_vel_;
  std::vector<double> hw_commands_last_vel_;
  std::vector<double> hw_commands_acc_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> transmission_ratio_;

  melcli_ctx_t *g_ctx = NULL;

  int is_active_ = false;
  int initialized = false;
  const double a = 0.161;
  const double d = 0.46;

  int pos_len = 10;
  int vel_len = 10;

};

} // namespace my_robot_hw

PLUGINLIB_EXPORT_CLASS(my_robot_hw::AMMRSystemHardware, hardware_interface::SystemInterface)
#endif
