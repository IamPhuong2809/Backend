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

#include "my_robot_hw/ammr_system.hpp"
#include "melcli/melcli.h"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_robot_hw
{

hardware_interface::CallbackReturn AMMRSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  size_t num_joints = info.joints.size();
  hw_commands_.resize(num_joints, 0.0);
  hw_positions_.resize(num_joints, 0.0);
  hw_velocities_.resize(num_joints, 0.0);

  for (size_t i = 0; i < num_joints; ++i)
  {
    if (info.joints[i].state_interfaces.size() != 2 ||
        (info.joints[i].command_interfaces.size() != 1))
    {
      RCLCPP_FATAL(rclcpp::get_logger("AMMRSystemHardware"),
                   "Joint %s must have 2 state interfaces and 1 command interface", info.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AMMRSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AMMRSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(info_.joints[i].name, info_.joints[i].command_interfaces[0].name, &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type AMMRSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // For mock: mirror command to position
  for (size_t i = 0; i < hw_positions_.size(); ++i)
  {
    hw_positions_[i] = hw_commands_[i];
    hw_velocities_[i] = 0.0;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AMMRSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Here you would normally send the command to your actuators
  return hardware_interface::return_type::OK;
}

} // namespace my_robot_hw
