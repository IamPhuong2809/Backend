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

#include "hardware_interface/lexical_casts.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "my_robot_hw/ammr_system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"

extern "C" {
    #include "melcli.h"
    #include "slmp.h"
}

namespace my_robot_hw
{

  static const char* POS_ADDRS[] = {
    "D1000","D1003","D1006","D1009","D1012",
    "D1015","D1018","D1021","D32336","D32384", NULL
  };
  static const char* VEL_ADDRS[] = {
    "D1030","D1032","D1034","D1036","D1038",
    "D1040","D1042","D1046","D1048","D1050", NULL
  };

  const char* CMD_ADDRS[] = {
      "D1100","D1102","D1104","D1106","D1108",
      "D1110","D1112","D1114","D1116","D1118", NULL
  };

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

    transmission_ratio_ = {150.0, 100.0, 112.0, 7.0, 100.0, 9.0, 1.0, 1.0, 43.75, 43.75};

    if (transmission_ratio_.size() != num_joints) {
      RCLCPP_FATAL(rclcpp::get_logger("AMMRSystemHardware"),
                  "Expected 10 transmission ratios, got %zu", transmission_ratio_.size());
      return CallbackReturn::ERROR;
    }

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

    RCLCPP_INFO(rclcpp::get_logger("AMMRSystemHardware"), "Hardware initialized");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> AMMRSystemHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(info_.joints[i].name, "position", &hw_positions_[i]);
      state_interfaces.emplace_back(info_.joints[i].name, "velocity", &hw_velocities_[i]);
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

    hardware_interface::CallbackReturn AMMRSystemHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    for (size_t i = 0; i < hw_positions_.size(); ++i)
    {
      hw_positions_[i] = 0.0;
      hw_velocities_[i] = 0.0;
      hw_commands_[i] = 0.0;
    }

    RCLCPP_INFO(rclcpp::get_logger("AMMRSystemHardware"), "System configured");
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn AMMRSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("AMMRSystemHardware"), "Activating...");

    if (initMelcli() != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("AMMRSystemHardware"), "Failed to connect to PLC during activation");
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("AMMRSystemHardware"), "System connected to PLC and activated");
    is_active_ = true;

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn AMMRSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("AMMRSystemHardware"), "Deactivating...");

    if (g_ctx) {
      melcli_disconnect(g_ctx);
      melcli_free_context(g_ctx);
      g_ctx = nullptr;
      RCLCPP_INFO(rclcpp::get_logger("AMMRSystemHardware"), "Disconnected from PLC");
    }

    is_active_ = false;
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type AMMRSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    if (!is_active_) {
      return hardware_interface::return_type::OK;
    }
    static thread_local bool slmp_ok = false;
    if (!slmp_ok) {
        slmp_ok = (slmp_init() == 0);
    }
    uint32_t *vel_data = NULL;
    uint32_t *pos_data = NULL;

    if (melcli_random_read_dword(g_ctx, NULL, POS_ADDRS, &pos_data, NULL) != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("AMMRSystemHardware"), "Position read failed: len=%d", pos_len);
      melcli_disconnect(g_ctx);
      melcli_free_context(g_ctx);
      g_ctx = nullptr;
      if (initMelcli() != 0) {
        return hardware_interface::return_type::ERROR;
      }
      return hardware_interface::return_type::ERROR;
    }

    // Read velocities
    if (melcli_random_read_dword(g_ctx, NULL, VEL_ADDRS, &vel_data, NULL) != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("AMMRSystemHardware"), "Velocity read failed: len=%d", vel_len);
      melcli_free(pos_data);
      melcli_disconnect(g_ctx);
      melcli_free_context(g_ctx);
      g_ctx = nullptr;
      if (initMelcli() != 0) {
        return hardware_interface::return_type::ERROR;
      }
      return hardware_interface::return_type::ERROR;
    }

    for (size_t i = 0; i < hw_positions_.size(); ++i) {
      uint32_t raw = pos_data[i];
      hw_positions_[i] = (i < 8)
        ? raw * M_PI / 18000000.0
        : (int32_t)raw / (10000000.0 * a);
    }
    for (size_t i = 0; i < hw_velocities_.size(); ++i) {
      uint32_t raw = vel_data[i];
      hw_velocities_[i] = raw * M_PI * a / (3000.0 * transmission_ratio_[i]);
    }

    free(pos_data);
    free(vel_data);
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type AMMRSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    // if (melcli_connect(g_ctx) != 0) {
    //   RCLCPP_ERROR(rclcpp::get_logger("AMMRSystemHardware"), "MELCLI context is null, attempting reconnect...");
    //   if (initMelcli() != 0) {
    //     RCLCPP_ERROR(rclcpp::get_logger("AMMRSystemHardware"), "Reconnection failed");
    //     return hardware_interface::return_type::ERROR;
    //   }
    // }

    if (!is_active_) {
      return hardware_interface::return_type::OK;
    }
    static thread_local bool slmp_ok = false;
    if (!slmp_ok) {
        slmp_ok = (slmp_init() == 0);
    }
    // for (size_t i = 0; i < hw_commands_.size(); ++i)
    // {
    //    RCLCPP_INFO(rclcpp::get_logger("AMMRSystemHardware"),"  - [%zu] = %.6f\n", i, hw_commands_[i]);
    // }
    return hardware_interface::return_type::OK;
  }

  int AMMRSystemHardware::initMelcli()
  {
    char target_ip[64] = "192.168.5.10";
    int target_port = 5010;
    const melcli_station_t target_station = MELCLI_CONNECTED_STATION;
    const melcli_timeout_t timeout = MELCLI_TIMEOUT_DEFAULT;

    srand((unsigned int)time(NULL));         
        
    g_ctx = melcli_new_context(MELCLI_TYPE_TCPIP, target_ip, target_port, "0.0.0.0", 0, &target_station, &timeout);

    if (g_ctx == NULL) {
        RCLCPP_ERROR(rclcpp::get_logger("AMMRSystemHardware"), "❌ Failed to create context.");
        return -1;
    }

    // melcli_set_debug(g_ctx, 1);

    // ✅ Kết nối PLC
    if (melcli_connect(g_ctx) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("AMMRSystemHardware"), "❌ Failed to connect to PLC.");
        return -1;
    }

    RCLCPP_INFO(rclcpp::get_logger("AMMRSystemHardware"), "Connected to PLC at %s:%d", target_ip, target_port);

    return 0;
  }

} // namespace my_robot_hw
