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

#include "hardware/drive.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace hardware_interface;
using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

namespace hardware_int {
/*
Driver left_back{"can0", 99};
Driver left_middle{"can0", 99};
Driver left_front{"can0", 99};

Driver right_back{"can0", 99};
Driver right_middle{"can0", 99};
Driver right_front{"can0", 99};

vector<Driver*> motors = {&left_back, &left_middle, &left_front, &right_back, &right_middle, &right_front};*/

CallbackReturn DiffBotSystemHardware::on_init(const HardwareInfo & info) {
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  
  hw_positions_.resize(info_.joints.size(), numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), numeric_limits<double>::quiet_NaN());

  for (const ComponentInfo & joint : info_.joints) {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

vector<StateInterface> DiffBotSystemHardware::export_state_interfaces() {
  vector<StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(StateInterface(info_.joints[i].name, HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(StateInterface(info_.joints[i].name, HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

vector<CommandInterface> DiffBotSystemHardware::export_command_interfaces() {
  vector<CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(CommandInterface(
      info_.joints[i].name, HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn DiffBotSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffBotSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  for (auto i = 0u; i < hw_positions_.size(); i++) {
    if (isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

return_type DiffBotSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) {
  /*for(size_t i = 0; i < hw_positions_.size(); i++) {
    hw_positions_[i] = convertShaftAngleToRotations(motors[i]->getMotorStatus2().shaft_angle);

    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Got position state of %.5f for %s!", hw_positions_[i], info_.joints[i].name.c_str());
  }*/
  return return_type::OK;
}

return_type hardware_int::DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Writing...");
  /*for(auto i = 0u; i<hw_commands_.size(); i++) {
      RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
      info_.joints[i].name.c_str());
      motors[i]->sendVelocitySetpoint(convertShaftAngleToRotations(hw_commands_[i]));
  }*/
  return return_type::OK;
}

}
/*
float convertShaftAngleToRotations(float shaftAngle)  {
    return shaftAngle * (1.0/360.0) * (36.0/1.0) * ((2.0*3.14159*.2667)/1.0);
  }*/
PLUGINLIB_EXPORT_CLASS(
  hardware_int::DiffBotSystemHardware, SystemInterface)