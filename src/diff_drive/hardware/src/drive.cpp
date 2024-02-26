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

#include "diff_drive/drive.hpp"

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

namespace diff_drive
{
  TalonSRX left_back(1);
  float wheel_radius = 0.5;
  /*TalonSRX left_middle(1);
  TalonSRX left_front(2);*/

  TalonSRX right_back(2);
  /*TalonSRX right_middle(4);
  TalonSRX right_front(5);*/
  int kTimeoutMs = 100;
  
  std::vector<TalonSRX *> motors = {&left_back,&right_back,};

  CallbackReturn DiffBotSystemHardware::on_init(const HardwareInfo &info)
  {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    hw_positions_.resize(info_.joints.size(), numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), numeric_limits<double>::quiet_NaN());
    last_hw_commands_.resize(info_.joints.size(), numeric_limits<double>::quiet_NaN());

    for (const ComponentInfo &joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), HW_IF_VELOCITY);
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), HW_IF_POSITION);
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), HW_IF_VELOCITY);
        return CallbackReturn::ERROR;
      }
    }
    /* Motor controller initialization */
    
    for (auto i = 0u; i < motors.size(); i++)
    {
      motors[i]->ConfigFactoryDefault();
      motors[i]->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, kTimeoutMs);
      if(i<1) { //change this when more motors are added
        motors[i]->SetSensorPhase(true);
      }
      else {
        motors[i]->SetSensorPhase(false);
      }
      
    }
    return CallbackReturn::SUCCESS;
  }

  vector<StateInterface> DiffBotSystemHardware::export_state_interfaces()
  {
    vector<StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(StateInterface(info_.joints[i].name, HW_IF_POSITION, &hw_positions_[i]));
      state_interfaces.emplace_back(StateInterface(info_.joints[i].name, HW_IF_VELOCITY, &hw_velocities_[i]));
    }

    return state_interfaces;
  }

  vector<CommandInterface> DiffBotSystemHardware::export_command_interfaces()
  {
    vector<CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(CommandInterface(
          info_.joints[i].name, HW_IF_VELOCITY, &hw_commands_[i]));
    }

    return command_interfaces;
  }

  CallbackReturn DiffBotSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
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

  CallbackReturn DiffBotSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    for (auto i = 0u; i < hw_positions_.size(); i++)
    {
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

  return_type DiffBotSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    for (size_t i = 0; i < hw_positions_.size(); i++)
    {
      /*hw_positions_[i] = convertTalonSRXUnitsToMeters(motors[i]->GetSelectedSensorPosition());

      RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Got position state of %.5f for %s!", hw_positions_[i], info_.joints[i].name.c_str());*/
    }
    return return_type::OK;
  }

return_type DiffBotSystemHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  //RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Writing...");
  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    if (hw_commands_[i] != last_hw_commands_[i]) // Compare with the previous command
    {
      RCLCPP_INFO(
          rclcpp::get_logger("DiffBotSystemHardware"), "Got command %.5f for '%s'! We tell it to do %f\n", hw_commands_[i]*wheel_radius,
          info_.joints[i].name.c_str(), convertTalonSRXUnitsToMeters(motors[i]->GetSelectedSensorVelocity()));
    }
    motors[i]->Set(ControlMode::Velocity, clamp(convertMPStoTalonSRXUnits(hw_commands_[i]*wheel_radius),-0.1,0.1)); //theoretical max speed...this should not work
    unmanaged::Unmanaged::FeedEnable(100); //in non FRC applications this is needed!
    last_hw_commands_[i] = hw_commands_[i]; // Update the last command
  }
  return return_type::OK;
}

  // m/s -> s/ms -> to 100ms -> rotation / m -> units / rotation = units / 100ms
// god i hate ctre who picks units of units/100ms. genuinely insane.
double DiffBotSystemHardware::convertMPStoTalonSRXUnits(float mps)
{
  return mps * (1.0 / 1000.0) * (10.0 / 1.0) * (1.0 / 2*3.14159 * wheel_radius) * ((1 + (46.0 / 11.0)) * (1 + (46.0 / 11.0)) * (1 + (46.0 / 11.0)) * 28.0);
}

// this is probably wrong?
//  units/rot / ms -> ms/s -> m/rot->
double DiffBotSystemHardware::convertTalonSRXUnitsToMeters(float nativeSensorUnits)
{
  return nativeSensorUnits * (1000.0 /1) * (1.0 / ((1 + (46.0 / 11.0)) * (1 + (46.0 / 11.0)) * (1 + (46.0 / 11.0)) * 28.0)) * (2*3.14159 * wheel_radius);
  //return nativeSensorUnits * (1.0 / ((1 + (46.0 / 11.0)) * (1 + (46.0 / 11.0)) * (1 + (46.0 / 11.0)) * 28.0)) * (3.14159 * .2667) / 1.0;
}

}



PLUGINLIB_EXPORT_CLASS(
  diff_drive::DiffBotSystemHardware, hardware_interface::SystemInterface)
