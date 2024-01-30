#include "urc_hw/hardware_interfaces/rover_drivetrain.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <iterator>
#include <ostream>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#include <string>

PLUGINLIB_EXPORT_CLASS(
    urc_hardware::hardware_interfaces::RoverDrivetrain,
    hardware_interface::SystemInterface);

namespace urc_hardware::hardware_interfaces
{

  RoverDrivetrain::RoverDrivetrain() : hardware_interface_name("Rover Drivetrain"), velocity_rps_commands(2, 0), velocity_rps_states(2, 0), velocity_rps_breakdown(6, 0) {}
  RoverDrivetrain::~RoverDrivetrain() = default;

  hardware_interface::CallbackReturn RoverDrivetrain::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.hardware_parameters.find("udp_address") == info_.hardware_parameters.end())
    {
      RCLCPP_ERROR(
          rclcpp::get_logger(hardware_interface_name),
          "Error during initialization: 'udp_address' configuration not found. Expect to enter the udp server address.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (info_.hardware_parameters.find("udp_port") == info_.hardware_parameters.end())
    {
      RCLCPP_ERROR(
          rclcpp::get_logger(hardware_interface_name),
          "Error during initialization: 'udp_port' configuration not "
          "found. Expect to enter the port number.");
      return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RoverDrivetrain::on_configure(const rclcpp_lifecycle::State &)
  {
    std::fill(velocity_rps_commands.begin(), velocity_rps_commands.end(), 0.0);
    std::fill(velocity_rps_states.begin(), velocity_rps_states.end(), 0.0);
    std::fill(velocity_rps_breakdown.begin(), velocity_rps_breakdown.end(), 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::CommandInterface> RoverDrivetrain::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back("left_side", "velocity", &this->velocity_rps_commands[0]);
    command_interfaces.emplace_back("right_side", "velocity", &this->velocity_rps_commands[1]);
    return command_interfaces;
  }

  std::vector<hardware_interface::StateInterface> RoverDrivetrain::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back("left_side", "velocity", &this->velocity_rps_states[0]);
    state_interfaces.emplace_back("right_side", "velocity", &this->velocity_rps_states[1]);
    state_interfaces.emplace_back("left_side", "velocity.front", &this->velocity_rps_breakdown[0]);
    state_interfaces.emplace_back("left_side", "velocity.mid", &this->velocity_rps_breakdown[1]);
    state_interfaces.emplace_back("left_side", "velocity.back", &this->velocity_rps_breakdown[2]);
    state_interfaces.emplace_back("right_side", "velocity.front", &this->velocity_rps_breakdown[3]);
    state_interfaces.emplace_back("right_side", "velocity.mid", &this->velocity_rps_breakdown[4]);
    state_interfaces.emplace_back("right_side", "velocity.back", &this->velocity_rps_breakdown[5]);
    return state_interfaces;
  }

  hardware_interface::CallbackReturn RoverDrivetrain::on_activate(const rclcpp_lifecycle::State &)
  {

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RoverDrivetrain::on_deactivate(const rclcpp_lifecycle::State &)
  {

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /*hardware_interface::return_type RoverDrivetrain::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    return hardware_interface::return_type::OK;
  }*/

  hardware_interface::return_type RoverDrivetrain::write(const rclcpp::Time &time, const rclcpp::Duration &duration)
  {
    if (duration.seconds() < 0.001)
    {
      return hardware_interface::return_type::OK;
    }
    time.nanoseconds();
    return hardware_interface::return_type::OK;
  }

} // namespace urc_hardware::hardware_interfaces