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

#include "diffdrive_ddsm115/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_ddsm115
{
hardware_interface::CallbackReturn DiffDriveDDSM115Hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.left_wheel_id = std::stoi(info_.hardware_parameters["left_wheel_id"]);
  cfg_.right_wheel_id = std::stoi(info_.hardware_parameters["right_wheel_id"]);

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.left_wheel_id);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.right_wheel_id);

//    imu_.setup("imu", 1);
//    cfg_.imu_device = info_.hardware_parameters["imu_device"];
//    cfg_.imu_baud_rate = std::stoi(info_.hardware_parameters["imu_baud_rate"]);


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveDDSM115Hardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveDDSM115Hardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveDDSM115Hardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveDDSM115Hardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveDDSM115Hardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // ..
  // check Imu state interfaces
  // if (info_.sensors[0].state_interfaces.size() != 10)
  // {
  //   RCLCPP_FATAL(
  //     rclcpp::get_logger("UnitreeHardware"),
  //     "Sensor[0] (should be IMU) has %zu state interfaces. 10 expected.", info_.sensors[0].state_interfaces.size());
  //   return hardware_interface::CallbackReturn::ERROR;
  // }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveDDSM115Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos_rads));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos_rads));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

//    // IMU State Interface
//    state_interfaces.emplace_back(hardware_interface::StateInterface(
//      info_.sensors[0].name, info_.sensors[0].state_interfaces[0].name, &imu_.orientation_x));
//    state_interfaces.emplace_back(hardware_interface::StateInterface(
//      info_.sensors[0].name, info_.sensors[0].state_interfaces[1].name, &imu_.orientation_y));
//    state_interfaces.emplace_back(hardware_interface::StateInterface(
//      info_.sensors[0].name, info_.sensors[0].state_interfaces[2].name, &imu_.orientation_z));
//    state_interfaces.emplace_back(hardware_interface::StateInterface(
//      info_.sensors[0].name, info_.sensors[0].state_interfaces[3].name, &imu_.orientation_w));
//    //
//    state_interfaces.emplace_back(hardware_interface::StateInterface(
//      info_.sensors[0].name, info_.sensors[0].state_interfaces[4].name, &imu_.angular_velocity_x));
//    state_interfaces.emplace_back(hardware_interface::StateInterface(
//      info_.sensors[0].name, info_.sensors[0].state_interfaces[5].name, &imu_.angular_velocity_y));
//    state_interfaces.emplace_back(hardware_interface::StateInterface(
//      info_.sensors[0].name, info_.sensors[0].state_interfaces[6].name, &imu_.angular_velocity_z));
//    //
//    state_interfaces.emplace_back(hardware_interface::StateInterface(
//      info_.sensors[0].name, info_.sensors[0].state_interfaces[7].name, &imu_.linear_acceleration_x));
//    state_interfaces.emplace_back(hardware_interface::StateInterface(
//      info_.sensors[0].name, info_.sensors[0].state_interfaces[8].name, &imu_.linear_acceleration_y));
//    state_interfaces.emplace_back(hardware_interface::StateInterface(
//      info_.sensors[0].name, info_.sensors[0].state_interfaces[9].name, &imu_.linear_acceleration_z));
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveDDSM115Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveDDSM115Hardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "Configuring ...please wait...");
  if (commsDDSM_.connected())
  {
    commsDDSM_.disconnect();
  }
  commsDDSM_.connect(cfg_.device, cfg_.timeout_ms);

//  if (mcuComms_.connected())
//  {
//    mcuComms_.disconnect();
//  }
//  mcuComms_.connect(cfg_.imu_device, cfg_.imu_baud_rate, cfg_.timeout_ms);

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveDDSM115Hardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "Cleaning up ...please wait...");
  if (commsDDSM_.connected())
  {
    commsDDSM_.disconnect();
  }

//  if (mcuComms_.connected())
//  {
//    mcuComms_.disconnect();
//  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DiffDriveDDSM115Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "Activating ...please wait...");
  if (!commsDDSM_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  commsDDSM_.set_ddsm115_mode(wheel_l_.id, VELOCITY_LOOP);
  commsDDSM_.get_ddsm115_mode(wheel_l_.id);
  wheel_l_.pos = commsDDSM_.responseData.angle;

  commsDDSM_.set_ddsm115_mode(wheel_r_.id, VELOCITY_LOOP);
  commsDDSM_.get_ddsm115_mode(wheel_r_.id);
  wheel_r_.pos = commsDDSM_.responseData.angle;

  //commsDDSM_.get_ddsm115_mode(1);
  //commsDDSM_.set_ddsm115_velocity(1, 2, 3);
  //commsDDSM_.set_ddsm115_brakes(1);
  
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveDDSM115Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "Deactivating ...please wait...");
  
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveDDSM115Hardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!commsDDSM_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  commsDDSM_.get_ddsm115_mode(wheel_l_.id);
  double wheel_pos = commsDDSM_.responseData.angle;
  double wheel_vel = commsDDSM_.responseData.velocity;
  

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "L Position Now (angle) is: %f", wheel_pos);
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "L Position Now (rads) is: %f", wheel_l_.degrees_to_radians(wheel_pos) );
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "L Accumulated (angle) is: %f", wheel_l_.calculate_accumulated_position(wheel_pos) );

  
  wheel_l_.pos_rads = wheel_l_.degrees_to_radians(wheel_l_.calculate_accumulated_position(wheel_pos));
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "L Accumulated (rads) is: %f", wheel_l_.pos_rads );
  wheel_l_.vel = wheel_l_.rpm_to_rad_per_sec(wheel_vel);
  

  
  commsDDSM_.get_ddsm115_mode(wheel_r_.id);
  wheel_pos = commsDDSM_.responseData.angle;
  wheel_vel = commsDDSM_.responseData.velocity;

  wheel_r_.pos_rads = wheel_r_.degrees_to_radians(-wheel_r_.calculate_accumulated_position(wheel_pos));
  wheel_r_.vel = wheel_r_.rpm_to_rad_per_sec(wheel_vel);
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "R Position is: %f", wheel_r_.pos);
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "R Velocity is: %f", wheel_r_.vel);


//  if (!mcuComms_.connected())
//  {
//    return hardware_interface::return_type::ERROR;
//  }
//  std::string imu_response = mcuComms_.get_imu_all_data().c_str();
//  if (!imu_.string_to_values(imu_response))
//  {
//    return hardware_interface::return_type::ERROR;
//  }
  // RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "IMU data - orientation_y: %f", imu_.orientation_y );
  // RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "IMU data: %s", mcuComms_.get_imu_all_data().c_str() );
  // mcuComms_.get_imu_all_data();


  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_ddsm115 ::DiffDriveDDSM115Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!commsDDSM_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "Write L ID:%d cmd:%f", wheel_l_.id, wheel_l_.cmd);
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "Write R ID:%d cmd:%f", wheel_r_.id, wheel_r_.cmd);
  commsDDSM_.set_ddsm115_velocity(wheel_l_.id, wheel_l_.cmd*10, 3);
  commsDDSM_.set_ddsm115_velocity(wheel_r_.id, -wheel_r_.cmd*10, 3);

  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_ddsm115

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_ddsm115::DiffDriveDDSM115Hardware, hardware_interface::SystemInterface)
