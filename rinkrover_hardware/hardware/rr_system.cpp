#include "rinkrover_hardware/rr_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rinkrover_hardware
{
hardware_interface::CallbackReturn RRSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Retrieve all the xacro file parameters from HardwareInfo
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Set interface configuration
  cfg_.steering_joint_name = info_.hardware_parameters["steering_joint_name"];
  cfg_.traction_joint_name = info_.hardware_parameters["traction_joint_name"];
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.track_width = std::stof(info_.hardware_parameters["track_width"]);
  cfg_.wheelbase = std::stof(info_.hardware_parameters["wheelbase"]);
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  wheel_l_.setup("left_wheel", cfg_.enc_counts_per_rev);
  wheel_r_.setup("right_wheel", cfg_.enc_counts_per_rev);

  // Create logger and clock
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.RinkRover"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // Check if the number of joints is correct based on the mode of operation (expecting one steering and traction joint from tricycle)
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      get_logger(),
      "RRSystemHardware::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 2.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = joint.name.find("steering") != std::string::npos;

    
    if (joint_is_steering)
    {
      /* STEERING JOINTS have a position command interface and a position state interface */
      RCLCPP_INFO(get_logger(), "Joint '%s' is a steering joint.", joint.name.c_str());

      if (joint.command_interfaces.size() != 1)
      {
        // Check NUMBER of command interfaces for steering joint is 1
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        // Check steering joint command interface is a POSITION interface
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 1)
      {
        // Check NUMBER of state interfaces for steering joint is 1
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        // Check steering joint state interface is POSITION interface
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Joint '%s' is a drive joint.", joint.name.c_str());

      /* Drive joints have a velocity command interface and a velocity state interface */
      if (joint.command_interfaces.size() != 1)
      {
        // Check NUMBER of command interfaces for traction joint is 1
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        // Check traction joint command interface is VELOCITY INTERFACE
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        // Check NUMBER of state interfaces for traction joint is 2
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        // Check traction joint state interface [0] is VELOCITY INTERFACE
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
      {
        // Check traction joint state interface [1] is POSITION INTERFACE
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  hw_interfaces_["steering"] = Joint("steering_joint");

  hw_interfaces_["traction"] = Joint("virtual_traction_joint");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RRSystemHardware::export_state_interfaces()
{
  // Expose state interface variables to the ros2control
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.second.joint_name, hardware_interface::HW_IF_POSITION, &joint.second.state.position));

    if (joint.first == "traction")
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.second.joint_name, hardware_interface::HW_IF_VELOCITY, &joint.second.state.velocity));
    }
  }

  RCLCPP_INFO(get_logger(), "Exported %zu state interfaces.", state_interfaces.size());

  for (auto s : state_interfaces)
  {
    RCLCPP_INFO(get_logger(), "Exported state interface '%s'.", s.get_name().c_str());
  }

  RCLCPP_INFO(get_logger(), "Exporting states FINISHED");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    if (joint.first == "steering")
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.second.joint_name, hardware_interface::HW_IF_POSITION,
        &joint.second.command.position));
    }
    else if (joint.first == "traction")
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.second.joint_name, hardware_interface::HW_IF_VELOCITY,
        &joint.second.command.velocity));
    }
  }

  RCLCPP_INFO(get_logger(), "Exported %zu command interfaces.", command_interfaces.size());

  for (auto i = 0u; i < command_interfaces.size(); i++)
  {
    RCLCPP_INFO(
      get_logger(), "Exported command interface '%s'.", command_interfaces[i].get_name().c_str());
  }

  RCLCPP_INFO(get_logger(), "Exporting commands FINISHED");
  return command_interfaces;
}

hardware_interface::CallbackReturn RRSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

  if(!comms_.connected())
  {
    RCLCPP_FATAL(get_logger(), "Unable to connect to Serial Device");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & joint : hw_interfaces_)
  {
    joint.second.state.position = 0.0;

    if (joint.first == "traction")
    {
      joint.second.state.velocity = 0.0;
      joint.second.command.velocity = 0.0;
    }

    else if (joint.first == "steering")
    {
      joint.second.command.position = 0.0;
    }
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  comms_.disconnect();

  if(comms_.connected())
  {
    RCLCPP_FATAL(get_logger(), "Unable to disconnect to Serial Device");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // TODO: FOR NOW, pass through last command as current state for testing purposes

  //RCLCPP_INFO(get_logger(), "READING!");
  //comms_.read_encoder_values();

  // Assume servo motor is highly accurate, pass through feedback as last command
  hw_interfaces_["steering"].state.position = hw_interfaces_["steering"].command.position;

  hw_interfaces_["traction"].state.velocity = hw_interfaces_["traction"].command.velocity;
  hw_interfaces_["traction"].state.position +=
    hw_interfaces_["traction"].state.velocity * period.seconds();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type rinkrover_hardware ::RRSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  //RCLCPP_INFO(get_logger(), "WRITING!");

  // comms.set_motor_values();

  return hardware_interface::return_type::OK;
}

}  // namespace rinkrover_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  rinkrover_hardware::RRSystemHardware, hardware_interface::SystemInterface)
