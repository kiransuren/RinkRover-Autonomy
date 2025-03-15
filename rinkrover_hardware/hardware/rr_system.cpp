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

  // IMU orientation (quaternion)
  state_interfaces.emplace_back("imu_sensor", "orientation.w", &orientation_[0]);
  state_interfaces.emplace_back("imu_sensor", "orientation.x", &orientation_[1]);
  state_interfaces.emplace_back("imu_sensor", "orientation.y", &orientation_[2]);
  state_interfaces.emplace_back("imu_sensor", "orientation.z", &orientation_[3]);
  // IMU angular velocity
  state_interfaces.emplace_back("imu_sensor", "angular_velocity.x", &angular_velocity_[0]);
  state_interfaces.emplace_back("imu_sensor", "angular_velocity.y", &angular_velocity_[1]);
  state_interfaces.emplace_back("imu_sensor", "angular_velocity.z", &angular_velocity_[2]);
  // IMU linear acceleration
  state_interfaces.emplace_back("imu_sensor", "linear_acceleration.x", &linear_acceleration_[0]);
  state_interfaces.emplace_back("imu_sensor", "linear_acceleration.y", &linear_acceleration_[1]);
  state_interfaces.emplace_back("imu_sensor", "linear_acceleration.z", &linear_acceleration_[2]);

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

  // Try to open the i2C bus
  if (!bus_.openBus()) {
    std::cerr << "Failed to connect to I2C Bus" << std::endl;
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

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  //Clean this up and move to read from params
  double track_width =  0.748;
  double wheelbase = 1.06;
  double correction = 1.0;
    
  // Buffer to hold encoder feedback [cm/s] -> [m/s] with 2 degrees resolution
  int enc1, enc2;

  if(!bus_.readEncoders(enc1, enc2))
  {
    RCLCPP_FATAL(get_logger(), "Encoder Reading Failed!");
  }

  int16_t ax, ay, az;
  if (!bus_.readAccelerometer(ax, ay, az)) {
    RCLCPP_FATAL(get_logger(), "Accelerometer Reading Failed!");
  }

  int16_t gx, gy, gz;
  if (!bus_.readGyroscope(gx, gy, gz)) {
    RCLCPP_FATAL(get_logger(), "Gyroscope Reading Failed!");
  }

  int16_t qw, qx, qy, qz;
  if(!bus_.readQuatAngles(qw, qx, qy, qz)) {
    RCLCPP_FATAL(get_logger(), "Quaternion Orientation Reading Failed!");
  }

  // Set IMU states after reading
  orientation_[0] = qw; // w
  orientation_[1] = qx; // x
  orientation_[2] = qy; // y
  orientation_[3] = qz; // z

  angular_velocity_[0] = gx / 100.0;
  angular_velocity_[1] = gy / 100.0;
  angular_velocity_[2] = gz / 100.0;

  linear_acceleration_[0] = ax / 100.0;
  linear_acceleration_[1] = ay / 100.0;
  linear_acceleration_[2] = az / 100.0;

  // Calculate wheel linear velocities [m/s]
  double left_motor_lin_vel = double(enc1)/100.0;  // [m/s] with 2 degrees resolution
  double right_motor_lin_vel = double(enc2)/100.0; // [m/s] with 2 degrees resolution

  // Calculate wheel angular velocities [rad/s]
  double left_motor_ang_vel = left_motor_lin_vel / 0.127;
  double right_motor_ang_vel = right_motor_lin_vel / 0.127;

  // Undo ackermann compensation for wheel velocities [rad/s]
  double virt_left_motor = left_motor_ang_vel / (1 - wheelbase*tan(hw_interfaces_["steering"].command.position)*correction / 2*track_width);
  double virt_right_motor = right_motor_ang_vel / (1 + wheelbase*tan(hw_interfaces_["steering"].command.position)*correction / 2*track_width);
  // Take average to find virtual center wheel angular velocities [rad/s]
  double virtual_traction_motor_vel = (virt_left_motor + virt_right_motor) /2;

  // Set new joint states
  hw_interfaces_["steering"].state.position = hw_interfaces_["steering"].command.position;  // pass through steering position
  hw_interfaces_["traction"].state.velocity = virtual_traction_motor_vel * 2;  
  hw_interfaces_["traction"].state.position +=
    hw_interfaces_["traction"].state.velocity * period.seconds();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type rinkrover_hardware ::RRSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  //Clean this up and move to read from params
  double track_width =  0.748;
  double wheelbase = 1.06;

  double req_velocity = hw_interfaces_["traction"].command.velocity * 0.127; // convert from rad/s to m/s
  double req_steering_angle = hw_interfaces_["steering"].command.position; // radians
  double correction = 1.0;

  //Take virtual center wheel velocity command and transpose to two separate wheel velocity commands
  double left_motor_vel = req_velocity * (1 - wheelbase*tan(req_steering_angle)*correction / 2*track_width);
  double right_motor_vel = req_velocity * (1 + wheelbase*tan(req_steering_angle)*correction / 2*track_width);

  //Convert doubles to x100 integer (0.01 -> 1)
  int16_t left_motor_cmd = int16_t(round(left_motor_vel*100));                      // send down as cm/s
  int16_t right_motor_cmd = int16_t(round(right_motor_vel*100));                    // send down as cm/scorrection
  int16_t steering_motor_cmd = int16_t(round(req_steering_angle*100*(180/(22/7)))); //send down as DEGREES!

  // Don't send any new commands if it is identical to the last command
  if((last_left_motor_cmd == left_motor_cmd && last_right_motor_cmd == right_motor_cmd) && last_steering_motor_cmd == steering_motor_cmd)
  {
    return hardware_interface::return_type::OK;
  }

  if(!bus_.writeMotorCommand(left_motor_cmd, right_motor_cmd, steering_motor_cmd))
  {
    RCLCPP_ERROR(get_logger(), "Unable to set motor commands!");
    //TODO: add fail and return?
  }

  last_left_motor_cmd = left_motor_cmd;
  last_right_motor_cmd = right_motor_cmd;
  last_steering_motor_cmd = steering_motor_cmd;

  return hardware_interface::return_type::OK;
}

}  // namespace rinkrover_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  rinkrover_hardware::RRSystemHardware, hardware_interface::SystemInterface)
