#ifndef rinkrover_hardware__RINKROVER_SYSTEM_HPP_
#define rinkrover_hardware__RINKROVER_SYSTEM_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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

#include "rinkrover_hardware/arduino_comms.hpp"
#include "rinkrover_hardware/wheel.hpp"

namespace rinkrover_hardware
{
struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct Joint
{
  explicit Joint(const std::string & name) : joint_name(name)
  {
    state = JointValue();
    command = JointValue();
  }

  Joint() = default;

  std::string joint_name;
  JointValue state;
  JointValue command;
};
class RRSystemHardware : public hardware_interface::SystemInterface
{

struct Config
{
  std::string steering_joint_name = "";
  std::string traction_joint_name = "";
  std::string device = "";
  float track_width = 0.0;
  float wheelbase = 0.0;
  float loop_rate = 0.0;

  int baud_rate = 0;
  int timeout_ms = 0;
  int enc_counts_per_rev = 0;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RRSystemHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Get the logger of the SystemInterface.
  /**
   * \return logger of the SystemInterface.
   */
  rclcpp::Logger get_logger() const { return *logger_; }

  /// Get the clock of the SystemInterface.
  /**
   * \return clock of the SystemInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:
  ArduinoComms comms_;
  Config cfg_;
  Wheel wheel_l_;
  Wheel wheel_r_;

  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  // std::vector<std::tuple<std::string, double, double>>
  //   hw_interfaces_;  // name of joint, state, command
  std::map<std::string, Joint> hw_interfaces_;

  int last_left_motor_cmd = 0;
  int last_right_motor_cmd = 0;
  int last_steering_motor_cmd =0;

  int i2c_file_;
};

}  // namespace rinkrover_hardware

#endif  // rinkrover_hardware__RINKROVER_SYSTEM_HPP_
