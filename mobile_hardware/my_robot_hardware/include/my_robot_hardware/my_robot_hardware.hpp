#ifndef MY_ROBOT_HARDWARE_HPP_
#define MY_ROBOT_HARDWARE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace my_robot_hardware
{

class MyRobotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MyRobotHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

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

private:
  bool openSerial();
  void closeSerial();
  bool writeVelocity(double left, double right);
  bool readEncoderLine(double & left, double & right);

  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;

  std::string serial_port_{"/dev/ttyUSB0"};
  int baud_rate_{115200};
  double gear_ratio_{21.3};
  double encoder_cpr_{58.0};

  int idx_left_{-1};
  int idx_right_{-1};

  int serial_fd_{-1};
  std::string rx_buffer_;
  bool first_read_{true};
  double prev_encoder_[2] = {0.0, 0.0};
};

}  // namespace my_robot_hardware

#endif  // MY_ROBOT_HARDWARE_HPP_