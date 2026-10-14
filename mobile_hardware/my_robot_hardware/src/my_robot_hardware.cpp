#include "my_robot_hardware/my_robot_hardware.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_robot_hardware
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

speed_t baudToSpeed(int baud)
{
  switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    default:     return B115200;
  }
}
}  // namespace

hardware_interface::CallbackReturn MyRobotHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotHardware"), "URDF cần khai báo đúng 2 joint!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (info_.joints[i].name == "left_wheel_joint")  { idx_left_ = static_cast<int>(i); }
    if (info_.joints[i].name == "right_wheel_joint") { idx_right_ = static_cast<int>(i); }
  }

  if (idx_left_ < 0 || idx_right_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotHardware"), "Không tìm thấy tên joint left_wheel_joint và right_wheel_joint!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto get_param = [this](const std::string & key, double def) -> double {
    auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end()) return def;
    try { return std::stod(it->second); } catch (...) { return def; }
  };

  auto it = info_.hardware_parameters.find("serial_port");
  if (it == info_.hardware_parameters.end() || it->second.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotHardware"), "Thiếu tham số 'serial_port'!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  serial_port_ = it->second;

  baud_rate_   = static_cast<int>(get_param("baud_rate", 115200));
  gear_ratio_  = get_param("gear_ratio", 34.0);
  encoder_cpr_ = get_param("encoder_cpr", 44.0);

  hw_commands_velocity_.resize(info_.joints.size(), 0.0);
  hw_states_position_.resize(info_.joints.size(), 0.0);
  hw_states_velocity_.resize(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MyRobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MyRobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn MyRobotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  first_read_ = true;
  prev_encoder_[0] = 0.0;
  prev_encoder_[1] = 0.0;
  rx_buffer_.clear();

  if (!openSerial()) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotHardware"), "Lỗi mở cổng Serial %s!", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyRobotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  writeVelocity(0.0, 0.0);
  closeSerial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MyRobotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (serial_fd_ < 0) return hardware_interface::return_type::OK;

  double enc[2] = {0.0, 0.0};
  if (!readEncoderLine(enc[0], enc[1])) return hardware_interface::return_type::OK;

  if (first_read_) {
    first_read_ = false;
    prev_encoder_[0] = enc[0];
    prev_encoder_[1] = enc[1];
    return hardware_interface::return_type::OK;
  }

  const double dt = period.seconds();
  const int joint_index[2] = {idx_left_, idx_right_};

  for (size_t i = 0; i < 2; ++i) {
    const double dcount = enc[i] - prev_encoder_[i];
    prev_encoder_[i] = enc[i];

    const double dpos = (dcount / encoder_cpr_) * 2.0 * kPi / gear_ratio_;

    hw_states_position_[joint_index[i]] += dpos;
    hw_states_velocity_[joint_index[i]] = (dt > 0.0) ? (dpos / dt) : 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyRobotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (serial_fd_ < 0) return hardware_interface::return_type::ERROR;

  if (!writeVelocity(hw_commands_velocity_[idx_left_], hw_commands_velocity_[idx_right_])) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

bool MyRobotHardware::openSerial()
{
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) return false;

  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(serial_fd_, &tty) != 0) { closeSerial(); return false; }

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  speed_t speed = baudToSpeed(baud_rate_);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) { closeSerial(); return false; }

  tcflush(serial_fd_, TCIOFLUSH);
  rx_buffer_.clear();
  return true;
}

void MyRobotHardware::closeSerial()
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool MyRobotHardware::writeVelocity(double left, double right)
{
  char buf[128];
  int n = snprintf(buf, sizeof(buf), "L%.4f R%.4f\n", left, right);
  if (n < 0) return false;

  ssize_t written = ::write(serial_fd_, buf, static_cast<size_t>(n));
  return (written == n);
}

bool MyRobotHardware::readEncoderLine(double & left, double & right)
{
  char buf[256];
  ssize_t n;
  while ((n = ::read(serial_fd_, buf, sizeof(buf))) > 0) {
    rx_buffer_.append(buf, static_cast<size_t>(n));
    if (rx_buffer_.size() > 2048) {
      rx_buffer_.erase(0, rx_buffer_.size() - 1024);
    }
  }

  size_t nl = rx_buffer_.rfind('\n');
  if (nl == std::string::npos) return false;

  size_t start = 0;
  if (nl > 0) {
    size_t prev = rx_buffer_.rfind('\n', nl - 1);
    start = (prev == std::string::npos) ? 0 : prev + 1;
  }

  std::string line = rx_buffer_.substr(start, nl - start);
  rx_buffer_.erase(0, nl + 1);

  double l = 0.0, r = 0.0;
  if (sscanf(line.c_str(), "L%lf R%lf", &l, &r) != 2) return false;

  left = l;
  right = r;
  return true;
}

}  // namespace my_robot_hardware

PLUGINLIB_EXPORT_CLASS(
  my_robot_hardware::MyRobotHardware, hardware_interface::SystemInterface)