#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MotorNode : public rclcpp::Node {
public:
  MotorNode() : Node("rover_motor") {
    device_ = declare_parameter<std::string>("device", "/dev/ttyUSB0");
    left_pwm_ = declare_parameter<int>("left_pwm", 120);
    right_pwm_ = declare_parameter<int>("right_pwm", 120);
    linear_deadband_ = declare_parameter<double>("linear_deadband", 0.05);
    angular_deadband_ = declare_parameter<double>("angular_deadband", 0.05);
    command_timeout_ms_ = declare_parameter<int>("command_timeout_ms", 500);

    left_pwm_ = std::clamp(left_pwm_, 0, 255);
    right_pwm_ = std::clamp(right_pwm_, 0, 255);

    if (!open_serial()) throw std::runtime_error("Could not open ESP32 serial port");
    send_pwm();
    send("S\n");

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&MotorNode::cmd_vel_callback, this, std::placeholders::_1));

    param_cb_ = add_on_set_parameters_callback(
      std::bind(&MotorNode::parameters_callback, this, std::placeholders::_1));

    last_cmd_ = now();
    timer_ = create_wall_timer(100ms, std::bind(&MotorNode::watchdog, this));
    RCLCPP_INFO(get_logger(), "ESP32 connected on %s, PWM L=%d R=%d",
                device_.c_str(), left_pwm_, right_pwm_);
  }

  ~MotorNode() override {
    if (fd_ >= 0) {
      send("S\n");
      ::close(fd_);
    }
  }

private:
  bool open_serial() {
    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "open(%s): %s", device_.c_str(), std::strerror(errno));
      return false;
    }
    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) return false;
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    return tcsetattr(fd_, TCSANOW, &tty) == 0;
  }

  void send(const std::string &s) {
    if (fd_ < 0) return;
    const char *p = s.data();
    size_t remaining = s.size();
    while (remaining > 0) {
      const ssize_t n = ::write(fd_, p, remaining);
      if (n < 0) {
        if (errno == EINTR) continue;
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Serial write failed: %s", std::strerror(errno));
        return;
      }
      p += n;
      remaining -= static_cast<size_t>(n);
    }
  }

  void send_pwm() {
    send("A" + std::to_string(left_pwm_) + "\n");
    std::this_thread::sleep_for(20ms);
    send("C" + std::to_string(right_pwm_) + "\n");
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const double x = msg->linear.x;
    const double z = msg->angular.z;
    char command = 'S';

    if (std::abs(x) > linear_deadband_) {
      if (x > 0.0) command = 'F';
      else command = 'B';
    } else if (std::abs(z) > angular_deadband_) {
      if (z > 0.0) command = 'L';
      else command = 'R';
    }

    current_command_ = command;
    last_cmd_ = now();
    send(std::string(1, command) + "\n");
  }

  void watchdog() {
    const auto elapsed_ms = (now() - last_cmd_).nanoseconds() / 1000000;
    if (elapsed_ms > command_timeout_ms_) {
      if (current_command_ != 'S') {
        current_command_ = 'S';
        send("S\n");
      }
      return;
    }
    // Refresh the ESP32 command so its own safety watchdog stays satisfied.
    send(std::string(1, current_command_) + "\n");
  }

  rcl_interfaces::msg::SetParametersResult parameters_callback(
      const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool pwm_changed = false;
    for (const auto &p : params) {
      if (p.get_name() == "left_pwm") {
        const int v = p.as_int();
        if (v < 0 || v > 255) { result.successful = false; result.reason = "left_pwm must be 0..255"; return result; }
        left_pwm_ = v; pwm_changed = true;
      } else if (p.get_name() == "right_pwm") {
        const int v = p.as_int();
        if (v < 0 || v > 255) { result.successful = false; result.reason = "right_pwm must be 0..255"; return result; }
        right_pwm_ = v; pwm_changed = true;
      }
    }
    if (pwm_changed) {
      send_pwm();
      RCLCPP_INFO(get_logger(), "PWM updated: L=%d R=%d", left_pwm_, right_pwm_);
    }
    return result;
  }

  int fd_{-1};
  std::string device_;
  int left_pwm_{120};
  int right_pwm_{120};
  double linear_deadband_{0.05};
  double angular_deadband_{0.05};
  int command_timeout_ms_{500};
  char current_command_{'S'};
  rclcpp::Time last_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MotorNode>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("rover_motor"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
