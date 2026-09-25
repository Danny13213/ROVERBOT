#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class ObstacleAvoidance : public rclcpp::Node {
public:
  ObstacleAvoidance() : Node("obstacle_avoidance") {
    caution_distance_ = declare_parameter<double>("caution_distance", 1.0);
    stop_distance_ = declare_parameter<double>("stop_distance", 0.50);
    too_close_distance_ = declare_parameter<double>("too_close_distance", 0.25);
    scan_clearance_ = declare_parameter<double>("scan_clearance", 0.75);
    reverse_time_s_ = declare_parameter<double>("reverse_time_s", 0.8);
    turn_time_s_ = declare_parameter<double>("turn_time_s", 0.65);
    settle_time_s_ = declare_parameter<double>("settle_time_s", 0.35);
    forward_speed_ = declare_parameter<double>("forward_speed", 0.20);
    reverse_speed_ = declare_parameter<double>("reverse_speed", -0.15);
    turn_speed_ = declare_parameter<double>("turn_speed", 0.50);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_raw", 10,
      std::bind(&ObstacleAvoidance::depth_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(50ms, std::bind(&ObstacleAvoidance::control_loop, this));
    state_start_ = now();
    RCLCPP_INFO(get_logger(), "Obstacle avoidance ready");
  }

private:
  enum class State {
    DRIVE,
    STOP_BEFORE_SCAN,
    TURN_LEFT_SCAN,
    SETTLE_LEFT,
    TURN_RIGHT_SCAN,
    SETTLE_RIGHT,
    RETURN_CENTER,
    SETTLE_CENTER,
    REVERSE,
    AVOID_TURN,
    BLOCKED
  };

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (msg->encoding != "16UC1" || msg->data.empty()) return;
    width_ = static_cast<int>(msg->width);
    height_ = static_cast<int>(msg->height);
    step_ = static_cast<int>(msg->step);
    depth_ = msg->data;
    have_depth_ = true;
    last_depth_ = now();
  }

  double region_distance(double x0f, double x1f, double y0f = 0.30, double y1f = 0.85) const {
    if (!have_depth_ || width_ <= 0 || height_ <= 0) return std::numeric_limits<double>::infinity();
    const int x0 = std::clamp(static_cast<int>(width_ * x0f), 0, width_ - 1);
    const int x1 = std::clamp(static_cast<int>(width_ * x1f), x0 + 1, width_);
    const int y0 = std::clamp(static_cast<int>(height_ * y0f), 0, height_ - 1);
    const int y1 = std::clamp(static_cast<int>(height_ * y1f), y0 + 1, height_);

    std::vector<uint16_t> samples;
    samples.reserve(((x1 - x0) / 8 + 1) * ((y1 - y0) / 8 + 1));
    for (int y = y0; y < y1; y += 8) {
      for (int x = x0; x < x1; x += 8) {
        const size_t offset = static_cast<size_t>(y) * step_ + static_cast<size_t>(x) * 2;
        if (offset + 1 >= depth_.size()) continue;
        uint16_t mm = 0;
        std::memcpy(&mm, &depth_[offset], sizeof(mm));
        if (mm >= 100 && mm <= 8000) samples.push_back(mm);
      }
    }
    if (samples.size() < 10) return std::numeric_limits<double>::infinity();
    const size_t idx = samples.size() / 5;  // 20th percentile: conservative, less noisy than minimum.
    std::nth_element(samples.begin(), samples.begin() + idx, samples.end());
    return samples[idx] / 1000.0;
  }

  void publish(double linear, double angular) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    cmd_pub_->publish(cmd);
  }

  void set_state(State s) {
    state_ = s;
    state_start_ = now();
  }

  double elapsed() const { return (now() - state_start_).seconds(); }

  void control_loop() {
    if (!have_depth_ || (now() - last_depth_).seconds() > 0.5) {
      publish(0.0, 0.0);
      return;
    }

    const double center = region_distance(0.33, 0.67);

    switch (state_) {
      case State::DRIVE:
        if (center <= too_close_distance_) {
          publish(0.0, 0.0);
          RCLCPP_WARN(get_logger(), "TOO CLOSE %.2f m: stop, scan left/right before reversing", center);
          set_state(State::STOP_BEFORE_SCAN);
        } else if (center <= stop_distance_) {
          publish(0.0, 0.0);
          avoid_left_ = region_distance(0.00, 0.33) >= region_distance(0.67, 1.00);
          set_state(State::AVOID_TURN);
        } else {
          publish(forward_speed_, 0.0);
        }
        break;

      case State::STOP_BEFORE_SCAN:
        publish(0.0, 0.0);
        if (elapsed() >= settle_time_s_) set_state(State::TURN_LEFT_SCAN);
        break;

      case State::TURN_LEFT_SCAN:
        publish(0.0, turn_speed_);
        if (elapsed() >= turn_time_s_) {
          publish(0.0, 0.0);
          set_state(State::SETTLE_LEFT);
        }
        break;

      case State::SETTLE_LEFT:
        publish(0.0, 0.0);
        if (elapsed() >= settle_time_s_) {
          left_scan_ = region_distance(0.25, 0.75);
          RCLCPP_INFO(get_logger(), "Left scan: %.2f m", left_scan_);
          set_state(State::TURN_RIGHT_SCAN);
        }
        break;

      case State::TURN_RIGHT_SCAN:
        // Turn right for twice as long: from left-looking, through center, to right-looking.
        publish(0.0, -turn_speed_);
        if (elapsed() >= 2.0 * turn_time_s_) {
          publish(0.0, 0.0);
          set_state(State::SETTLE_RIGHT);
        }
        break;

      case State::SETTLE_RIGHT:
        publish(0.0, 0.0);
        if (elapsed() >= settle_time_s_) {
          right_scan_ = region_distance(0.25, 0.75);
          RCLCPP_INFO(get_logger(), "Right scan: %.2f m", right_scan_);
          set_state(State::RETURN_CENTER);
        }
        break;

      case State::RETURN_CENTER:
        publish(0.0, turn_speed_);
        if (elapsed() >= turn_time_s_) {
          publish(0.0, 0.0);
          set_state(State::SETTLE_CENTER);
        }
        break;

      case State::SETTLE_CENTER:
        publish(0.0, 0.0);
        if (elapsed() >= settle_time_s_) {
          // With one front camera we cannot see the rear continuously. Only reverse after
          // both side scans are clear; otherwise fail safe and stay stopped.
          if (left_scan_ >= scan_clearance_ && right_scan_ >= scan_clearance_) {
            RCLCPP_WARN(get_logger(), "Scan clear L=%.2f R=%.2f: short reverse", left_scan_, right_scan_);
            set_state(State::REVERSE);
          } else {
            RCLCPP_ERROR(get_logger(), "Reverse rejected: scan not clear L=%.2f R=%.2f", left_scan_, right_scan_);
            set_state(State::BLOCKED);
          }
        }
        break;

      case State::REVERSE:
        publish(reverse_speed_, 0.0);
        if (elapsed() >= reverse_time_s_) {
          publish(0.0, 0.0);
          avoid_left_ = left_scan_ >= right_scan_;
          set_state(State::AVOID_TURN);
        }
        break;

      case State::AVOID_TURN:
        publish(0.0, avoid_left_ ? turn_speed_ : -turn_speed_);
        if (elapsed() >= turn_time_s_) {
          publish(0.0, 0.0);
          set_state(State::DRIVE);
        }
        break;

      case State::BLOCKED:
        publish(0.0, 0.0);
        // Stay stopped until the forward view becomes comfortably clear again.
        if (center > caution_distance_) set_state(State::DRIVE);
        break;
    }
  }

  State state_{State::DRIVE};
  rclcpp::Time state_start_;
  rclcpp::Time last_depth_;
  bool have_depth_{false};
  bool avoid_left_{true};
  int width_{0}, height_{0}, step_{0};
  std::vector<uint8_t> depth_;
  double left_scan_{0.0}, right_scan_{0.0};

  double caution_distance_{1.0};
  double stop_distance_{0.50};
  double too_close_distance_{0.25};
  double scan_clearance_{0.75};
  double reverse_time_s_{0.8};
  double turn_time_s_{0.65};
  double settle_time_s_{0.35};
  double forward_speed_{0.20};
  double reverse_speed_{-0.15};
  double turn_speed_{0.50};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidance>());
  rclcpp::shutdown();
  return 0;
}
