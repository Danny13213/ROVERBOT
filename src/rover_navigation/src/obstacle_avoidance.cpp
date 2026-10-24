#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ObstacleAvoidance : public rclcpp::Node {
public:
  ObstacleAvoidance() : Node("obstacle_avoidance") {
    // Default to sensor-only diagnostics; no motion until explicitly enabled.
    allow_motion_ = declare_parameter<bool>("allow_motion", false);
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
    stop_for_person_ = declare_parameter<bool>("stop_for_person", true);
    person_hold_s_ = declare_parameter<double>("person_hold_s", 0.75);
    person_confirm_frames_ = declare_parameter<int>("person_confirm_frames", 3);
    person_clear_frames_ = declare_parameter<int>("person_clear_frames", 3);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/navigation/status", 10);

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_raw", 10,
      std::bind(&ObstacleAvoidance::depth_callback, this, std::placeholders::_1));
    yolo_sub_ = create_subscription<std_msgs::msg::String>(
      "/yolo/detections", 10,
      std::bind(&ObstacleAvoidance::yolo_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(50ms, std::bind(&ObstacleAvoidance::control_loop, this));
    state_start_ = now();
    last_person_seen_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    RCLCPP_INFO(get_logger(), "Obstacle avoidance ready (depth + YOLO person stop)");
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
    if (msg->encoding != "16UC1" || msg->data.empty() || msg->width == 0 || msg->height == 0 ||
        msg->step < msg->width * sizeof(uint16_t) ||
        msg->data.size() < static_cast<size_t>(msg->step) * msg->height) return;
    width_ = static_cast<int>(msg->width);
    height_ = static_cast<int>(msg->height);
    step_ = static_cast<int>(msg->step);
    depth_ = msg->data;
    have_depth_ = true;
    last_depth_ = now();
  }

  void yolo_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (!stop_for_person_) return;

    const bool person_detected =
      msg->data.find("\"class\":\"person\"") != std::string::npos;

    if (person_detected) {
      last_person_seen_ = now();
      person_clear_count_ = 0;
      if (person_detect_count_ < person_confirm_frames_) ++person_detect_count_;

      if (!person_blocking_ && person_detect_count_ >= person_confirm_frames_) {
        person_blocking_ = true;
        person_avoidance_ = true;
        set_state(State::BLOCKED);
        RCLCPP_WARN(
          get_logger(), "PERSON CONFIRMED for %d detection frames: stopping and scanning for alternate path",
          person_detect_count_);
      }
    } else {
      person_detect_count_ = 0;
      if (person_blocking_) {
        ++person_clear_count_;
        if (person_clear_count_ >= person_clear_frames_) {
          person_blocking_ = false;
          person_clear_count_ = 0;
          // Stay blocked until the operator deliberately restarts the node.
          RCLCPP_INFO(
            get_logger(), "Person absent for %d detection frames: resuming depth navigation",
            person_clear_frames_);
        }
      }
    }
  }

  bool person_recent() const {
    if (!stop_for_person_ || !person_blocking_) return false;
    if (last_person_seen_.nanoseconds() == 0) return false;
    return (now() - last_person_seen_).seconds() <= person_hold_s_;
  }

  double region_distance(double x0f, double x1f, double y0f = 0.15, double y1f = 0.90) const {
    if (!have_depth_ || width_ <= 0 || height_ <= 0) return std::numeric_limits<double>::quiet_NaN();
    const int x0 = std::clamp(static_cast<int>(width_ * x0f), 0, width_ - 1);
    const int x1 = std::clamp(static_cast<int>(width_ * x1f), x0 + 1, width_);
    const int y0 = std::clamp(static_cast<int>(height_ * y0f), 0, height_ - 1);
    const int y1 = std::clamp(static_cast<int>(height_ * y1f), y0 + 1, height_);

    std::vector<uint16_t> samples;
    samples.reserve(((x1 - x0) / 4 + 1) * ((y1 - y0) / 4 + 1));
    size_t sampled = 0;
    for (int y = y0; y < y1; y += 4) {
      for (int x = x0; x < x1; x += 4) {
        const size_t offset = static_cast<size_t>(y) * step_ + static_cast<size_t>(x) * 2;
        if (offset + 1 >= depth_.size()) continue;
        ++sampled;
        uint16_t mm = 0;
        std::memcpy(&mm, &depth_[offset], sizeof(mm));
        if (mm >= 100 && mm <= 8000) samples.push_back(mm);
      }
    }
    // Missing or sparse depth is UNKNOWN, never free space.
    if (samples.size() < 10 || sampled == 0 ||
        static_cast<double>(samples.size()) / sampled < 0.20)
      return std::numeric_limits<double>::quiet_NaN();
    const size_t idx = samples.size() / 10;
    std::nth_element(samples.begin(), samples.begin() + idx, samples.end());
    return samples[idx] / 1000.0;
  }

  void publish_cmd(double linear, double angular) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    cmd_pub_->publish(cmd);
  }

  void publish_status(const std::string &mode, double distance) {
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << mode;
    if (std::isfinite(distance)) {
      out << " | " << std::fixed << std::setprecision(2) << distance << " m";
    }
    msg.data = out.str();
    status_pub_->publish(msg);
  }

  void set_state(State s) {
    state_ = s;
    state_start_ = now();
  }

  double elapsed() const { return (now() - state_start_).seconds(); }

  void control_loop() {
    if (!allow_motion_) {
      publish_cmd(0.0, 0.0);
      publish_status("DIAGNOSTIC ONLY - STOP", std::numeric_limits<double>::quiet_NaN());
      return;
    }
    if (!have_depth_ || (now() - last_depth_).seconds() > 0.5) {
      publish_cmd(0.0, 0.0);
      publish_status("DEPTH LOST - STOP", std::numeric_limits<double>::infinity());
      return;
    }

    // Require valid readings across the width of the robot, not just image center.
    const double left_front = region_distance(0.12, 0.37);
    const double center_front = region_distance(0.37, 0.63);
    const double right_front = region_distance(0.63, 0.88);
    const double center = std::min({left_front, center_front, right_front});
    if (!std::isfinite(left_front) || !std::isfinite(center_front) ||
        !std::isfinite(right_front)) {
      publish_cmd(0.0, 0.0);
      publish_status("INVALID DEPTH - STOP", center);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Invalid depth in driving corridor; stopping");
      // Reset the maneuver so stale scan decisions cannot resume motion.
      set_state(State::BLOCKED);
      return;
    }

    if (person_recent() && state_ == State::DRIVE) {
      publish_cmd(0.0, 0.0);
      publish_status("PERSON DETECTED - STOP", center);
      return;
    }

    switch (state_) {
      case State::DRIVE:
        if (center <= too_close_distance_) {
          publish_cmd(0.0, 0.0);
          publish_status("TOO CLOSE - STOP LATCHED", center);
          RCLCPP_WARN(get_logger(), "TOO CLOSE %.2f m: stop latched; restart only after inspection", center);
          set_state(State::BLOCKED);
        } else if (center <= stop_distance_) {
          publish_cmd(0.0, 0.0);
          publish_status("OBSTACLE - STOP LATCHED", center);
          // Do not start an unverified turn next to furniture.
          set_state(State::BLOCKED);
        } else if (center <= caution_distance_) {
          publish_cmd(forward_speed_, 0.0);
          publish_status("CAUTION - FIXED MOTOR PWM", center);
        } else {
          publish_cmd(forward_speed_, 0.0);
          publish_status("PATH CLEAR", center);
        }
        break;

      case State::STOP_BEFORE_SCAN:
        publish_cmd(0.0, 0.0);
        publish_status(person_avoidance_ ? "PERSON - PREPARING SCAN" : "STOPPED - PREPARING SCAN", center);
        set_state(State::BLOCKED);
        break;

      case State::TURN_LEFT_SCAN:
        publish_cmd(0.0, 0.0);
        publish_status("SCANNING LEFT", center);
        if (elapsed() >= turn_time_s_) {
          publish_cmd(0.0, 0.0);
          set_state(State::SETTLE_LEFT);
        }
        break;

      case State::SETTLE_LEFT:
        publish_cmd(0.0, 0.0);
        publish_status("MEASURING LEFT", center);
        if (elapsed() >= settle_time_s_) {
          left_scan_ = region_distance(0.25, 0.75);
          RCLCPP_INFO(get_logger(), "Left scan: %.2f m", left_scan_);
          set_state(State::TURN_RIGHT_SCAN);
        }
        break;

      case State::TURN_RIGHT_SCAN:
        publish_cmd(0.0, 0.0);
        publish_status("SCANNING RIGHT", center);
        if (elapsed() >= 2.0 * turn_time_s_) {
          publish_cmd(0.0, 0.0);
          set_state(State::SETTLE_RIGHT);
        }
        break;

      case State::SETTLE_RIGHT:
        publish_cmd(0.0, 0.0);
        publish_status("MEASURING RIGHT", center);
        if (elapsed() >= settle_time_s_) {
          right_scan_ = region_distance(0.25, 0.75);
          RCLCPP_INFO(get_logger(), "Right scan: %.2f m", right_scan_);
          set_state(State::RETURN_CENTER);
        }
        break;

      case State::RETURN_CENTER:
        publish_cmd(0.0, turn_speed_);
        publish_status("RETURNING CENTER", center);
        if (elapsed() >= turn_time_s_) {
          publish_cmd(0.0, 0.0);
          set_state(State::SETTLE_CENTER);
        }
        break;

      case State::SETTLE_CENTER:
        publish_cmd(0.0, 0.0);
        publish_status("EVALUATING PATH", center);
        if (elapsed() >= settle_time_s_) {
          if (person_avoidance_) {
            const bool left_clear = left_scan_ >= scan_clearance_;
            const bool right_clear = right_scan_ >= scan_clearance_;
            if (left_clear || right_clear) {
              avoid_left_ = left_clear && (!right_clear || left_scan_ >= right_scan_);
              person_avoidance_ = false;
              person_blocking_ = false;
              person_detect_count_ = 0;
              person_clear_count_ = 0;
              RCLCPP_WARN(
                get_logger(), "Person alternate path selected: %s (L=%.2f R=%.2f)",
                avoid_left_ ? "LEFT" : "RIGHT", left_scan_, right_scan_);
              set_state(State::BLOCKED);
            } else {
              RCLCPP_ERROR(
                get_logger(), "No clear alternate path around person: L=%.2f R=%.2f",
                left_scan_, right_scan_);
              set_state(State::BLOCKED);
            }
          } else if (left_scan_ >= scan_clearance_ && right_scan_ >= scan_clearance_) {
            RCLCPP_WARN(get_logger(), "Scan clear L=%.2f R=%.2f: short reverse", left_scan_, right_scan_);
            set_state(State::BLOCKED);
          } else {
            RCLCPP_ERROR(get_logger(), "Reverse rejected: scan not clear L=%.2f R=%.2f", left_scan_, right_scan_);
            set_state(State::BLOCKED);
          }
        }
        break;

      case State::REVERSE:
        publish_cmd(0.0, 0.0);
        publish_status("REVERSING", center);
        if (elapsed() >= reverse_time_s_) {
          publish_cmd(0.0, 0.0);
          avoid_left_ = left_scan_ >= right_scan_;
          set_state(State::AVOID_TURN);
        }
        break;

      case State::AVOID_TURN:
        publish_cmd(0.0, 0.0);
        publish_status(avoid_left_ ? "AVOIDING - TURN LEFT" : "AVOIDING - TURN RIGHT", center);
        if (elapsed() >= turn_time_s_) {
          publish_cmd(0.0, 0.0);
          set_state(State::DRIVE);
        }
        break;

      case State::BLOCKED:
        publish_cmd(0.0, 0.0);
        publish_status(person_avoidance_ ? "PERSON/BLOCKED - STOP" : "BLOCKED - STOP", center);
        // Deliberately latched: camera clearance alone never restarts motion.
        break;
    }
  }

  bool allow_motion_{false};
  State state_{State::DRIVE};
  rclcpp::Time state_start_;
  rclcpp::Time last_depth_;
  rclcpp::Time last_person_seen_;
  bool have_depth_{false};
  bool avoid_left_{true};
  bool person_blocking_{false};
  bool person_avoidance_{false};
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
  bool stop_for_person_{true};
  double person_hold_s_{0.75};
  int person_confirm_frames_{3};
  int person_clear_frames_{3};
  int person_detect_count_{0};
  int person_clear_count_{0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidance>());
  rclcpp::shutdown();
  return 0;
}
