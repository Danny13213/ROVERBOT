#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

using namespace std::chrono_literals;

class RgbNode : public rclcpp::Node {
public:
  RgbNode() : Node("astra_rgb") {
    device_ = declare_parameter<int>("device", 0);
    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 480);
    fps_ = declare_parameter<int>("fps", 30);
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_link");
    fx_ = declare_parameter<double>("fx", 525.0);
    fy_ = declare_parameter<double>("fy", 525.0);
    cx_ = declare_parameter<double>("cx", width_ / 2.0);
    cy_ = declare_parameter<double>("cy", height_ / 2.0);

    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", 10);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info", 10);

    cap_.open(device_, cv::CAP_V4L2);
    if (!cap_.isOpened()) throw std::runtime_error("Could not open Astra RGB V4L2 camera");
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

    const auto period = std::chrono::milliseconds(std::max(1, 1000 / std::max(1, fps_)));
    timer_ = create_wall_timer(period, std::bind(&RgbNode::capture, this));
    RCLCPP_INFO(get_logger(), "RGB camera opened: /dev/video%d %dx%d @ %d FPS",
                device_, width_, height_, fps_);
  }

private:
  sensor_msgs::msg::CameraInfo camera_info(const rclcpp::Time &stamp, int w, int h) const {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = frame_id_;
    info.width = static_cast<uint32_t>(w);
    info.height = static_cast<uint32_t>(h);
    info.distortion_model = "plumb_bob";
    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    info.k = {fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {fx_, 0.0, cx_, 0.0, 0.0, fy_, cy_, 0.0, 0.0, 0.0, 1.0, 0.0};
    return info;
  }

  void capture() {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "RGB frame read failed");
      return;
    }

    const auto stamp = now();
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id_;
    auto image = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
    image_pub_->publish(*image);
    info_pub_->publish(camera_info(stamp, frame.cols, frame.rows));
  }

  int device_{0}, width_{640}, height_{480}, fps_{30};
  std::string frame_id_;
  double fx_{525.0}, fy_{525.0}, cx_{320.0}, cy_{240.0};
  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RgbNode>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("astra_rgb"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
