#include <OpenNI.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

using namespace std::chrono_literals;

class DepthNode : public rclcpp::Node {
public:
  DepthNode() : Node("astra_depth") {
    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 480);
    fps_ = declare_parameter<int>("fps", 30);
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_link");
    fx_ = declare_parameter<double>("fx", 525.0);
    fy_ = declare_parameter<double>("fy", 525.0);
    cx_ = declare_parameter<double>("cx", width_ / 2.0);
    cy_ = declare_parameter<double>("cy", height_ / 2.0);

    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", 10);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/camera/depth/camera_info", 10);

    check(openni::OpenNI::initialize(), "OpenNI2 initialize");
    check(device_.open(openni::ANY_DEVICE), "Astra open");
    check(depth_.create(device_, openni::SENSOR_DEPTH), "Depth stream create");

    openni::VideoMode mode = depth_.getVideoMode();
    mode.setResolution(width_, height_);
    mode.setFps(fps_);
    mode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
    const auto mode_status = depth_.setVideoMode(mode);
    if (mode_status != openni::STATUS_OK) {
      RCLCPP_WARN(get_logger(), "Requested depth mode not accepted; using camera default mode");
    }

    check(depth_.start(), "Depth stream start");
    const auto active = depth_.getVideoMode();
    width_ = active.getResolutionX();
    height_ = active.getResolutionY();
    fps_ = active.getFps();

    const auto period = std::chrono::milliseconds(std::max(1, 1000 / std::max(1, fps_)));
    timer_ = create_wall_timer(period, std::bind(&DepthNode::capture, this));
    RCLCPP_INFO(get_logger(), "Astra depth started: %dx%d @ %d FPS", width_, height_, fps_);
  }

  ~DepthNode() override {
    depth_.stop();
    depth_.destroy();
    device_.close();
    openni::OpenNI::shutdown();
  }

private:
  void check(openni::Status status, const char *operation) {
    if (status != openni::STATUS_OK) {
      throw std::runtime_error(std::string(operation) + " failed: " + openni::OpenNI::getExtendedError());
    }
  }

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
    openni::VideoFrameRef frame;
    const auto status = depth_.readFrame(&frame);
    if (status != openni::STATUS_OK || !frame.isValid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Depth frame read failed");
      return;
    }

    const auto stamp = now();
    sensor_msgs::msg::Image image;
    image.header.stamp = stamp;
    image.header.frame_id = frame_id_;
    image.height = static_cast<uint32_t>(frame.getHeight());
    image.width = static_cast<uint32_t>(frame.getWidth());
    image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    image.is_bigendian = false;
    image.step = image.width * sizeof(uint16_t);
    image.data.resize(static_cast<size_t>(image.step) * image.height);
    std::memcpy(image.data.data(), frame.getData(), image.data.size());

    image_pub_->publish(image);
    info_pub_->publish(camera_info(stamp, frame.getWidth(), frame.getHeight()));
  }

  int width_{640}, height_{480}, fps_{30};
  std::string frame_id_;
  double fx_{525.0}, fy_{525.0}, cx_{320.0}, cy_{240.0};
  openni::Device device_;
  openni::VideoStream depth_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DepthNode>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("astra_depth"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
