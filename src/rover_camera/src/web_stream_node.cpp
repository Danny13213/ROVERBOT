#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class WebStreamNode : public rclcpp::Node {
public:
  WebStreamNode() : Node("web_stream_node") {
    port_ = declare_parameter<int>("port", 5000);
    quality_ = declare_parameter<int>("jpeg_quality", 75);
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/color/image_raw");

    sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&WebStreamNode::image_callback, this, std::placeholders::_1));

    running_ = true;
    server_thread_ = std::thread(&WebStreamNode::server_loop, this);
    RCLCPP_INFO(get_logger(), "Browser stream: http://<JETSON_IP>:%d", port_);
  }

  ~WebStreamNode() override {
    running_ = false;
    if (server_fd_ >= 0) {
      ::shutdown(server_fd_, SHUT_RDWR);
      ::close(server_fd_);
      server_fd_ = -1;
    }
    if (server_thread_.joinable()) server_thread_.join();
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (msg->encoding != "bgr8" || msg->data.empty() || msg->width == 0 || msg->height == 0) return;
    const size_t needed = static_cast<size_t>(msg->step) * msg->height;
    if (msg->data.size() < needed || msg->step < msg->width * 3) return;

    try {
      cv::Mat image(
        static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
        const_cast<unsigned char *>(msg->data.data()), static_cast<size_t>(msg->step));
      std::vector<uchar> encoded;
      const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, quality_};
      if (cv::imencode(".jpg", image, encoded, params)) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        jpeg_ = std::move(encoded);
        ++frame_id_;
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "JPEG conversion failed: %s", e.what());
    }
  }

  static bool send_all(int fd, const void *data, size_t len) {
    const char *p = static_cast<const char *>(data);
    while (len > 0) {
      const ssize_t n = ::send(fd, p, len, MSG_NOSIGNAL);
      if (n <= 0) return false;
      p += n;
      len -= static_cast<size_t>(n);
    }
    return true;
  }

  void send_text(int client, const std::string &text) { send_all(client, text.data(), text.size()); }

  void handle_client(int client) {
    char request[2048]{};
    const ssize_t n = ::recv(client, request, sizeof(request) - 1, 0);
    if (n <= 0) return;
    const std::string req(request, static_cast<size_t>(n));

    if (req.find("GET /stream") == 0) {
      send_text(client,
        "HTTP/1.1 200 OK\r\n"
        "Cache-Control: no-cache, no-store, must-revalidate\r\n"
        "Pragma: no-cache\r\n"
        "Connection: close\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n");

      uint64_t last_id = 0;
      while (running_) {
        std::vector<uchar> frame;
        uint64_t id = 0;
        {
          std::lock_guard<std::mutex> lock(frame_mutex_);
          id = frame_id_;
          if (id != last_id) frame = jpeg_;
        }
        if (frame.empty() || id == last_id) {
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
          continue;
        }
        last_id = id;
        std::ostringstream hdr;
        hdr << "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " << frame.size() << "\r\n\r\n";
        const std::string header = hdr.str();
        if (!send_all(client, header.data(), header.size()) ||
            !send_all(client, frame.data(), frame.size()) ||
            !send_all(client, "\r\n", 2)) break;
      }
    } else {
      const std::string body =
        "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>ROVERBOT Camera</title></head>"
        "<body style='margin:0;background:#111;color:white;font-family:sans-serif;text-align:center'>"
        "<h2>ROVERBOT - Orbbec Astra Pro</h2>"
        "<img src='/stream' style='max-width:100%;height:auto' alt='camera stream'>"
        "</body></html>";
      std::ostringstream response;
      response << "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
               << "Content-Length: " << body.size() << "\r\nConnection: close\r\n\r\n" << body;
      send_text(client, response.str());
    }
  }

  void server_loop() {
    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Could not create HTTP socket");
      return;
    }
    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (::bind(server_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0 ||
        ::listen(server_fd_, 4) < 0) {
      RCLCPP_ERROR(get_logger(), "Could not listen on HTTP port %d", port_);
      return;
    }

    while (running_) {
      sockaddr_in client_addr{};
      socklen_t client_len = sizeof(client_addr);
      const int client = ::accept(server_fd_, reinterpret_cast<sockaddr *>(&client_addr), &client_len);
      if (client < 0) continue;
      std::thread([this, client]() {
        handle_client(client);
        ::shutdown(client, SHUT_RDWR);
        ::close(client);
      }).detach();
    }
  }

  int port_{5000};
  int quality_{75};
  std::string image_topic_;
  std::atomic<bool> running_{false};
  int server_fd_{-1};
  std::thread server_thread_;
  std::mutex frame_mutex_;
  std::vector<uchar> jpeg_;
  uint64_t frame_id_{0};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebStreamNode>());
  rclcpp::shutdown();
  return 0;
}
