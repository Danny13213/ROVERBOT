#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/string.hpp"

namespace {
class TrtLogger : public nvinfer1::ILogger {
public:
  void log(Severity severity, const char *msg) noexcept override {
    if (severity <= Severity::kWARNING) RCLCPP_WARN(rclcpp::get_logger("tensorrt"), "%s", msg);
  }
};

void cuda_check(cudaError_t e, const char *what) {
  if (e != cudaSuccess) throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(e));
}

size_t volume(const nvinfer1::Dims &dims) {
  size_t v = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) throw std::runtime_error("TensorRT tensor has unresolved dynamic dimensions");
    v *= static_cast<size_t>(dims.d[i]);
  }
  return v;
}

const std::vector<std::string> kCoco = {
  "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light",
  "fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow",
  "elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee",
  "skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle",
  "wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange",
  "broccoli","carrot","hot dog","pizza","donut","cake","chair","couch","potted plant","bed",
  "dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven",
  "toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"
};

struct Detection {
  cv::Rect box;
  float score;
  int class_id;
};
}  // namespace

class YoloNode : public rclcpp::Node {
public:
  YoloNode() : Node("rover_yolo") {
    engine_path_ = declare_parameter<std::string>("engine", "models/yolo11n.engine");
    conf_ = declare_parameter<double>("confidence", 0.35);
    iou_ = declare_parameter<double>("iou", 0.45);
    input_size_ = declare_parameter<int>("input_size", 640);

    load_engine();
    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>("/yolo/annotated", 5);
    detections_pub_ = create_publisher<std_msgs::msg::String>("/yolo/detections", 10);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&YoloNode::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "YOLO TensorRT loaded: %s (%dx%d)",
                engine_path_.c_str(), input_w_, input_h_);
  }

  ~YoloNode() override {
    if (stream_) cudaStreamDestroy(stream_);
    if (input_device_) cudaFree(input_device_);
    if (output_device_) cudaFree(output_device_);
  }

private:
  void load_engine() {
    std::ifstream file(engine_path_, std::ios::binary);
    if (!file) throw std::runtime_error("Cannot open TensorRT engine: " + engine_path_);
    file.seekg(0, std::ios::end);
    const size_t size = static_cast<size_t>(file.tellg());
    file.seekg(0, std::ios::beg);
    std::vector<char> bytes(size);
    file.read(bytes.data(), static_cast<std::streamsize>(size));

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) throw std::runtime_error("createInferRuntime failed");
    engine_.reset(runtime_->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (!engine_) throw std::runtime_error("deserializeCudaEngine failed");
    context_.reset(engine_->createExecutionContext());
    if (!context_) throw std::runtime_error("createExecutionContext failed");

    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char *name = engine_->getIOTensorName(i);
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) input_name_ = name;
      else if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kOUTPUT && output_name_.empty()) output_name_ = name;
    }
    if (input_name_.empty() || output_name_.empty()) throw std::runtime_error("Could not identify TensorRT input/output tensors");
    if (engine_->getTensorDataType(input_name_.c_str()) != nvinfer1::DataType::kFLOAT ||
        engine_->getTensorDataType(output_name_.c_str()) != nvinfer1::DataType::kFLOAT) {
      throw std::runtime_error("This node expects FP32 engine I/O. Export the engine with half=False.");
    }

    auto in_dims = engine_->getTensorShape(input_name_.c_str());
    if (in_dims.nbDims != 4) throw std::runtime_error("Expected NCHW YOLO input");
    input_h_ = in_dims.d[2] > 0 ? in_dims.d[2] : input_size_;
    input_w_ = in_dims.d[3] > 0 ? in_dims.d[3] : input_size_;
    if (in_dims.d[0] < 0 || in_dims.d[2] < 0 || in_dims.d[3] < 0) {
      nvinfer1::Dims4 requested{1, 3, input_h_, input_w_};
      if (!context_->setInputShape(input_name_.c_str(), requested)) throw std::runtime_error("Could not set dynamic input shape");
    }

    const auto actual_in = context_->getTensorShape(input_name_.c_str());
    const auto out_dims = context_->getTensorShape(output_name_.c_str());
    input_count_ = volume(actual_in);
    output_dims_ = out_dims;
    output_count_ = volume(out_dims);

    cuda_check(cudaMalloc(&input_device_, input_count_ * sizeof(float)), "cudaMalloc input");
    cuda_check(cudaMalloc(&output_device_, output_count_ * sizeof(float)), "cudaMalloc output");
    cuda_check(cudaStreamCreate(&stream_), "cudaStreamCreate");
    if (!context_->setTensorAddress(input_name_.c_str(), input_device_) ||
        !context_->setTensorAddress(output_name_.c_str(), output_device_)) {
      throw std::runtime_error("setTensorAddress failed");
    }
    output_host_.resize(output_count_);
  }

  cv::Mat preprocess(const cv::Mat &image, float &scale, int &pad_x, int &pad_y) {
    scale = std::min(input_w_ / static_cast<float>(image.cols), input_h_ / static_cast<float>(image.rows));
    const int new_w = static_cast<int>(std::round(image.cols * scale));
    const int new_h = static_cast<int>(std::round(image.rows * scale));
    pad_x = (input_w_ - new_w) / 2;
    pad_y = (input_h_ - new_h) / 2;

    cv::Mat resized, canvas(input_h_, input_w_, CV_8UC3, cv::Scalar(114, 114, 114));
    cv::resize(image, resized, cv::Size(new_w, new_h));
    resized.copyTo(canvas(cv::Rect(pad_x, pad_y, new_w, new_h)));
    return cv::dnn::blobFromImage(canvas, 1.0 / 255.0, cv::Size(), cv::Scalar(), true, false, CV_32F);
  }

  std::vector<Detection> decode(const cv::Size &original, float scale, int pad_x, int pad_y) {
    if (output_dims_.nbDims != 3) throw std::runtime_error("Expected YOLO output with 3 dimensions");
    const int a = output_dims_.d[1];
    const int b = output_dims_.d[2];
    const bool channels_first = a < b;
    const int channels = channels_first ? a : b;
    const int candidates = channels_first ? b : a;
    const int classes = channels - 4;
    if (classes <= 0) throw std::runtime_error("Unexpected YOLO output shape");

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> class_ids;

    auto value = [&](int candidate, int channel) -> float {
      if (channels_first) return output_host_[static_cast<size_t>(channel) * candidates + candidate];
      return output_host_[static_cast<size_t>(candidate) * channels + channel];
    };

    for (int i = 0; i < candidates; ++i) {
      int best_class = -1;
      float best_score = 0.0f;
      for (int c = 0; c < classes; ++c) {
        const float s = value(i, 4 + c);
        if (s > best_score) { best_score = s; best_class = c; }
      }
      if (best_score < static_cast<float>(conf_)) continue;

      const float cx = value(i, 0);
      const float cy = value(i, 1);
      const float w = value(i, 2);
      const float h = value(i, 3);
      int x1 = static_cast<int>((cx - w * 0.5f - pad_x) / scale);
      int y1 = static_cast<int>((cy - h * 0.5f - pad_y) / scale);
      int x2 = static_cast<int>((cx + w * 0.5f - pad_x) / scale);
      int y2 = static_cast<int>((cy + h * 0.5f - pad_y) / scale);
      x1 = std::clamp(x1, 0, original.width - 1);
      y1 = std::clamp(y1, 0, original.height - 1);
      x2 = std::clamp(x2, 0, original.width - 1);
      y2 = std::clamp(y2, 0, original.height - 1);
      if (x2 <= x1 || y2 <= y1) continue;
      boxes.emplace_back(x1, y1, x2 - x1, y2 - y1);
      scores.push_back(best_score);
      class_ids.push_back(best_class);
    }

    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, scores, static_cast<float>(conf_), static_cast<float>(iou_), keep);
    std::vector<Detection> result;
    result.reserve(keep.size());
    for (int idx : keep) result.push_back({boxes[idx], scores[idx], class_ids[idx]});
    return result;
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
      cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
      float scale = 1.0f;
      int pad_x = 0, pad_y = 0;
      cv::Mat blob = preprocess(image, scale, pad_x, pad_y);

      cuda_check(cudaMemcpyAsync(input_device_, blob.ptr<float>(), input_count_ * sizeof(float),
                                 cudaMemcpyHostToDevice, stream_), "copy input");
      if (!context_->enqueueV3(stream_)) throw std::runtime_error("TensorRT enqueueV3 failed");
      cuda_check(cudaMemcpyAsync(output_host_.data(), output_device_, output_count_ * sizeof(float),
                                 cudaMemcpyDeviceToHost, stream_), "copy output");
      cuda_check(cudaStreamSynchronize(stream_), "inference sync");

      const auto detections = decode(image.size(), scale, pad_x, pad_y);
      std::ostringstream json;
      json << "[";
      for (size_t i = 0; i < detections.size(); ++i) {
        const auto &d = detections[i];
        const std::string label = d.class_id >= 0 && d.class_id < static_cast<int>(kCoco.size())
          ? kCoco[d.class_id] : ("class_" + std::to_string(d.class_id));
        cv::rectangle(image, d.box, cv::Scalar(0, 255, 0), 2);
        std::ostringstream text;
        text.precision(2);
        text << std::fixed << label << " " << d.score;
        cv::putText(image, text.str(), cv::Point(d.box.x, std::max(15, d.box.y - 5)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        if (i) json << ",";
        json << "{\"class\":\"" << label << "\",\"confidence\":" << d.score
             << ",\"x\":" << d.box.x << ",\"y\":" << d.box.y
             << ",\"w\":" << d.box.width << ",\"h\":" << d.box.height << "}";
      }
      json << "]";

      std_msgs::msg::String det_msg;
      det_msg.data = json.str();
      detections_pub_->publish(det_msg);
      auto annotated = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, image).toImageMsg();
      annotated_pub_->publish(*annotated);
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "YOLO inference error: %s", e.what());
    }
  }

  TrtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  std::string engine_path_, input_name_, output_name_;
  double conf_{0.35}, iou_{0.45};
  int input_size_{640}, input_w_{640}, input_h_{640};
  size_t input_count_{0}, output_count_{0};
  nvinfer1::Dims output_dims_{};
  void *input_device_{nullptr};
  void *output_device_{nullptr};
  cudaStream_t stream_{nullptr};
  std::vector<float> output_host_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detections_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<YoloNode>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("rover_yolo"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
