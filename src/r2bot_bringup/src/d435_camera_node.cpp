#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <librealsense2/rs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class D435CameraNode : public rclcpp::Node {
public:
  D435CameraNode()
  : Node("d435_camera_node"),
    align_to_color_(RS2_STREAM_COLOR) {
    camera_name_ = this->declare_parameter<std::string>("camera_name", "camera");
    serial_no_ = this->declare_parameter<std::string>("serial_no", "");
    color_width_ = this->declare_parameter<int>("color_width", 640);
    color_height_ = this->declare_parameter<int>("color_height", 480);
    depth_width_ = this->declare_parameter<int>("depth_width", 640);
    depth_height_ = this->declare_parameter<int>("depth_height", 480);
    fps_ = this->declare_parameter<int>("fps", 30);
    color_frame_id_ = this->declare_parameter<std::string>("color_frame_id", "camera_color_optical_frame");
    depth_frame_id_ = this->declare_parameter<std::string>("depth_frame_id", "camera_color_optical_frame");

    color_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/" + camera_name_ + "/color/image_raw", rclcpp::SensorDataQoS());
    color_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/" + camera_name_ + "/color/camera_info", rclcpp::SensorDataQoS());
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/" + camera_name_ + "/aligned_depth_to_color/image_raw", rclcpp::SensorDataQoS());
    depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/" + camera_name_ + "/aligned_depth_to_color/camera_info", rclcpp::SensorDataQoS());

    start_pipeline();

    running_ = true;
    publish_thread_ = std::thread(&D435CameraNode::publish_loop, this);
  }

  ~D435CameraNode() override {
    running_ = false;
    if (publish_thread_.joinable()) {
      publish_thread_.join();
    }
    try {
      pipeline_.stop();
    } catch (const rs2::error &) {
    }
  }

private:
  void start_pipeline() {
    try {
      rs2::config cfg;
      cfg.enable_stream(RS2_STREAM_COLOR, color_width_, color_height_, RS2_FORMAT_BGR8, fps_);
      cfg.enable_stream(RS2_STREAM_DEPTH, depth_width_, depth_height_, RS2_FORMAT_Z16, fps_);
      if (!serial_no_.empty()) {
        cfg.enable_device(serial_no_);
      }

      auto profile = pipeline_.start(cfg);
      auto device = profile.get_device();

      depth_scale_m_ = 0.001;
      for (auto sensor : device.query_sensors()) {
        if (auto depth_sensor = sensor.as<rs2::depth_sensor>()) {
          depth_scale_m_ = depth_sensor.get_depth_scale();
          break;
        }
      }

      RCLCPP_INFO(
        this->get_logger(),
        "D435 started: name=%s serial=%s depth_scale=%.6f",
        camera_name_.c_str(),
        serial_no_.empty() ? "<auto>" : serial_no_.c_str(),
        depth_scale_m_);
    } catch (const rs2::error &exc) {
      throw std::runtime_error(
              "Failed to start D435 pipeline: " + std::string(exc.what()));
    }
  }

  void publish_loop() {
    while (rclcpp::ok() && running_) {
      try {
        rs2::frameset frames = pipeline_.wait_for_frames(1000);
        frames = align_to_color_.process(frames);

        rs2::video_frame color_frame = frames.get_color_frame();
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        if (!color_frame || !depth_frame) {
          continue;
        }

        const auto stamp = this->now();
        publish_color_frame(color_frame, stamp);
        publish_depth_frame(depth_frame, stamp);
      } catch (const rs2::error &exc) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "D435 frame read failed: %s", exc.what());
        std::this_thread::sleep_for(200ms);
      }
    }
  }

  void publish_color_frame(const rs2::video_frame &frame, const rclcpp::Time &stamp) {
    auto msg = sensor_msgs::msg::Image();
    msg.header.stamp = stamp;
    msg.header.frame_id = color_frame_id_;
    msg.width = static_cast<uint32_t>(frame.get_width());
    msg.height = static_cast<uint32_t>(frame.get_height());
    msg.encoding = sensor_msgs::image_encodings::BGR8;
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.get_stride_in_bytes());

    const auto *src = static_cast<const uint8_t *>(frame.get_data());
    msg.data.assign(src, src + msg.step * msg.height);
    color_pub_->publish(msg);

    color_info_pub_->publish(build_camera_info(frame, stamp, color_frame_id_));
  }

  void publish_depth_frame(const rs2::depth_frame &frame, const rclcpp::Time &stamp) {
    auto msg = sensor_msgs::msg::Image();
    msg.header.stamp = stamp;
    msg.header.frame_id = depth_frame_id_;
    msg.width = static_cast<uint32_t>(frame.get_width());
    msg.height = static_cast<uint32_t>(frame.get_height());
    msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.get_stride_in_bytes());

    const auto *src = static_cast<const uint8_t *>(frame.get_data());
    msg.data.assign(src, src + msg.step * msg.height);
    depth_pub_->publish(msg);

    depth_info_pub_->publish(build_camera_info(frame, stamp, depth_frame_id_));
  }

  sensor_msgs::msg::CameraInfo build_camera_info(
    const rs2::video_frame &frame,
    const rclcpp::Time &stamp,
    const std::string &frame_id) const {
    const auto intrinsics = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = frame_id;
    info.width = static_cast<uint32_t>(intrinsics.width);
    info.height = static_cast<uint32_t>(intrinsics.height);

    info.k = {
      intrinsics.fx, 0.0, intrinsics.ppx,
      0.0, intrinsics.fy, intrinsics.ppy,
      0.0, 0.0, 1.0};
    info.p = {
      intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
      0.0, intrinsics.fy, intrinsics.ppy, 0.0,
      0.0, 0.0, 1.0, 0.0};
    info.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};

    info.distortion_model = distortion_model_from_rs(intrinsics.model);
    info.d.assign(intrinsics.coeffs, intrinsics.coeffs + 5);
    return info;
  }

  static std::string distortion_model_from_rs(rs2_distortion model) {
    switch (model) {
      case RS2_DISTORTION_KANNALA_BRANDT4:
      case RS2_DISTORTION_FTHETA:
        return "equidistant";
      case RS2_DISTORTION_BROWN_CONRADY:
      case RS2_DISTORTION_INVERSE_BROWN_CONRADY:
      case RS2_DISTORTION_MODIFIED_BROWN_CONRADY:
      default:
        return "plumb_bob";
    }
  }

  std::string camera_name_;
  std::string serial_no_;
  std::string color_frame_id_;
  std::string depth_frame_id_;
  int color_width_;
  int color_height_;
  int depth_width_;
  int depth_height_;
  int fps_;
  double depth_scale_m_{0.001};

  rs2::pipeline pipeline_;
  rs2::align align_to_color_;
  std::atomic<bool> running_{false};
  std::thread publish_thread_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<D435CameraNode>();
    rclcpp::spin(node);
  } catch (const std::exception &exc) {
    RCLCPP_FATAL(rclcpp::get_logger("d435_camera_node"), "%s", exc.what());
  }
  rclcpp::shutdown();
  return 0;
}
