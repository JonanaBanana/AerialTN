#ifndef IRCAM_NODE_HPP_
#define IRCAM_NODE_HPP_

#include <atomic>
#include <memory>
#include <thread>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "ircam/v4l2_capture.hpp"

namespace ir_v4l2_camera {

enum class OutputEncoding {
    YUYV, // RAW YUYV 4:2:2 (2 bytes/pixel)
    MONO8, // Y-Channel only (1 byte/pixel)
    MONO16, // 16 bit ir??
};

class IrCameraNode : public rclcpp::Node {
public:
    explicit IrCameraNode(const rclcpp::NodeOptions& options);
    ~IrCameraNode() override;

private:
    void declare_params();
    void read_params();

    void capture_loop();

    void publish_yuyv(const uint8_t* data, size_t size, uint64_t ts_us);
    void publish_mono8(const uint8_t* data, size_t size, uint64_t ts_us);
    void publish_mono16(const uint8_t* data, size_t size, uint64_t ts_us);

    void diagnostics_callback();

    V4L2Capture capture_;
    CaptureConfig capture_cfg_;
    OutputEncoding output_encoding_ = OutputEncoding::YUYV;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    std::atomic<bool> running_{false};
    std::thread capture_thread_;

    std::string frame_id_;
    int queue_depth_ = 2;

    std::atomic<uint64_t> publish_count_{0};
    uint64_t last_diag_count_ = 0;
    rclcpp::Time last_diag_time_;
};

} // namespace ir_v4l2_camera


#endif