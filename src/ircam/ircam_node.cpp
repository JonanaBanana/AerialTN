#include "ircam/ircam_node.hpp"

#include <chrono>
#include <cstring>

#ifndef __linux__
#include <pthread.h>
#include <sched.h>
#endif

using namespace std::chrono_literals;

namespace ir_v4l2_camera {

IrCameraNode::IrCameraNode(const rclcpp::NodeOptions& options) : Node("ir_v4l2_camera", options) {
    declare_params();
    read_params();

    auto qos = rclcpp::QoS(queue_depth_).reliability(rclcpp::ReliabilityPolicy::BestEffort).durability(rclcpp::DurabilityPolicy::Volatile);

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("ircam/image_raw", qos);

    // Open device
    if (!capture_.open(capture_cfg_)) {
        RCLCPP_FATAL(this->get_logger(),
            "FAiled to open V4L2 device '%s' - Check device path and permissions ",
            capture_cfg_.device.c_str()    
        );
        throw std::runtime_error("V4L2 open failed");
    }

    RCLCPP_INFO(this->get_logger(),
        "V4L2 opened: %s  %dx%d @ %d fps (%zu mmap buffers)",
        capture_cfg_.device.c_str(),
        capture_.width(), capture_.height(), capture_.fps(),
        static_cast<size_t>(capture_cfg_.num_buffers)
    );

    // Start stream
    if (!capture_.start()) {
        RCLCPP_FATAL(this->get_logger(),
            "V4L2 STREAMON failed (errno %d: %s)",
            errno, strerror(errno)    
        );
        throw std::runtime_error("V4L2 start failed");
    }

    // Diagnostics timer (2 sec interval)
    last_diag_time_ = this->now();
    diag_timer_ = this->create_wall_timer(
        2s,
        std::bind(&IrCameraNode::diagnostics_callback, this)
    );

    // Launch capture thread
    running_ = true;
    capture_thread_ = std::thread(&IrCameraNode::capture_loop, this);

    // Try to pin thread to specific core
    #ifdef __linux__
    {
        auto handle = capture_thread_.native_handle();
        sched_param sp{};
        sp.sched_priority = 50;
        if (pthread_setschedparam(handle, SCHED_FIFO, &sp) != 0) {
            RCLCPP_INFO(this->get_logger(),
                "Could not set SCHED_FIFO on capture thread - Falling back to default scheduling"
            );
        }
        // pin to core 0 (adjust?)
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(0, &cpuset);
        pthread_setaffinity_np(handle, sizeof(cpu_set_t), &cpuset);
    }
    #endif

    RCLCPP_INFO(this->get_logger(),
        "Streaming -> Topic '%s' w. encoding: %s",
        image_pub_->get_topic_name(),
        output_encoding_ == OutputEncoding::YUYV ? "yuv422_yuy2" : output_encoding_ == OutputEncoding::MONO16 ? "mono16" : "mono8"
    );
}

IrCameraNode::~IrCameraNode() {
    running_ = false;
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    capture_.close();

    auto& s = capture_.stats();
    RCLCPP_INFO(this->get_logger(),
        "Shutdown - captured: %lu  dropped: %lu  timeouts: %lu",
        s.frames_captured, s.frames_dropped, s.select_timeouts
    );
}

void IrCameraNode::declare_params() {
    this->declare_parameter<std::string>("device", "/dev/video0");
    this->declare_parameter<int>("width", 640);
    this->declare_parameter<int>("height", 512);
    this->declare_parameter<int>("fps", 30);
    this->declare_parameter<int>("v4l2_buffers", 8);
    this->declare_parameter<std::string>("encoding", "yuyv");
    this->declare_parameter<std::string>("frame_id", "ircam");
    this->declare_parameter<int>("queue_depth", 2);
}

void IrCameraNode::read_params() {
    capture_cfg_.device = this->get_parameter("device").as_string();
    capture_cfg_.width = this->get_parameter("width").as_int();
    capture_cfg_.height = this->get_parameter("height").as_int();
    capture_cfg_.fps = this->get_parameter("fps").as_int();
    capture_cfg_.num_buffers = this->get_parameter("v4l2_buffers").as_int();

    frame_id_ = this->get_parameter("frame_id").as_string();
    queue_depth_ = this->get_parameter("queue_depth").as_int();

    auto enc_str = this->get_parameter("encoding").as_string();
    if (enc_str == "yuv422" || enc_str == "yuyv") output_encoding_ = OutputEncoding::YUYV;
    else if (enc_str == "mono16") output_encoding_ = OutputEncoding::MONO16;
    else output_encoding_ = OutputEncoding::MONO8;
}

void IrCameraNode::capture_loop() {
    while (running_.load(std::memory_order_relaxed) && rclcpp::ok()) {
        size_t bytes_used = 0;
        uint64_t ts_us = 0;

        const uint8_t* frame = capture_.grab_frame(bytes_used, ts_us);

        if (!frame) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "V4L2: grap_frame failed (timeouts: %lu, drops: %lu)",
                capture_.stats().select_timeouts,
                capture_.stats().frames_dropped
            );
            continue;
        }

        switch (output_encoding_) {
            case OutputEncoding::YUYV:
                publish_yuyv(frame, bytes_used, ts_us);
                break;
            case OutputEncoding::MONO8:
                publish_mono8(frame, bytes_used, ts_us);
                break;
            case OutputEncoding::MONO16:
                publish_mono16(frame, bytes_used, ts_us);
                break;
        }

        // return the buffer to V4L2 after copying data out
        capture_.release_frame();
    }
}

void IrCameraNode::publish_yuyv(const uint8_t* data, size_t size, uint64_t ts_us) {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();

    msg->header.stamp.sec = static_cast<int32_t>(ts_us / 1'000'000ULL);
    msg->header.stamp.nanosec = static_cast<uint32_t>((ts_us % 1'000'000ULL) * 1000ULL);
    msg->header.frame_id = frame_id_;

    msg->width = static_cast<uint32_t>(capture_.width());
    msg->height = static_cast<uint32_t>(capture_.height());
    msg->encoding = "yuv422_yuy2";
    msg->step = static_cast<uint32_t>(capture_.width() * 2);
    msg->is_bigendian = false;

    msg->data.assign(data, data + size);
    image_pub_->publish(std::move(msg));
    publish_count_.fetch_add(1, std::memory_order_relaxed);
}

void IrCameraNode::publish_mono8(const uint8_t* data, size_t size, uint64_t ts_us) {

}

void IrCameraNode::publish_mono16(const uint8_t* data, size_t size, uint64_t ts_us) {
    
}

void IrCameraNode::diagnostics_callback() {
    auto now_t = this->now();
    double dt = (now_t - last_diag_time_).seconds();
    if (dt < 0.01) return;

    uint64_t current = publish_count_.load(std::memory_order_relaxed);
    double fps = static_cast<double>(current - last_diag_count_) / dt;

    last_diag_count_ = current;
    last_diag_time_ = now_t;

    auto& s = capture_.stats();

    RCLCPP_INFO(this->get_logger(),
        "FPS: %.1f  |  Captured: %lu  dropped: %lu  timeouts: %lu",
        fps, s.frames_captured, s.frames_dropped, s.select_timeouts
    );

    if (fps < capture_.fps() * 0.8 && s.frames_captured > 10) {
        RCLCPP_WARN(this->get_logger(),
            "FPS %.1f is below target %d - check usb bandwidth, cpu load etc...",
            fps, capture_.fps()
        );
    }
}

} // namespace ir_v4l2_camera