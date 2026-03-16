#ifndef IRCAM_H264_ENC_HPP_
#define IRCAM_H264_ENC_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace ir_v4l2_camera {

// ---------------------------------------------------------------------------
// Helper: map ROS image encoding string → FFmpeg pixel format
// ---------------------------------------------------------------------------
static AVPixelFormat ros_encoding_to_av(const std::string & enc)
{
    if (enc == "rgb8")             return AV_PIX_FMT_RGB24;
    if (enc == "bgr8")             return AV_PIX_FMT_BGR24;
    if (enc == "rgba8")            return AV_PIX_FMT_RGBA;
    if (enc == "bgra8")            return AV_PIX_FMT_BGRA;
    if (enc == "mono8")            return AV_PIX_FMT_GRAY8;
    if (enc == "mono16")           return AV_PIX_FMT_GRAY16LE;
    if (enc == "8UC1")             return AV_PIX_FMT_GRAY8;
    if (enc == "8UC3")             return AV_PIX_FMT_BGR24;   // OpenCV default
    if (enc == "16UC1")            return AV_PIX_FMT_GRAY16LE;
    if (enc == "yuv422" || enc == "uyvy") return AV_PIX_FMT_UYVY422;
    if (enc == "yuyv")             return AV_PIX_FMT_YUYV422;
    if (enc == "yuv422_yuy2")      return AV_PIX_FMT_YUYV422;
    return AV_PIX_FMT_NONE;
}

// Encoder class
class H264Encoder {
public:
    H264Encoder() = default;
    ~H264Encoder() { shutdown(); }

    // Non-copyable
    H264Encoder(const H264Encoder &) = delete;
    H264Encoder & operator=(const H264Encoder &) = delete;

    /**
     * Lazily initialise (or reinitialise) the encoder when frame geometry or
     * pixel format changes.
     */
    void init(int width, int height, AVPixelFormat src_fmt,
              int bitrate, const std::string & preset);
    
    /**
     * Encode one raw frame and return the concatenated NAL-unit bytes.
     * May return an empty vector when the encoder is buffering.
     */
    std::vector<uint8_t> encode(const uint8_t * raw_data, int step);
    
    void shutdown();

    const std::string & encoder_name() const { return encoder_name_; }

private: 
    AVCodecContext* codec_ctx_ = nullptr;
    SwsContext* sws_ = nullptr;
    AVFrame* frame_ = nullptr;

    int width_ = 0;
    int height_ = 0;
    AVPixelFormat src_fmt_ = AV_PIX_FMT_NONE;
    int64_t pts_ = 0;
    std::string encoder_name_;        
};

// Encoder republisher ros2 node
class IrcamH264Republisher : public rclcpp::Node {
public:
    explicit IrcamH264Republisher(const rclcpp::NodeOptions& options);

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_h264_pub_;

    H264Encoder encoder_;

    int bitrate_;
    std::string preset_;
    bool logged_encoder_ = false;
};


} // namespace ir_v4l2_camera

#endif // IRCAM_H264_ENC_HPP_