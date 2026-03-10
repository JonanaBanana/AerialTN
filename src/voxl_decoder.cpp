// voxl_h264_decoder.cpp
//
// Subscribes to a voxl-mpa-to-ros2 H264 encoded image topic
// (sensor_msgs/CompressedImage with format="h264") and republishes as
// a standard BGR8 sensor_msgs/Image using libavcodec.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <cstring>
#include <stdexcept>
#include <string>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

// ---------------------------------------------------------------------------
// Small helper: av_err2str is a C99 compound-literal macro that doesn't
// compile cleanly in C++.  This wrapper is safe in all compilers.
// ---------------------------------------------------------------------------
static std::string av_error_string(int errnum)
{
    char buf[AV_ERROR_MAX_STRING_SIZE]{};
    av_strerror(errnum, buf, sizeof(buf));
    return {buf};
}

// ---------------------------------------------------------------------------

class VoxlH264Decoder : public rclcpp::Node
{
public:
    VoxlH264Decoder()
    : Node("voxl_h264_decoder"),
      codec_ctx_(nullptr),
      sws_ctx_(nullptr),
      frame_(nullptr),
      frame_bgr_(nullptr),
      bgr_buffer_size_(0)
    {
        declare_parameter("input_topic",  "/low_light_down_small_encoded");
        declare_parameter("output_topic", "/low_light_down_small_decoded");
        declare_parameter("frame_id",     "low_light_down_small");
        declare_parameter("encoding",     "h264");   // "h264" or "hevc"

        const auto in_topic  = get_parameter("input_topic").as_string();
        const auto out_topic = get_parameter("output_topic").as_string();
        frame_id_            = get_parameter("frame_id").as_string();
        const auto encoding  = get_parameter("encoding").as_string();

        // ---- FFmpeg setup ------------------------------------------------
        const AVCodec * codec = avcodec_find_decoder(
            encoding == "hevc" ? AV_CODEC_ID_HEVC : AV_CODEC_ID_H264);

        if (!codec)
            throw std::runtime_error("avcodec_find_decoder failed for " + encoding);

        codec_ctx_ = avcodec_alloc_context3(codec);
        if (!codec_ctx_)
            throw std::runtime_error("avcodec_alloc_context3 failed");

        if (avcodec_open2(codec_ctx_, codec, nullptr) < 0)
            throw std::runtime_error("avcodec_open2 failed");

        frame_     = av_frame_alloc();
        frame_bgr_ = av_frame_alloc();
        if (!frame_ || !frame_bgr_)
            throw std::runtime_error("av_frame_alloc failed");

        // ---- ROS2 setup --------------------------------------------------
        pub_ = create_publisher<sensor_msgs::msg::Image>(out_topic, 10);
        sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            in_topic, 10,
            std::bind(&VoxlH264Decoder::callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Decoding %s  →  %s  [%s]",
            in_topic.c_str(), out_topic.c_str(), encoding.c_str());
    }

    ~VoxlH264Decoder()
    {
        if (sws_ctx_)   sws_freeContext(sws_ctx_);
        if (frame_)     av_frame_free(&frame_);
        if (frame_bgr_) av_frame_free(&frame_bgr_);
        if (codec_ctx_) avcodec_free_context(&codec_ctx_);
    }

private:
    void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        const auto & fmt = msg->format;
        if (fmt != "h264" && fmt != "h265" && fmt != "hevc" &&
            fmt != "H264" && fmt != "H265")
        {
            RCLCPP_WARN_ONCE(get_logger(), "Unexpected format: %s", fmt.c_str());
            return;
        }
        AVPacket pkt{};
        pkt.data = const_cast<uint8_t *>(msg->data.data());
        pkt.size = static_cast<int>(msg->data.size());

        int ret = avcodec_send_packet(codec_ctx_, &pkt);
        if (ret < 0) {
            RCLCPP_WARN(get_logger(), "avcodec_send_packet error: %s",
                av_error_string(ret).c_str());
            return;
        }

        // A single packet may produce 0 or more decoded frames
        while (true) {
            ret = avcodec_receive_frame(codec_ctx_, frame_);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                break;
            if (ret < 0) {
                RCLCPP_WARN(get_logger(), "avcodec_receive_frame error: %s",
                    av_error_string(ret).c_str());
                break;
            }
            publish_frame(msg->header);
            av_frame_unref(frame_);
        }
    }

    void publish_frame(const std_msgs::msg::Header & src_header)
    {
        const int w = frame_->width;
        const int h = frame_->height;

        // ---- Ensure BGR frame buffer is large enough ---------------------
        const int needed = av_image_get_buffer_size(AV_PIX_FMT_BGR24, w, h, 1);
        if (bgr_buffer_size_ < needed) {
            av_frame_unref(frame_bgr_);
            frame_bgr_->format = AV_PIX_FMT_BGR24;
            frame_bgr_->width  = w;
            frame_bgr_->height = h;
            if (av_frame_get_buffer(frame_bgr_, 1) < 0) {
                RCLCPP_ERROR(get_logger(), "av_frame_get_buffer failed");
                return;
            }
            bgr_buffer_size_ = needed;
        }

        // ---- Colour-space conversion -------------------------------------
        sws_ctx_ = sws_getCachedContext(
            sws_ctx_,
            w, h, static_cast<AVPixelFormat>(frame_->format),
            w, h, AV_PIX_FMT_BGR24,
            SWS_BILINEAR, nullptr, nullptr, nullptr);

        if (!sws_ctx_) {
            RCLCPP_ERROR(get_logger(), "sws_getCachedContext failed");
            return;
        }

        sws_scale(
            sws_ctx_,
            frame_->data,     frame_->linesize,     0, h,
            frame_bgr_->data, frame_bgr_->linesize);

        // ---- Build ROS message -------------------------------------------
        // FFmpeg may add row padding so linesize[0] >= width*3.
        // We copy row-by-row to strip padding and guarantee step == width*3,
        // which is what most ROS consumers (cv_bridge, RViz) expect.
        auto out         = sensor_msgs::msg::Image();
        out.header       = src_header;   // preserve hardware timestamp
        out.header.frame_id = frame_id_;
        out.height       = static_cast<uint32_t>(h);
        out.width        = static_cast<uint32_t>(w);
        out.encoding     = "bgr8";
        out.is_bigendian = false;
        out.step         = static_cast<uint32_t>(w * 3);
        out.data.resize(h * w * 3);

        const int row_bytes = w * 3;
        for (int row = 0; row < h; ++row) {
            std::memcpy(
                out.data.data() + row * row_bytes,
                frame_bgr_->data[0] + row * frame_bgr_->linesize[0],
                row_bytes);
        }

        pub_->publish(out);
    }

    // FFmpeg state
    AVCodecContext * codec_ctx_;
    SwsContext *     sws_ctx_;
    AVFrame *        frame_;           // decoded YUV frame (reused per callback)
    AVFrame *        frame_bgr_;       // converted BGR frame (reused per callback)
    int              bgr_buffer_size_; // tracks allocated size to detect resize need

    // ROS state
    std::string frame_id_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxlH264Decoder>());
    rclcpp::shutdown();
    return 0;
}