#include "ircam/ircam_h264_node.hpp"

namespace ir_v4l2_camera {

// Encoder
void H264Encoder::init(int width, int height, AVPixelFormat src_fmt, int bitrate, const std::string& preset) {
    if (codec_ctx_ && width == width_ && height == height_ && src_fmt == src_fmt_)
        return;  // already set up for this geometry

    shutdown();
    
    width_ = width;
    height_ = height;
    src_fmt_ = src_fmt;

    // Chose encoder from software
    const char* enc_name = "libx264";
    const AVCodec* codec = avcodec_find_encoder_by_name(enc_name);
    AVCodecContext* ctx = avcodec_alloc_context3(codec);

    ctx->width = width;
    ctx->height = height;
    ctx->pix_fmt = AV_PIX_FMT_YUV420P;
    ctx->bit_rate = bitrate;
    ctx->gop_size = 30;
    ctx->max_b_frames = 0;
    ctx->time_base = AVRational{1, 30};
    ctx->framerate = AVRational{30, 1};

    av_opt_set(ctx->priv_data, "preset", preset.c_str(), 0);
    av_opt_set(ctx->priv_data, "tune", "zerolatency", 0);

    if (avcodec_open2(ctx, codec, nullptr) < 0) {
        avcodec_free_context(&ctx);
        throw std::runtime_error("avcodec_open2 failed");
    }

    codec_ctx_ = ctx;

    if (!codec_ctx_) throw std::runtime_error("No H.264 encoder could be opened – install libx264 or enable a HW codec");

    encoder_name_ = codec->name;

    // Allocate frame & scaler
    frame_ = av_frame_alloc();
    frame_->format = AV_PIX_FMT_YUV420P;
    frame_->width = width;
    frame_->height = height;
    av_frame_get_buffer(frame_, 0);

    sws_ = sws_getContext(width, height, src_fmt, width, height, AV_PIX_FMT_YUV420P, SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);

    if (!sws_) throw std::runtime_error("sws_getContext failed");

    pts_ = 0; 
}

std::vector<uint8_t> H264Encoder::encode(const uint8_t* raw_data, int step) {
    const uint8_t* src_slices[1] = { raw_data };
    int src_stride[1] = { step };

    av_frame_make_writable(frame_);
    sws_scale(sws_, src_slices, src_stride, 0, height_, frame_->data, frame_->linesize);

    frame_->pts = pts_++;

    // Send frame to encoder
    int ret = avcodec_send_frame(codec_ctx_, frame_);
    if (ret < 0) throw std::runtime_error("avcodec_send_frame failed");

    // Drain all available packets
    std::vector<uint8_t> out;
    AVPacket* pkt = av_packet_alloc();
    while (avcodec_receive_packet(codec_ctx_, pkt) == 0) {
        out.insert(out.end(), pkt->data, pkt->data + pkt->size);
        av_packet_unref(pkt);
    }
    av_packet_free(&pkt);
    return out;
}

void H264Encoder::shutdown() {
    if (sws_) { sws_freeContext(sws_); sws_ = nullptr; }
    if (frame_) { av_frame_free(&frame_); frame_ = nullptr; }
    if (codec_ctx_) { avcodec_free_context(&codec_ctx_); codec_ctx_ = nullptr; }
}

// ROS2 Encoder Node republish
IrcamH264Republisher::IrcamH264Republisher(const rclcpp::NodeOptions& options) : Node("ircam_h264_republisher", options) {
    // ---- Declare parameters ------------------------------------------
    declare_parameter<int>("bitrate",    2'000'000);  // 2 Mbit/s default
    declare_parameter<std::string>("preset", "ultrafast");
    declare_parameter<std::string>("input_topic",  "/ircam/raw_image");
    declare_parameter<std::string>("output_topic", "/ircam/h264");

    bitrate_    = get_parameter("bitrate").as_int();
    preset_     = get_parameter("preset").as_string();
    auto in_topic  = get_parameter("input_topic").as_string();
    auto out_topic = get_parameter("output_topic").as_string();

    RCLCPP_INFO(get_logger(),
        "Encoding %s → %s  [bitrate=%d, preset=%s]",
        in_topic.c_str(), out_topic.c_str(), bitrate_, preset_.c_str());

    image_raw_sub_ = create_subscription<sensor_msgs::msg::Image>(
        in_topic, rclcpp::SensorDataQoS(),
        std::bind(&IrcamH264Republisher::image_callback, this, std::placeholders::_1));

        auto pub_qos = rclcpp::QoS(30).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::Volatile);
    // auto pub_qos = rclcpp::QoS(30).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::TransientLocal);
    // auto pub_qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort).durability(rclcpp::DurabilityPolicy::Volatile);
    image_h264_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(out_topic, pub_qos);
}

void IrcamH264Republisher::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    AVPixelFormat src_fmt = ros_encoding_to_av(msg->encoding);
    if (src_fmt == AV_PIX_FMT_NONE) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Unsupported encoding '%s' – skipping frame", msg->encoding.c_str());
        return;
    }

    // init (will only do once)
    try {
        encoder_.init(msg->width, msg->height, src_fmt, bitrate_, preset_);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Encoder init failed: %s", e.what());
        return;
    }

    if (!logged_encoder_) {
        RCLCPP_INFO(get_logger(), "Using encoder: %s  (%dx%d, input=%s)", encoder_.encoder_name().c_str(), msg->width, msg->height, msg->encoding.c_str());
        logged_encoder_ = true;
    }

    // Encode
    std::vector<uint8_t> nal;
    try {
        nal = encoder_.encode(msg->data.data(), msg->step);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Encode failed: %s", e.what());
        return;
    }

    // Publish
    auto out = std::make_unique<sensor_msgs::msg::CompressedImage>();
    out->header = msg->header;
    out->format = "h264";
    out->data = std::move(nal);
    image_h264_pub_->publish(std::move(out));
}

} // namespace ir_v4l2_camera