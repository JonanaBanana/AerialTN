// voxl_h264_decoder.cpp
//
// Subscribes to a voxl-mpa-to-ros2 H264 encoded image topic
// (sensor_msgs/CompressedImage with format="h264") and republishes as
// a standard BGR8 sensor_msgs/Image using libavcodec.
//
// Build deps (CMakeLists.txt):
//   find_package(PkgConfig REQUIRED)
//   pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
//     libavcodec libavutil libswscale)
//   target_link_libraries(<target> PkgConfig::LIBAV)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

// ---------------------------------------------------------------------------
// av_err2str is a C99 compound-literal macro that doesn't compile in C++.
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
    : Node("voxl_decoder"),
      codec_ctx_(nullptr),
      sws_ctx_(nullptr),
      frame_(nullptr)
    {
        declare_parameter("input_topic",  "/hires_small_encoded");
        declare_parameter("output_topic", "/hires_small_decoded");
        declare_parameter("frame_id",     "hires_small");

        const auto in_topic  = get_parameter("input_topic").as_string();
        const auto out_topic = get_parameter("output_topic").as_string();
        frame_id_            = get_parameter("frame_id").as_string();

        // ---- FFmpeg setup ------------------------------------------------
        const AVCodec * codec = avcodec_find_decoder(AV_CODEC_ID_H264);
        if (!codec)
            throw std::runtime_error("avcodec_find_decoder failed for H264");

        codec_ctx_ = avcodec_alloc_context3(codec);
        if (!codec_ctx_)
            throw std::runtime_error("avcodec_alloc_context3 failed");

        if (avcodec_open2(codec_ctx_, codec, nullptr) < 0)
            throw std::runtime_error("avcodec_open2 failed");

        frame_ = av_frame_alloc();
        if (!frame_)
            throw std::runtime_error("av_frame_alloc failed");

        // ---- ROS setup ---------------------------------------------------
        pub_ = create_publisher<sensor_msgs::msg::Image>(out_topic, 10);

        // Match voxl-mpa-to-ros2's publisher QoS exactly — it publishes with
        // VOLATILE + BestEffort.  A Reliable subscriber is incompatible and
        // silently receives nothing, which causes the startup race symptoms.
        auto qos = rclcpp::QoS(30)
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::TransientLocal);

        sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            in_topic, qos,
            std::bind(&VoxlH264Decoder::callback, this, std::placeholders::_1));

        // Watchdog: fires once after 5s to catch the case where the decoder
        // node started after voxl-mpa-to-ros2 and missed the SPS+PPS packet.
        watchdog_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&VoxlH264Decoder::watchdog, this));

        RCLCPP_INFO(get_logger(), "Decoding %s  →  %s  [h264]",
            in_topic.c_str(), out_topic.c_str());
    }

    ~VoxlH264Decoder()
    {
        if (sws_ctx_)   sws_freeContext(sws_ctx_);
        if (frame_)     av_frame_free(&frame_);
        if (codec_ctx_) avcodec_free_context(&codec_ctx_);
    }

private:
    void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        const auto & fmt = msg->format;
        if (fmt != "h264" && fmt != "H264") {
            RCLCPP_WARN_ONCE(get_logger(),
                "Unexpected format '%s' — expected h264", fmt.c_str());
            return;
        }

        // Cache the SPS+PPS parameter set packet so it can be replayed after
        // any decoder flush.  Without this, H264 fails with "non-existing PPS
        // 0 referenced" after a flush because VOXL only sends SPS+PPS once at
        // stream startup — identical problem to HEVC's "PPS id out of range".
        if (contains_parameter_sets(msg->data)) {
            param_set_cache_ = msg->data;
            RCLCPP_DEBUG(get_logger(),
                "Cached %zu-byte SPS+PPS packet", param_set_cache_.size());
        }

        packets_received_++;

        // Stack-allocated AVPacket pointing directly at the ROS buffer.
        // avcodec_send_packet() treats data as read-only so the const_cast
        // is safe.  No heap allocation or free needed.
        AVPacket pkt{};
        pkt.data = const_cast<uint8_t *>(msg->data.data());
        pkt.size = static_cast<int>(msg->data.size());

        int ret = avcodec_send_packet(codec_ctx_, &pkt);
        if (ret < 0) {
            RCLCPP_WARN(get_logger(), "avcodec_send_packet error: %s",
                av_error_string(ret).c_str());

            // After repeated failures flush and restore SPS+PPS so the decoder
            // can resync cleanly on the next I-frame rather than looping on
            // "non-existing PPS" errors forever.
            if (++consecutive_errors_ >= kMaxConsecutiveErrors) {
                RCLCPP_WARN(get_logger(),
                    "Too many consecutive errors — flushing decoder and "
                    "restoring SPS+PPS");
                flush_and_restore_params();
            }
            return;
        }
        consecutive_errors_ = 0;

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

            // Don't publish until we have a clean I-frame anchor.
            if (frame_->pict_type == AV_PICTURE_TYPE_I)
                got_keyframe_ = true;

            // Drop frames FFmpeg flagged as corrupt (missing reference etc.)
            if (frame_->decode_error_flags != 0) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Corrupt frame (flags=0x%x) — waiting for next I-frame",
                    frame_->decode_error_flags);
                got_keyframe_ = false;
                av_frame_unref(frame_);
                continue;
            }

            if (!got_keyframe_) {
                RCLCPP_DEBUG(get_logger(), "Dropping frame — no I-frame yet");
                av_frame_unref(frame_);
                continue;
            }

            publish_frame(msg->header);
            frames_decoded_++;
            av_frame_unref(frame_);
        }
    }

    // -----------------------------------------------------------------------
    // Returns true if the packet contains at least one H264 parameter set NAL:
    //   SPS (type 7) or PPS (type 8)
    //
    // Intentionally returns true even if the packet also contains other NALs
    // (e.g. an IDR frame).  VOXL frequently bundles SPS+PPS together with the
    // first IDR in a single packet.  The old approach (returning false if ANY
    // non-param NAL was present) caused param_set_cache_ to stay empty in that
    // case, making flush recovery impossible.
    //
    // Replaying a cached packet that includes an IDR frame is harmless — the
    // decoder decodes it and flush_and_restore_params() drains the output.
    // -----------------------------------------------------------------------
    static bool contains_parameter_sets(const std::vector<uint8_t> & data)
    {
        size_t i = 0;
        while (i < data.size()) {
            // Find Annex-B start code: 0x000001 or 0x00000001
            size_t sc_len = 0;
            if (i + 3 < data.size() &&
                data[i]==0 && data[i+1]==0 && data[i+2]==1)
                sc_len = 3;
            else if (i + 4 < data.size() &&
                data[i]==0 && data[i+1]==0 && data[i+2]==0 && data[i+3]==1)
                sc_len = 4;
            else { ++i; continue; }

            i += sc_len;
            if (i >= data.size()) break;

            // H264 NAL type: bits[0..4] of first byte — SPS=7, PPS=8
            const uint8_t nal = data[i] & 0x1F;
            if (nal == 7 || nal == 8)
                return true;   // found at least one param set NAL

            ++i;
        }
        return false;
    }

    // Flush the decoder and immediately replay the cached SPS+PPS so the
    // decoder is ready to accept the next I-frame without erroring.
    void flush_and_restore_params()
    {
        avcodec_flush_buffers(codec_ctx_);
        got_keyframe_       = false;
        consecutive_errors_ = 0;

        if (param_set_cache_.empty()) {
            RCLCPP_WARN(get_logger(),
                "Flushed decoder but no SPS+PPS cached yet — "
                "stream will recover on next I-frame");
            return;
        }

        AVPacket ps_pkt{};
        ps_pkt.data = param_set_cache_.data();
        ps_pkt.size = static_cast<int>(param_set_cache_.size());

        if (avcodec_send_packet(codec_ctx_, &ps_pkt) < 0) {
            RCLCPP_WARN(get_logger(), "Failed to replay SPS+PPS after flush");
        } else {
            RCLCPP_DEBUG(get_logger(),
                "Replayed %zu-byte SPS+PPS after flush",
                param_set_cache_.size());
            // Drain any spurious output triggered by the replay
            AVFrame * tmp = av_frame_alloc();
            while (tmp && avcodec_receive_frame(codec_ctx_, tmp) == 0)
                av_frame_unref(tmp);
            av_frame_free(&tmp);
        }
    }

    void watchdog()
    {
        watchdog_timer_->cancel();

        if (packets_received_ == 0) {
            RCLCPP_WARN(get_logger(),
                "No packets received after 5s — check input topic name "
                "and that voxl-mpa-to-ros2 is running");
            return;
        }

        if (frames_decoded_ == 0) {
            RCLCPP_ERROR(get_logger(),
                "Received %d packets but decoded 0 frames — the SPS+PPS "
                "parameter set packet was probably published before this node "
                "subscribed. Fix: restart voxl-mpa-to-ros2 after the decoder "
                "is running, or lower small_venc_nPframes in voxl-camera-server "
                "config so a fresh I-frame arrives sooner.",
                packets_received_);
        }
    }

    void publish_frame(const std_msgs::msg::Header & src_header)
    {
        const int w = frame_->width;
        const int h = frame_->height;
        const int row_bytes = w * 3;
        const size_t needed = h * row_bytes;

        // Grow output buffer only when frame size increases.
        // Reused across frames — no allocation per callback.
        if (output_buffer_.size() < needed)
            output_buffer_.resize(needed);

        // Update sws context if frame dimensions or pixel format changed
        const auto src_fmt = static_cast<AVPixelFormat>(frame_->format);
        sws_ctx_ = sws_getCachedContext(
            sws_ctx_,
            w, h, src_fmt,
            w, h, AV_PIX_FMT_BGR24,
            SWS_BILINEAR, nullptr, nullptr, nullptr);

        if (!sws_ctx_) {
            RCLCPP_ERROR(get_logger(), "sws_getCachedContext failed");
            return;
        }

        // Scale directly into output_buffer_ — no intermediate frame needed.
        // av_image_fill_arrays sets dst pointers so sws_scale writes packed
        // BGR24 (step == width*3, no padding) straight into the buffer.
        uint8_t * dst_data[4]     = {};
        int       dst_linesize[4] = {};
        av_image_fill_arrays(
            dst_data, dst_linesize,
            output_buffer_.data(),
            AV_PIX_FMT_BGR24, w, h, 1);

        sws_scale(
            sws_ctx_,
            frame_->data, frame_->linesize, 0, h,
            dst_data,     dst_linesize);

        // Move output_buffer_ into the message — zero-copy into ROS.
        // Moved back out immediately after publish so it can be reused.
        auto out         = sensor_msgs::msg::Image();
        out.header       = src_header;
        out.header.frame_id = frame_id_;
        out.height       = static_cast<uint32_t>(h);
        out.width        = static_cast<uint32_t>(w);
        out.encoding     = "bgr8";
        out.is_bigendian = false;
        out.step         = static_cast<uint32_t>(row_bytes);
        out.data         = std::move(output_buffer_);

        pub_->publish(out);

        output_buffer_ = std::move(out.data);   // reclaim buffer after publish
    }

    // FFmpeg state
    AVCodecContext * codec_ctx_;
    SwsContext *     sws_ctx_;
    AVFrame *        frame_;       // decoded YUV frame (reused per callback)

    // Persistent BGR output buffer — sws_scale writes directly into this,
    // eliminating the intermediate AVFrame and row-by-row memcpy.
    std::vector<uint8_t> output_buffer_;

    // Cached SPS+PPS packet — replayed after any flush so the decoder doesn't
    // fail with "non-existing PPS 0 referenced" on the next I-frame.
    // VOXL only sends SPS+PPS once at startup so without this a flush
    // permanently breaks decoding.
    std::vector<uint8_t> param_set_cache_;

    // Decoder state
    bool got_keyframe_       {false};
    int  consecutive_errors_ {0};
    int  packets_received_   {0};
    int  frames_decoded_     {0};
    static constexpr int kMaxConsecutiveErrors {5};

    // ROS state
    std::string frame_id_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              pub_;
    rclcpp::TimerBase::SharedPtr                                       watchdog_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxlH264Decoder>());
    rclcpp::shutdown();
    return 0;
}