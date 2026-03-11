// voxl_hevc_decoder.cpp
//
// Subscribes to a voxl-mpa-to-ros2 HEVC encoded image topic
// (sensor_msgs/CompressedImage with format="h265") and republishes
// as a standard BGR8 sensor_msgs/Image using libavcodec.
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

class VoxlHevcDecoder : public rclcpp::Node
{
public:
    VoxlHevcDecoder()
    : Node("voxl_hevc_decoder"),
      codec_ctx_(nullptr),
      sws_ctx_(nullptr),
      frame_(nullptr),
      frame_bgr_(nullptr),
      bgr_buffer_size_(0)
    {
        declare_parameter("input_topic",  "/hires_small_encoded");
        declare_parameter("output_topic", "/hires_small_decoded");
        declare_parameter("frame_id",     "hires_small");

        const auto in_topic  = get_parameter("input_topic").as_string();
        const auto out_topic = get_parameter("output_topic").as_string();
        frame_id_            = get_parameter("frame_id").as_string();

        // ---- FFmpeg setup ------------------------------------------------
        const AVCodec * codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
        if (!codec)
            throw std::runtime_error("avcodec_find_decoder failed for HEVC");

        codec_ctx_ = avcodec_alloc_context3(codec);
        if (!codec_ctx_)
            throw std::runtime_error("avcodec_alloc_context3 failed");

        if (avcodec_open2(codec_ctx_, codec, nullptr) < 0)
            throw std::runtime_error("avcodec_open2 failed");

        frame_     = av_frame_alloc();
        frame_bgr_ = av_frame_alloc();
        if (!frame_ || !frame_bgr_)
            throw std::runtime_error("av_frame_alloc failed");

        // ---- ROS setup ---------------------------------------------------
        pub_ = create_publisher<sensor_msgs::msg::Image>(out_topic, 10);

        // Use VOLATILE + BestEffort to match voxl-mpa-to-ros2's publisher QoS.
        // TRANSIENT_LOCAL would be ideal to catch the VPS+SPS+PPS packet when
        // subscribing late, but voxl-mpa-to-ros2 uses VOLATILE so the two are
        // incompatible — no messages would arrive at all with TRANSIENT.
        // The startup race is instead handled by the watchdog below.
        auto qos = rclcpp::QoS(30)
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::TransientLocal);

        sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            in_topic, qos,
            std::bind(&VoxlHevcDecoder::callback, this, std::placeholders::_1));

        // Watchdog: fires once after 5s. If packets arrived but nothing decoded,
        // the VPS+SPS+PPS packet was missed at startup — print a clear fix.
        watchdog_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&VoxlHevcDecoder::watchdog, this));

        RCLCPP_INFO(get_logger(), "Decoding %s  →  %s  [hevc]",
            in_topic.c_str(), out_topic.c_str());
    }

    ~VoxlHevcDecoder()
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
        if (fmt != "h265" && fmt != "H265" && fmt != "hevc" && fmt != "HEVC") {
            RCLCPP_WARN_ONCE(get_logger(),
                "Unexpected format '%s' — expected h265/hevc", fmt.c_str());
            return;
        }

        // Cache any packet containing VPS/SPS/PPS so it can be replayed after
        // a decoder flush.  Without this, HEVC fails with "PPS id out of range"
        // after a flush because VOXL only sends parameter sets once at startup.
        //
        // Intentionally caches packets that also contain an IDR frame — VOXL
        // frequently bundles VPS+SPS+PPS together with the first IDR in a single
        // packet.  Replaying such a packet after a flush is harmless because
        // flush_and_restore_params() drains any decoded output frames produced.
        if (contains_parameter_sets(msg->data)) {
            param_set_cache_ = msg->data;
            RCLCPP_DEBUG(get_logger(),
                "Cached %zu-byte parameter set packet",
                param_set_cache_.size());
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

            // After repeated failures flush and restore parameter sets so the
            // decoder can resync cleanly on the next IDR frame.
            if (++consecutive_errors_ >= kMaxConsecutiveErrors) {
                RCLCPP_WARN(get_logger(),
                    "Too many consecutive errors — flushing decoder and "
                    "restoring VPS+SPS+PPS");
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
            // Suppresses corrupt output when joining mid-stream.
            if (frame_->pict_type == AV_PICTURE_TYPE_I)
                got_keyframe_ = true;

            // FFmpeg sets decode_error_flags when it detected bitstream errors
            // (e.g. a P-frame whose reference frame was missing — "Could not
            // find ref with POC X").  Count these: if they run long it means
            // the reference chain is broken and we should flush proactively
            // rather than waiting passively for the next natural IDR, which
            // could be many seconds away.
            if (frame_->decode_error_flags != 0) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Corrupt frame (flags=0x%x) — waiting for next I-frame",
                    frame_->decode_error_flags);
                got_keyframe_ = false;
                av_frame_unref(frame_);

                if (++consecutive_corrupt_ >= kMaxConsecutiveCorrupt) {
                    RCLCPP_WARN(get_logger(),
                        "%d consecutive corrupt frames — flushing decoder "
                        "and restoring VPS+SPS+PPS to resync faster",
                        kMaxConsecutiveCorrupt);
                    flush_and_restore_params();
                }
                continue;
            }
            consecutive_corrupt_ = 0;

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

    void watchdog()
    {
        watchdog_timer_->cancel();   // fire once only

        if (packets_received_ == 0) {
            RCLCPP_WARN(get_logger(),
                "No packets received after 5s — check input topic name "
                "and that voxl-mpa-to-ros2 is running");
            return;
        }

        if (frames_decoded_ == 0) {
            RCLCPP_ERROR(get_logger(),
                "Received %d packets but decoded 0 frames — the VPS+SPS+PPS "
                "parameter set packet was probably published before this node "
                "subscribed. "
                "Fix: restart voxl-mpa-to-ros2 AFTER the decoder is running, "
                "or lower small_venc_nPframes in voxl-camera-server config "
                "so a fresh I-frame arrives sooner.",
                packets_received_);
        }
    }

    // -----------------------------------------------------------------------
    // Returns true if the packet contains at least one HEVC parameter set NAL:
    //   VPS (type 32), SPS (type 33), PPS (type 34)
    //
    // Intentionally returns true even if the packet also contains other NALs
    // (e.g. an IDR frame).  VOXL frequently bundles VPS+SPS+PPS together with
    // the first IDR in a single packet.  The old approach (returning false if
    // ANY non-param NAL was present) caused param_set_cache_ to stay empty in
    // that case, making flush recovery impossible.
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

            // HEVC NAL type: bits[1..6] of the first byte of the two-byte
            // NAL unit header — VPS=32, SPS=33, PPS=34
            const uint8_t hevc_nal = (data[i] >> 1) & 0x3F;
            if (hevc_nal == 32 || hevc_nal == 33 || hevc_nal == 34)
                return true;   // found at least one parameter set NAL

            ++i;
        }
        return false;
    }

    // Flush the decoder and immediately replay the cached parameter set packet
    // so the decoder is ready to accept the next IDR without erroring.
    void flush_and_restore_params()
    {
        avcodec_flush_buffers(codec_ctx_);
        got_keyframe_        = false;
        consecutive_errors_  = 0;
        consecutive_corrupt_ = 0;

        if (param_set_cache_.empty()) {
            RCLCPP_WARN(get_logger(),
                "Flushed decoder but no VPS+SPS+PPS cached yet — "
                "stream will recover on next I-frame");
            return;
        }

        AVPacket ps_pkt{};
        ps_pkt.data = param_set_cache_.data();
        ps_pkt.size = static_cast<int>(param_set_cache_.size());

        if (avcodec_send_packet(codec_ctx_, &ps_pkt) < 0) {
            RCLCPP_WARN(get_logger(),
                "Failed to replay VPS+SPS+PPS after flush");
        } else {
            RCLCPP_DEBUG(get_logger(),
                "Replayed %zu-byte VPS+SPS+PPS after flush",
                param_set_cache_.size());
            // Drain any output frames produced by the replay
            AVFrame * tmp = av_frame_alloc();
            while (tmp && avcodec_receive_frame(codec_ctx_, tmp) == 0)
                av_frame_unref(tmp);
            av_frame_free(&tmp);
        }
    }

    void publish_frame(const std_msgs::msg::Header & src_header)
    {
        const int w = frame_->width;
        const int h = frame_->height;

        // Reallocate the BGR buffer only when the frame size changes
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

        // YUV → BGR colour-space conversion
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

        // Copy row-by-row to strip any FFmpeg row padding so that
        // step == width * 3, which ROS consumers (RViz, cv_bridge) expect.
        auto out         = sensor_msgs::msg::Image();
        out.header       = src_header;
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
    AVFrame *        frame_;           // decoded YUV frame  (reused per callback)
    AVFrame *        frame_bgr_;       // converted BGR frame (reused per callback)
    int              bgr_buffer_size_;

    // Decoder state
    bool got_keyframe_        {false};
    int  consecutive_errors_  {0};
    int  consecutive_corrupt_ {0};   // corrupt frames from broken ref chain
    int  packets_received_    {0};
    int  frames_decoded_      {0};
    static constexpr int kMaxConsecutiveErrors  {5};
    static constexpr int kMaxConsecutiveCorrupt {10};  // ~330ms at 30fps

    // Cached parameter set packet — replayed after any flush so the decoder
    // doesn't fail with "PPS id out of range" on the next IDR.
    std::vector<uint8_t> param_set_cache_;

    // ROS state
    std::string frame_id_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              pub_;
    rclcpp::TimerBase::SharedPtr                                       watchdog_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxlHevcDecoder>());
    rclcpp::shutdown();
    return 0;
}