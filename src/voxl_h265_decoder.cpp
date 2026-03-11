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
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <chrono>
#include <cstdarg>
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
// Custom FFmpeg log callback.
//
// The HEVC decoder sometimes emits error messages (e.g. "Could not find ref
// with POC X") without setting decode_error_flags on the output frame,
// producing silently garbled output.  We intercept those messages here and
// set a thread_local flag so the callback can drop the affected frame.
//
// thread_local ensures multiple decoder nodes in the same process don't
// interfere with each other.
// ---------------------------------------------------------------------------
static thread_local bool g_ffmpeg_decode_error = false;

static void ffmpeg_log_callback(void * /*avcl*/, int level, const char * fmt, va_list vl)
{
    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, vl);

    if (level <= AV_LOG_WARNING) {
        if (strstr(buf, "Could not find ref")       ||
            strstr(buf, "decode_slice_header error") ||
            strstr(buf, "no frame!")                ||
            strstr(buf, "PPS id out of range")      ||
            strstr(buf, "non-existing PPS"))
        {
            g_ffmpeg_decode_error = true;
            return;   // suppress — counted in stats, no need to print
        }
    }

    // Pass everything else through to the default handler.
    av_log_default_callback(nullptr, level, fmt, vl);
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
        // avcodec_register_all() is required in FFmpeg < 4.0 to register
        // all built-in codecs before avcodec_find_decoder() can find them.
        // It was deprecated in 4.0 (codecs auto-register) and removed later,
        // so we guard it with a version check for cross-version compatibility.
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(58, 0, 0)
        avcodec_register_all();
#endif

        const AVCodec * codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
        if (!codec)
            throw std::runtime_error("avcodec_find_decoder failed for HEVC");

        codec_ctx_ = avcodec_alloc_context3(codec);
        if (!codec_ctx_)
            throw std::runtime_error("avcodec_alloc_context3 failed");

        if (avcodec_open2(codec_ctx_, codec, nullptr) < 0)
            throw std::runtime_error("avcodec_open2 failed");

        // Promote concealed frames to flagged errors where possible.
        codec_ctx_->err_recognition |= AV_EF_CRCCHECK | AV_EF_BITSTREAM | AV_EF_BUFFER;

        // Intercept FFmpeg log messages for corruption that decode_error_flags misses.
        av_log_set_callback(ffmpeg_log_callback);

        frame_     = av_frame_alloc();
        frame_bgr_ = av_frame_alloc();
        if (!frame_ || !frame_bgr_)
            throw std::runtime_error("av_frame_alloc failed");

        // ---- ROS setup ---------------------------------------------------
        pub_ = create_publisher<sensor_msgs::msg::Image>(out_topic, 10);

        // RELIABLE + TRANSIENT_LOCAL guarantees the publisher replays its
        // cached VPS+SPS+PPS to this subscriber on connection, regardless of
        // startup order.  BEST_EFFORT + VOLATILE does not guarantee this
        // replay, causing intermittent "waiting for parameter sets" failures.
        //
        // Backlog is kept in check by depth=1 and a staleness check in the
        // callback that discards burst packets.
        rmw_qos_profile_t qos_profile  = rmw_qos_profile_default;
        qos_profile.reliability        = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        qos_profile.durability         = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        qos_profile.depth              = 1;
        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile);

        sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            in_topic, qos,
            std::bind(&VoxlHevcDecoder::callback, this, std::placeholders::_1));

        // Watchdog fires once after 5s to catch startup issues.
        watchdog_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&VoxlHevcDecoder::watchdog, this));

        // Periodic stats summary every 30s.
        stats_timer_ = create_wall_timer(
            std::chrono::seconds(30),
            std::bind(&VoxlHevcDecoder::print_stats, this));

        RCLCPP_INFO(get_logger(), "HEVC decoder started  |  %s → %s",
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
    // -----------------------------------------------------------------------
    void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        const auto & fmt = msg->format;
        if (fmt != "h265" && fmt != "H265" && fmt != "hevc" && fmt != "HEVC") {
            RCLCPP_WARN_ONCE(get_logger(),
                "Unexpected format '%s' — expected h265/hevc. "
                "Check voxl-mpa-to-ros2 configuration.", fmt.c_str());
            return;
        }

        // Cache packets containing VPS/SPS/PPS so they can be replayed after
        // a decoder flush.  VOXL sends parameter sets once at startup only.
        // Packets that bundle VPS+SPS+PPS with an IDR are also cached — the
        // IDR replay after a flush is drained immediately and is harmless.
        if (contains_parameter_sets(msg->data))
            param_set_cache_ = msg->data;

        packets_received_++;

        // Backlog detection using a rolling average of inter-arrival time.
        // At 30fps normal cadence is ~33ms per packet.  When a RELIABLE
        // middleware queue drains a backlog, packets arrive significantly
        // faster than the rolling average — we discard these to stay real-time.
        const int64_t now_ns = std::chrono::steady_clock::now()
                                   .time_since_epoch().count();
        const int64_t gap_ms = (last_recv_ns_ == 0) ? 33
                               : (now_ns - last_recv_ns_) / 1'000'000LL;
        last_recv_ns_ = now_ns;

        avg_gap_ms_ = (avg_gap_ms_ < 0.0)
            ? static_cast<double>(gap_ms)
            : avg_gap_ms_ * 0.9 + gap_ms * 0.1;

        if (packets_received_ > 10 && avg_gap_ms_ > 10.0
            && gap_ms < avg_gap_ms_ * 0.5)
        {
            frames_dropped_++;
            return;   // discard backlog packet silently
        }

        // Clear the FFmpeg log error flag before each packet so it only
        // reflects errors from decoding this specific packet.
        g_ffmpeg_decode_error = false;

        AVPacket pkt{};
        pkt.data = const_cast<uint8_t *>(msg->data.data());
        pkt.size = static_cast<int>(msg->data.size());

        int ret = avcodec_send_packet(codec_ctx_, &pkt);
        if (ret < 0) {
            packets_failed_++;
            if (++consecutive_errors_ >= kMaxConsecutiveErrors)
                flush_and_restore_params();
            return;
        }
        consecutive_errors_ = 0;

        while (true) {
            ret = avcodec_receive_frame(codec_ctx_, frame_);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) break;
            if (ret < 0) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "avcodec_receive_frame error: %s", av_error_string(ret).c_str());
                break;
            }

            if (frame_->pict_type == AV_PICTURE_TYPE_I)
                got_keyframe_ = true;

            // Drop frame if decode_error_flags is set or the FFmpeg log
            // callback detected a reference error for this packet.
            const bool corrupt = (frame_->decode_error_flags != 0) || g_ffmpeg_decode_error;
            g_ffmpeg_decode_error = false;

            if (corrupt) {
                frames_dropped_++;
                got_keyframe_ = false;
                av_frame_unref(frame_);

                if (++consecutive_corrupt_ >= kMaxConsecutiveCorrupt)
                    flush_and_restore_params();
                continue;
            }
            consecutive_corrupt_ = 0;

            if (!got_keyframe_) {
                av_frame_unref(frame_);
                continue;
            }

            publish_frame(msg->header);
            frames_published_++;
            av_frame_unref(frame_);
        }
    }

    // -----------------------------------------------------------------------
    void watchdog()
    {
        watchdog_timer_->cancel();

        if (packets_received_ == 0) {
            RCLCPP_ERROR(get_logger(),
                "No packets received after 5s — check that voxl-mpa-to-ros2 "
                "is running and the input topic name is correct");
            return;
        }

        if (frames_published_ == 0) {
            RCLCPP_ERROR(get_logger(),
                "Received %d packets but published 0 frames — "
                "VPS+SPS+PPS was likely missed at startup. "
                "Restart voxl-mpa-to-ros2 to recover.",
                packets_received_);
        }
    }

    // -----------------------------------------------------------------------
    // Prints a one-line stats summary and resets counters for the next window.
    // -----------------------------------------------------------------------
    void print_stats()
    {
        const int drop_pct = packets_received_ > 0
            ? (frames_dropped_ * 100) / packets_received_ : 0;

        RCLCPP_INFO(get_logger(),
            "[ 30s stats ]  received: %d  published: %d  "
            "dropped: %d (%d%%)  send_errors: %d  flushes: %d",
            packets_received_, frames_published_,
            frames_dropped_,   drop_pct,
            packets_failed_,   flush_count_);

        packets_received_ = 0;
        frames_published_ = 0;
        frames_dropped_   = 0;
        packets_failed_   = 0;
        flush_count_      = 0;
    }

    // -----------------------------------------------------------------------
    // Returns true if the packet contains at least one HEVC parameter set NAL:
    // VPS (32), SPS (33), or PPS (34).  Returns true even if the packet also
    // contains other NAL types (e.g. an IDR frame).
    // -----------------------------------------------------------------------
    static bool contains_parameter_sets(const std::vector<uint8_t> & data)
    {
        size_t i = 0;
        while (i < data.size()) {
            size_t sc_len = 0;
            if (i + 3 < data.size() && data[i]==0 && data[i+1]==0 && data[i+2]==1)
                sc_len = 3;
            else if (i + 4 < data.size() && data[i]==0 && data[i+1]==0 && data[i+2]==0 && data[i+3]==1)
                sc_len = 4;
            else { ++i; continue; }

            i += sc_len;
            if (i >= data.size()) break;

            const uint8_t nal = (data[i] >> 1) & 0x3F;   // HEVC NAL type
            if (nal == 32 || nal == 33 || nal == 34)       // VPS / SPS / PPS
                return true;
            ++i;
        }
        return false;
    }

    // -----------------------------------------------------------------------
    // Flush the decoder and replay cached VPS+SPS+PPS so it is immediately
    // ready to decode the next IDR without erroring.
    // -----------------------------------------------------------------------
    void flush_and_restore_params()
    {
        avcodec_flush_buffers(codec_ctx_);
        got_keyframe_        = false;
        consecutive_errors_  = 0;
        consecutive_corrupt_ = 0;
        flush_count_++;

        if (param_set_cache_.empty()) return;

        AVPacket ps_pkt{};
        ps_pkt.data = param_set_cache_.data();
        ps_pkt.size = static_cast<int>(param_set_cache_.size());

        if (avcodec_send_packet(codec_ctx_, &ps_pkt) < 0) return;

        // Drain any frames produced by the parameter set replay.
        AVFrame * tmp = av_frame_alloc();
        while (tmp && avcodec_receive_frame(codec_ctx_, tmp) == 0)
            av_frame_unref(tmp);
        av_frame_free(&tmp);
    }

    // -----------------------------------------------------------------------
    void publish_frame(const std_msgs::msg::Header & src_header)
    {
        const int w = frame_->width;
        const int h = frame_->height;

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

        sws_ctx_ = sws_getCachedContext(
            sws_ctx_,
            w, h, static_cast<AVPixelFormat>(frame_->format),
            w, h, AV_PIX_FMT_BGR24,
            SWS_BILINEAR, nullptr, nullptr, nullptr);

        if (!sws_ctx_) {
            RCLCPP_ERROR(get_logger(), "sws_getCachedContext failed");
            return;
        }

        sws_scale(sws_ctx_,
            frame_->data,     frame_->linesize,     0, h,
            frame_bgr_->data, frame_bgr_->linesize);

        auto out            = sensor_msgs::msg::Image();
        out.header          = src_header;
        out.header.frame_id = frame_id_;
        out.height          = static_cast<uint32_t>(h);
        out.width           = static_cast<uint32_t>(w);
        out.encoding        = "bgr8";
        out.is_bigendian    = false;
        out.step            = static_cast<uint32_t>(w * 3);
        out.data.resize(h * w * 3);

        const int row_bytes = w * 3;
        for (int row = 0; row < h; ++row)
            std::memcpy(out.data.data() + row * row_bytes,
                        frame_bgr_->data[0] + row * frame_bgr_->linesize[0],
                        row_bytes);

        pub_->publish(out);
    }

    // ---- FFmpeg state ------------------------------------------------------
    AVCodecContext * codec_ctx_;
    SwsContext *     sws_ctx_;
    AVFrame *        frame_;        // decoded YUV frame,   reused per callback
    AVFrame *        frame_bgr_;    // converted BGR frame, reused per callback
    int              bgr_buffer_size_;

    // ---- Decoder state -----------------------------------------------------
    bool got_keyframe_        {false};
    int  consecutive_errors_  {0};
    int  consecutive_corrupt_ {0};
    int64_t last_recv_ns_     {0};      // steady_clock time of last packet
    double  avg_gap_ms_       {-1.0};   // rolling average inter-arrival (ms)
    static constexpr int kMaxConsecutiveErrors  {5};
    static constexpr int kMaxConsecutiveCorrupt {4};   // ~133ms at 30fps

    // ---- Stats (reset every 30s by print_stats) ----------------------------
    int packets_received_ {0};
    int frames_published_ {0};
    int frames_dropped_   {0};
    int packets_failed_   {0};
    int flush_count_      {0};

    // ---- Parameter set cache -----------------------------------------------
    // Replayed after any flush so the decoder doesn't fail on the next IDR.
    std::vector<uint8_t> param_set_cache_;

    // ---- ROS state ---------------------------------------------------------
    std::string frame_id_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxlHevcDecoder>());
    rclcpp::shutdown();
    return 0;
}