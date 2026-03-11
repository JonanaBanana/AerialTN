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


/* 
When readuing YUV420p in OpenCV use this conversion:
cv::Mat yuv(h * 3 / 2, w, CV_8UC1, msg->data.data());
cv::Mat bgr;
cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_I420); 
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <chrono>
#include <cstdarg>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <rmw/qos_profiles.h>

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
// The H264 decoder sometimes emits error messages (e.g. "non-existing PPS 0
// referenced") without setting decode_error_flags on the output frame,
// producing silently garbled output.  We intercept those messages here and
// set a thread_local flag so the callback can drop the affected frame.
//
// thread_local ensures multiple decoder nodes in the same process don't
// interfere with each other.
// ---------------------------------------------------------------------------
static thread_local bool g_ffmpeg_decode_error = false;

static void ffmpeg_log_callback(void * /*avcl*/, int level, const char * fmt, va_list vl)
{
    // Gate on level before any string work — FFmpeg emits many INFO/DEBUG
    // messages internally; we only care about warnings and above.
    if (level > AV_LOG_WARNING) return;

    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, vl);

    if (strstr(buf, "non-existing PPS")         ||
        strstr(buf, "decode_slice_header error") ||
        strstr(buf, "no frame!")                ||
        strstr(buf, "reference picture missing") ||
        strstr(buf, "mmco: unref short failure"))
    {
        g_ffmpeg_decode_error = true;
        return;   // suppress — counted in stats, no need to print
    }

    av_log_default_callback(nullptr, level, fmt, vl);
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
        // Set live_stream:=true when receiving over WiFi to enable backlog
        // detection.  Disable for bag file playback or onboard processing.
        declare_parameter("live_stream",  true);
        // When false, publishes raw YUV420p instead of converting to BGR.
        // Skips sws_scale entirely — roughly halves per-frame CPU cost.
        // Use when downstream algorithms accept YUV natively (OpenCV, most
        // ML pipelines).  When true, publishes bgr8 (default).
        declare_parameter("convert_to_bgr", true);

        const auto in_topic  = get_parameter("input_topic").as_string();
        const auto out_topic = get_parameter("output_topic").as_string();
        frame_id_            = get_parameter("frame_id").as_string();
        live_stream_         = get_parameter("live_stream").as_bool();
        convert_to_bgr_      = get_parameter("convert_to_bgr").as_bool();

        // ---- FFmpeg setup ------------------------------------------------
        // avcodec_register_all() is required in FFmpeg < 4.0 to register
        // all built-in codecs before avcodec_find_decoder() can find them.
        // It was deprecated in 4.0 (codecs auto-register) and removed later,
        // so we guard it with a version check for cross-version compatibility.
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(58, 0, 0)
        avcodec_register_all();
#endif

        const AVCodec * codec = avcodec_find_decoder(AV_CODEC_ID_H264);
        if (!codec)
            throw std::runtime_error("avcodec_find_decoder failed for H264");

        codec_ctx_ = avcodec_alloc_context3(codec);
        if (!codec_ctx_)
            throw std::runtime_error("avcodec_alloc_context3 failed");

        if (avcodec_open2(codec_ctx_, codec, nullptr) < 0)
            throw std::runtime_error("avcodec_open2 failed");

        // Promote concealed frames to flagged errors where possible.
        // AV_EF_AGGRESSIVE is critical for H264 — without it FFmpeg silently
        // conceals missing reference data and outputs a corrupt-looking frame
        // without setting decode_error_flags or printing any log message,
        // bypassing both our corruption detection mechanisms entirely.
        codec_ctx_->err_recognition |= AV_EF_CRCCHECK | AV_EF_BITSTREAM
                                     | AV_EF_BUFFER   | AV_EF_AGGRESSIVE;

        // Suppress FFmpeg INFO/DEBUG log traffic entirely — our callback only
        // acts on WARNING and above, and FFmpeg is very chatty internally.
        av_log_set_level(AV_LOG_WARNING);

        // Intercept FFmpeg log messages for corruption that decode_error_flags misses.
        av_log_set_callback(ffmpeg_log_callback);

        frame_ = av_frame_alloc();
        if (!frame_)
            throw std::runtime_error("av_frame_alloc failed");

        // ---- ROS setup ---------------------------------------------------
        pub_ = create_publisher<sensor_msgs::msg::Image>(out_topic, 10);

        // RELIABLE + TRANSIENT_LOCAL guarantees the publisher replays its
        // cached SPS+PPS to this subscriber on connection, regardless of
        // startup order.  BEST_EFFORT + VOLATILE does not guarantee this
        // replay, causing intermittent "waiting for SPS+PPS" failures.
        //
        // Backlog is kept in check by depth=1 and a staleness check in the
        // callback that discards packets older than kMaxAgeMs.
        rmw_qos_profile_t qos_profile  = rmw_qos_profile_default;
        qos_profile.reliability        = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        qos_profile.durability         = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        qos_profile.depth              = 1;
        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile);

        sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            in_topic, qos,
            [this](sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                callback(std::move(msg));
            });

        watchdog_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            [this]() { watchdog(); });

        stats_timer_ = create_wall_timer(
            std::chrono::seconds(10),
            [this]() { print_stats(); });

        RCLCPP_INFO(get_logger(), "H264 decoder started  |  %s → %s",
            in_topic.c_str(), out_topic.c_str());
    }

    ~VoxlH264Decoder()
    {
        if (sws_ctx_)   sws_freeContext(sws_ctx_);
        if (frame_)     av_frame_free(&frame_);
        if (codec_ctx_) avcodec_free_context(&codec_ctx_);
    }

private:
    // -----------------------------------------------------------------------
    void callback(sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        // Format check: only run once, then skip on the hot path.
        if (!format_confirmed_) {
            const auto & fmt = msg->format;
            if (fmt != "h264" && fmt != "H264") {
                RCLCPP_WARN_ONCE(get_logger(),
                    "Unexpected format '%s' — expected h264. "
                    "Check voxl-mpa-to-ros2 configuration.", fmt.c_str());
                return;
            }
            format_confirmed_ = true;
        }

        // Param set scan: only needed until the cache is filled.
        // SPS+PPS arrives once at startup — skip the O(n) scan for every
        // subsequent P/B-frame.
        if (param_set_cache_.empty() || needs_params_) {
            if (contains_parameter_sets(msg->data)) {
                param_set_cache_ = msg->data;
                needs_params_    = false;
            }
        }

        if (needs_params_) return;

        packets_received_++;

        // Backlog detection: only active for live WiFi streams.
        // Disabled for bag playback where inter-arrival timing is unreliable
        // and would cause false-positive flushes and corrupted frames.
        if (live_stream_) {
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
                flush_and_restore_params();
                return;
            }
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
                "SPS+PPS was likely missed at startup. "
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
            "[ 10s stats ]  received: %d  published: %d  "
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
    // Returns true if the packet contains at least one H264 parameter set NAL:
    // SPS (7) or PPS (8).  Returns true even if the packet also contains other
    // NAL types (e.g. an IDR frame).
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

            const uint8_t nal = data[i] & 0x1F;   // H264 NAL type
            if (nal == 7 || nal == 8)              // SPS / PPS
                return true;
            ++i;
        }
        return false;
    }

    // -----------------------------------------------------------------------
    // Flush the decoder and replay cached SPS+PPS so it is immediately ready
    // to decode the next I-frame without erroring.
    // -----------------------------------------------------------------------
    void flush_and_restore_params()
    {
        avcodec_flush_buffers(codec_ctx_);
        got_keyframe_        = false;
        consecutive_errors_  = 0;
        consecutive_corrupt_ = 0;
        flush_count_++;

        if (param_set_cache_.empty()) {
            needs_params_ = true;
            return;
        }

        AVPacket ps_pkt{};
        ps_pkt.data = param_set_cache_.data();
        ps_pkt.size = static_cast<int>(param_set_cache_.size());

        if (avcodec_send_packet(codec_ctx_, &ps_pkt) < 0) return;

        // Drain any frames produced by the parameter set replay using the
        // member frame_ — no heap allocation needed.
        while (avcodec_receive_frame(codec_ctx_, frame_) == 0)
            av_frame_unref(frame_);
    }

    // -----------------------------------------------------------------------
    void publish_frame(const std_msgs::msg::Header & src_header)
    {
        const int w = frame_->width;
        const int h = frame_->height;

        auto out            = sensor_msgs::msg::Image();
        out.header          = src_header;
        out.header.frame_id = frame_id_;
        out.height          = static_cast<uint32_t>(h);
        out.width           = static_cast<uint32_t>(w);
        out.is_bigendian    = false;

        if (convert_to_bgr_) {
            // ---- YUV → BGR conversion ----------------------------------------
            const int row_bytes = w * 3;
            const size_t needed = static_cast<size_t>(h * row_bytes);

            if (output_buffer_.size() < needed)
                output_buffer_.resize(needed);

            const auto src_fmt = static_cast<AVPixelFormat>(frame_->format);
            sws_ctx_ = sws_getCachedContext(
                sws_ctx_,
                w, h, src_fmt,
                w, h, AV_PIX_FMT_BGR24,
                SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);

            if (!sws_ctx_) {
                RCLCPP_ERROR(get_logger(), "sws_getCachedContext failed");
                return;
            }

            uint8_t * dst_data[4]     = {};
            int       dst_linesize[4] = {};
            av_image_fill_arrays(dst_data, dst_linesize,
                output_buffer_.data(), AV_PIX_FMT_BGR24, w, h, 1);

            sws_scale(sws_ctx_,
                frame_->data, frame_->linesize, 0, h,
                dst_data,     dst_linesize);

            out.encoding = "bgr8";
            out.step     = static_cast<uint32_t>(row_bytes);
            out.data     = std::move(output_buffer_);

            pub_->publish(out);

            output_buffer_ = std::move(out.data);   // reclaim buffer after publish

        } else {
            // ---- Raw YUV420p — no conversion, zero extra CPU cost ------------
            // YUV420p layout: full Y plane, then half-res U and V planes.
            // Total size: w*h + w*h/4 + w*h/4 = w*h*3/2 bytes.
            // step (stride) is set to w for the Y plane as per ROS convention.
            const size_t needed = static_cast<size_t>(w * h * 3 / 2);

            if (output_buffer_.size() < needed)
                output_buffer_.resize(needed);

            // Copy the three planes into a contiguous buffer.
            // frame_->linesize may include padding so we copy row by row.
            uint8_t * dst = output_buffer_.data();

            // Y plane
            for (int row = 0; row < h; ++row) {
                std::memcpy(dst, frame_->data[0] + row * frame_->linesize[0], w);
                dst += w;
            }
            // U plane (half height and width)
            for (int row = 0; row < h / 2; ++row) {
                std::memcpy(dst, frame_->data[1] + row * frame_->linesize[1], w / 2);
                dst += w / 2;
            }
            // V plane (half height and width)
            for (int row = 0; row < h / 2; ++row) {
                std::memcpy(dst, frame_->data[2] + row * frame_->linesize[2], w / 2);
                dst += w / 2;
            }

            out.encoding = "yuv420p";
            out.step     = static_cast<uint32_t>(w);
            out.data     = std::move(output_buffer_);

            pub_->publish(out);

            output_buffer_ = std::move(out.data);
        }
    }

    // ---- FFmpeg state ------------------------------------------------------
    AVCodecContext * codec_ctx_;
    SwsContext *     sws_ctx_;
    AVFrame *        frame_;        // decoded YUV frame, reused per callback

    // Persistent BGR output buffer — sws_scale writes directly into this,
    // eliminating the intermediate AVFrame and row-by-row memcpy.
    std::vector<uint8_t> output_buffer_;

    // ---- Decoder state -----------------------------------------------------
    bool needs_params_        {true};   // true until first SPS+PPS seen
    bool got_keyframe_        {false};
    bool format_confirmed_    {false};  // skip format string check after first packet
    int  consecutive_errors_  {0};
    int  consecutive_corrupt_ {0};
    int64_t last_recv_ns_     {0};      // steady_clock time of last packet
    double  avg_gap_ms_       {-1.0};   // rolling average inter-arrival (ms)
    static constexpr int kMaxConsecutiveErrors  {5};
    static constexpr int kMaxConsecutiveCorrupt {4};   // ~133ms at 30fps

    // ---- Stats (reset every 10s by print_stats) ----------------------------
    int packets_received_ {0};
    int frames_published_ {0};
    int frames_dropped_   {0};
    int packets_failed_   {0};
    int flush_count_      {0};

    // ---- Parameter set cache -----------------------------------------------
    // Replayed after any flush so the decoder doesn't fail on the next I-frame.
    std::vector<uint8_t> param_set_cache_;

    // ---- ROS state ---------------------------------------------------------
    std::string frame_id_;
    bool        live_stream_    {true};
    bool        convert_to_bgr_ {true};
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxlH264Decoder>());
    rclcpp::shutdown();
    return 0;
}