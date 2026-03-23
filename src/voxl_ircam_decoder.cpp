#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
}

#include <cstring>
#include <vector>

// ---------------------------------------------------------------------------
// NAL helpers
// ---------------------------------------------------------------------------
static constexpr uint8_t NAL_TYPE_IDR = 5;

static bool contains_idr(const uint8_t * data, size_t size)
{
    // Scan for 3- or 4-byte start codes and check NAL type
    for (size_t i = 0; i + 3 < size; ) {
        if (data[i] == 0 && data[i + 1] == 0) {
            if (data[i + 2] == 1) {
                if ((data[i + 3] & 0x1F) == NAL_TYPE_IDR) return true;
                i += 3;
            } else if (data[i + 2] == 0 && i + 4 < size && data[i + 3] == 1) {
                if ((data[i + 4] & 0x1F) == NAL_TYPE_IDR) return true;
                i += 4;
            } else {
                ++i;
            }
        } else {
            ++i;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
class H264LiveDecoder : public rclcpp::Node
{
public:
    H264LiveDecoder() : Node("h264_live_decoder") {
        
        // Precomputed heatmap Look-Up Table for color conversion
        for (int v = 0; v < 256; ++v) {
            if (v < 128) {
                lut_r_[v] = static_cast<uint8_t>(2 * v);
                lut_g_[v] = static_cast<uint8_t>(2 * v);
                lut_b_[v] = static_cast<uint8_t>(255 - 2 * v);
            } else {
                lut_r_[v] = 255;
                lut_g_[v] = static_cast<uint8_t>(510 - 2 * v);
                lut_b_[v] = 0;
            }
        }

        declare_parameter<std::string>("input_topic",  "/ircam/h264");
        declare_parameter<std::string>("output_topic", "/ircam/decoded");
        auto in_topic = get_parameter("input_topic").as_string();
        auto out_topic = get_parameter("output_topic").as_string();


        // Decoder init 
        const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_H264);
        if (!codec) {
            RCLCPP_FATAL(get_logger(), "H.264 decoder not found in FFmpeg");
            throw std::runtime_error("No H.264 decoder");
        }

        dec_ctx_ = avcodec_alloc_context3(codec);
        // Low-latency: single-threaded slice decode, no frame threading
        dec_ctx_->thread_count = 1;
        dec_ctx_->thread_type = FF_THREAD_SLICE;
        // Don't output corrupt frames
        dec_ctx_->flags |= AV_CODEC_FLAG_OUTPUT_CORRUPT;
        dec_ctx_->flags2 &= ~AV_CODEC_FLAG2_SHOW_ALL;

        if (avcodec_open2(dec_ctx_, codec, nullptr) < 0) {
            RCLCPP_FATAL(get_logger(), "Failed to open H.264 decoder");
            throw std::runtime_error("avcodec_open2 failed");
        }

        frame_  = av_frame_alloc();
        packet_ = av_packet_alloc();

        // ----- ROS plumbing ------------------------------------------------
        sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            in_topic, rclcpp::SensorDataQoS(),
            std::bind(&H264LiveDecoder::on_compressed, this, std::placeholders::_1));

        rclcpp::QoS pub_qos(10);
        pub_qos.reliable();
        pub_ = create_publisher<sensor_msgs::msg::Image>(out_topic, pub_qos);

        RCLCPP_INFO(get_logger(), "Decoding %s → %s", in_topic.c_str(), out_topic.c_str());
    }

    ~H264LiveDecoder() override
    {
        if (frame_) av_frame_free(&frame_);
        if (packet_) av_packet_free(&packet_);
        if (dec_ctx_) avcodec_free_context(&dec_ctx_);
    }

private:
    void on_compressed(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        const uint8_t* nal_data = msg->data.data();
        const size_t nal_size = msg->data.size();

        // Gate: wait for an IDR before feeding the decoder
        if (!got_keyframe_) {
            if (contains_idr(nal_data, nal_size)) {
                got_keyframe_ = true;
                RCLCPP_INFO(get_logger(), "IDR keyframe received – starting decode");
            } else {
                return;
            }
        }

        // Feed NAL data to decoder
        packet_->data = const_cast<uint8_t *>(nal_data);
        packet_->size = static_cast<int>(nal_size);

        int ret = avcodec_send_packet(dec_ctx_, packet_);
        if (ret < 0) return;

        // Pull all decoded frames
        while (avcodec_receive_frame(dec_ctx_, frame_) == 0) {
            // Skip corrupt frames
            if (frame_->flags & AV_FRAME_FLAG_CORRUPT) continue;
            // Skip obviously broken dimensions
            if (frame_->width <= 0 || frame_->height <= 0) continue;

            publish_frame(msg->header);
        }
    }

    void publish_frame(const std_msgs::msg::Header & header) {
        bool grey = true;

        const int w = frame_->width;
        const int h = frame_->height;

        // YUV420P data[0] is the Y (luma) plane
        const int stride = frame_->linesize[0];
        
        // Decoded message
        auto out = std::make_unique<sensor_msgs::msg::Image>();
        out->header = header;
        out->height = static_cast<uint32_t>(h);
        out->width = static_cast<uint32_t>(w);


        if (grey) {
            out->encoding = "mono8";
            out->step     = static_cast<uint32_t>(w);
            out->data.resize(static_cast<size_t>(w) * h);

            if (stride == w) {
                std::memcpy(out->data.data(), frame_->data[0], static_cast<size_t>(w) * h);
            } 
            else {
                // Row-by-row copy (stride has padding)
                for (int y = 0; y < h; ++y) {
                    std::memcpy(out->data.data() + y * w,
                                frame_->data[0] + y * stride,
                                static_cast<size_t>(w));
                }
            }
        }
        else {
            out->encoding = "rgb8";
            out->step = static_cast<uint32_t>(3 * w);
            out->data.resize(static_cast<size_t>(3 * w) * h);

            uint8_t * dst = out->data.data();
            for (int row = 0; row < h; ++row) {
                const uint8_t * src = frame_->data[0] + row * stride;
                for (int x = 0; x < w; ++x) {
                    const uint8_t v = src[x];
                    *dst++ = lut_r_[v];
                    *dst++ = lut_g_[v];
                    *dst++ = lut_b_[v];
                }
            }
        }

        pub_->publish(std::move(out));
    }

    // Decoder state
    AVCodecContext* dec_ctx_ = nullptr;
    AVFrame* frame_ = nullptr;
    AVPacket* packet_ = nullptr;

    uint8_t lut_r_[256], lut_g_[256], lut_b_[256]; // heatmap precomputed lookup table
    
    bool got_keyframe_ = false;

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<H264LiveDecoder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}