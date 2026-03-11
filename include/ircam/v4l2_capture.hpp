#ifndef V4L2_CAPTURE_HPP_
#define V4L2_CAPTURE_HPP_

#include <cstdint>
#include <string>
#include <vector>
#include <functional>

namespace ir_v4l2_camera {

struct MmapBuffer {
    void* start = nullptr;
    size_t length = 0;
};

struct CaptureConfig {
    std::string device = "/dev/video2";
    int width = 640;
    int height = 512;
    int fps = 30;
    int num_buffers = 8;
    int select_timeout_s = 2;
};

struct CaptureStats {
    uint64_t frames_captured = 0;
    uint64_t frames_dropped = 0;
    uint64_t select_timeouts = 0;
};

class V4L2Capture {
public:
    V4L2Capture() = default;
    ~V4L2Capture();

    V4L2Capture(const V4L2Capture&) = delete; //non-copyable
    V4L2Capture& operator=(const V4L2Capture&) = delete; //non-movable

    // Open device, check format, allocate buffers
    bool open(const CaptureConfig& cfg);

    // Queue buffers and start stream
    bool start();

    // Return pointer to YUYV data in mmap buffer
    const uint8_t* grab_frame(size_t& bytes_used, uint64_t& timestamp_us);

    // Returns current buffer V4L2 driver queue
    // called after each grap_frame
    void release_frame();

    // Stream off and cleanup
    void close();

    int width() const { return actual_width_; }
    int height() const { return actual_height_; }
    int fps() const { return actual_fps_; }

    const CaptureStats& stats() const { return stats_; }

private:
    int fd_ = -1;

    int actual_width_ = 0;
    int actual_height_ = 0;
    int actual_fps_ = 0;
    int select_timeout_s_ = 2;

    std::vector<MmapBuffer> buffers_;
    int current_buf_index_ = -1;

    CaptureStats stats_{};

    static int xioctl(int fd, unsigned long request, void* arg);
};

} // namespace ir_v4l2_camera

#endif // V4L2_CAPTURE_HPP_