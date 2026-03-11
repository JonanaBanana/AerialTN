#include "ircam/v4l2_capture.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>

namespace ir_v4l2_camera {

int V4L2Capture::xioctl(int fd, unsigned long request, void* arg) {
    int r;
    do {
        r = ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

bool V4L2Capture::open(const CaptureConfig& cfg) {
    fd_ = ::open(cfg.device.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) return false;

    select_timeout_s_ = cfg.select_timeout_s;

    // Query device capabilities
    v4l2_capability cap{};
    if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }
 
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) || !(cap.capabilities & V4L2_CAP_STREAMING)) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // set pixel format
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = static_cast<uint32_t>(cfg.width);
    fmt.fmt.pix.height = static_cast<uint32_t>(cfg.height);
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    actual_width_ = static_cast<int>(fmt.fmt.pix.width);
    actual_height_ = static_cast<int>(fmt.fmt.pix.height);

    // set framerate
    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = static_cast<uint32_t>(cfg.fps);
    parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;

    if (xioctl(fd_, VIDIOC_S_PARM, &parm) == 0) {
        // read back what the driver set
        if (parm.parm.capture.timeperframe.numerator > 0) {
            actual_fps_ = static_cast<int>(
                parm.parm.capture.timeperframe.denominator / 
                parm.parm.capture.timeperframe.numerator
            );
        }
        else {
            actual_fps_ = cfg.fps;
        }
    }
    else {
        actual_fps_ = cfg.fps; 
    }

    // request mmap buffers
    v4l2_requestbuffers req{};
    req.count = static_cast<uint32_t>(cfg.num_buffers);
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // dicer may grant fewer buffers than requested
    if (req.count < 2) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    buffers_.resize(req.count);

    for (uint32_t i = 0; i < req.count; ++i) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            close();
            return false;
        }

        buffers_[i].length = buf.length;
        buffers_[i].start = mmap(nullptr, buf.length,
                                 PROT_READ | PROT_WRITE, MAP_SHARED,
                                 fd_, buf.m.offset);

        if (buffers_[i].start == MAP_FAILED) {
            buffers_[i].start = nullptr;
            close();
            return false;
        }
    }

    stats_ = {};
    return true;
}

bool V4L2Capture::start() {
    if (fd_ < 0) return false;

    // query buffers
    for (uint32_t i = 0; i < buffers_.size(); ++i) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            return false;
        }
    }

    // start stream
    auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    return xioctl(fd_, VIDIOC_STREAMON, &type) == 0;
}

const uint8_t* V4L2Capture::grab_frame(size_t& bytes_used, uint64_t& timestamp_us) {
    if (fd_ < 0) return nullptr;

    // block until the driver signals a buffer is ready
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);

    timeval tv{};
    tv.tv_sec = select_timeout_s_;
    tv.tv_usec = 0;

    int r = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (r <= 0) {
        stats_.select_timeouts++;
        return nullptr;
    }

    // dequeue buffer
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        stats_.frames_dropped++;
        return nullptr;
    }

    current_buf_index_ = static_cast<int>(buf.index);
    bytes_used = buf.bytesused;

    timestamp_us = static_cast<uint64_t>(buf.timestamp.tv_sec) * 1'000'000ULL + static_cast<uint64_t>(buf.timestamp.tv_usec);

    stats_.frames_captured++;
    return static_cast<const uint8_t*>(buffers_[buf.index].start);
}

void V4L2Capture::release_frame() {
    if (fd_ < 0 || current_buf_index_ < 0) return;

    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = static_cast<uint32_t>(current_buf_index_);

    xioctl(fd_, VIDIOC_QBUF, &buf);
    current_buf_index_ = -1;
}

void V4L2Capture::close() {
    if (fd_ < 0) return;

    auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd_, VIDIOC_STREAMOFF, &type);

    for (auto& b : buffers_) {
        if (b.start && b.start != MAP_FAILED) {
            munmap(b.start, b.length);
        }
    }
    buffers_.clear();
    current_buf_index_ = -1;

    ::close(fd_);
    fd_ = -1;
}

V4L2Capture::~V4L2Capture() {
    close();
}

} // namespace ir_v4l2_camera