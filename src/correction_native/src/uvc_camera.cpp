// Continuous latest-JPEG UVC acquisition; decode and publish only scheduler-selected frames.
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>

#include "correction_native/capture_clock.hpp"
#include "correction_native/image.hpp"

namespace correction_native {
// Signal handlers only stop the loop; ROS context remains alive until publishers close.
volatile std::sig_atomic_t stopping = 0;

// Preserve the standalone executable's normal SIGINT/SIGTERM cleanup path.
static void stop_signal(int) { stopping = 1; }

// Use the camera driver's clock for acquisition age, independent of its negotiated fps.
static std::int64_t monotonic_ns() {
  timespec value{};
  clock_gettime(CLOCK_MONOTONIC, &value);
  return value.tv_sec * 1'000'000'000LL + value.tv_nsec;
}

// Keep ioctl errors visible while retrying interrupted system calls.
static void control(int fd, unsigned long request, void *argument) {
  int result;
  do {
    result = ioctl(fd, request, argument);
  } while (result < 0 && errno == EINTR);
  if (result < 0) throw std::system_error(errno, std::generic_category(), "UVC ioctl");
}

struct Sample {
  std::vector<unsigned char> jpeg;
  v4l2_buffer buffer{};
};

class Capture {
 public:
  // Configure the same native MJPEG profile and four mmap buffers as before.
  Capture(const std::string &device, int width, int height, int fps) {
    try {
      fd_ = open(device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
      if (fd_ < 0) throw std::system_error(errno, std::generic_category(), "open UVC device");
      v4l2_format format{};
      format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      format.fmt.pix.width = width;
      format.fmt.pix.height = height;
      format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
      control(fd_, VIDIOC_S_FMT, &format);
      if (format.fmt.pix.width != static_cast<unsigned>(width) ||
          format.fmt.pix.height != static_cast<unsigned>(height) ||
          format.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG)
        throw std::runtime_error("UVC device changed the requested MJPEG size/format");
      v4l2_streamparm parameters{};
      parameters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      parameters.parm.capture.timeperframe.numerator = 1;
      parameters.parm.capture.timeperframe.denominator = fps;
      control(fd_, VIDIOC_S_PARM, &parameters);
      negotiated_num = parameters.parm.capture.timeperframe.denominator;
      negotiated_den = parameters.parm.capture.timeperframe.numerator;
      v4l2_requestbuffers request{};
      request.count = 4;
      request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      request.memory = V4L2_MEMORY_MMAP;
      control(fd_, VIDIOC_REQBUFS, &request);
      if (request.count < 2)
        throw std::runtime_error("UVC device did not allocate enough mmap buffers");
      for (unsigned index = 0; index < request.count; ++index) {
        v4l2_buffer buffer{};
        buffer.index = index;
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        control(fd_, VIDIOC_QUERYBUF, &buffer);
        void *address =
            mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buffer.m.offset);
        if (address == MAP_FAILED)
          throw std::system_error(errno, std::generic_category(), "UVC mmap");
        buffers_.push_back({address, buffer.length});
        control(fd_, VIDIOC_QBUF, &buffer);
      }
      int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      control(fd_, VIDIOC_STREAMON, &type);
      streaming_ = true;
      thread_ = std::thread([this]() { read_loop(); });
    } catch (...) {
      close();
      throw;
    }
  }

  // Join the reader before touching mmap or fd; closing the fd releases kernel buffers.
  ~Capture() { close(); }

  // Take only the latest compressed frame while acquisition continues during decoding.
  std::optional<Sample> latest() {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait_for(lock, std::chrono::milliseconds(200), [this]() {
      return latest_.has_value() || !error_.empty() || closing_.load();
    });
    if (!error_.empty()) throw std::runtime_error("UVC capture failed: " + error_);
    auto result = std::move(latest_);
    latest_.reset();
    return result;
  }

  // Expose acquisition throughput independently of selected publication throughput.
  std::uint64_t received() const { return received_.load(); }
  unsigned negotiated_num = 0, negotiated_den = 0;

 private:
  struct Buffer {
    void *address;
    std::size_t size;
  };
  int fd_ = -1;
  bool streaming_ = false;
  std::vector<Buffer> buffers_;
  std::atomic<bool> closing_{false};
  std::atomic<std::uint64_t> received_{0};
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<Sample> latest_;
  std::string error_;

  // Drain at most the allocated buffers and return every dequeued buffer immediately.
  std::optional<Sample> read_device() {
    fd_set readable;
    FD_ZERO(&readable);
    FD_SET(fd_, &readable);
    timeval timeout{0, 200000};
    const int ready = select(fd_ + 1, &readable, nullptr, nullptr, &timeout);
    if (ready < 0 && errno != EINTR)
      throw std::system_error(errno, std::generic_category(), "UVC select");
    if (ready <= 0) return {};
    std::optional<Sample> latest;
    for (std::size_t index = 0; index < buffers_.size(); ++index) {
      v4l2_buffer buffer{};
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;
      if (ioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
        if (errno == EAGAIN) break;
        if (errno == EINTR) continue;
        throw std::system_error(errno, std::generic_category(), "UVC dequeue");
      }
      try {
        if (buffer.index >= buffers_.size())
          throw std::runtime_error("UVC driver returned an invalid buffer index");
        const auto &storage = buffers_[buffer.index];
        if (buffer.bytesused > storage.size)
          throw std::runtime_error("UVC payload exceeds mmap buffer");
        if (buffer.bytesused && !(buffer.flags & V4L2_BUF_FLAG_ERROR)) {
          const auto *bytes = static_cast<const unsigned char *>(storage.address);
          latest = Sample{{bytes, bytes + buffer.bytesused}, buffer};
        }
      } catch (...) {
        control(fd_, VIDIOC_QBUF, &buffer);
        throw;
      }
      control(fd_, VIDIOC_QBUF, &buffer);
    }
    return latest;
  }

  // Keep decompression and all ROS publishing outside the acquisition thread.
  void read_loop() {
    try {
      while (!closing_) {
        auto sample = read_device();
        if (sample) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_ = std::move(sample);
            ++received_;
          }
          condition_.notify_one();
        }
      }
    } catch (const std::exception &error) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        error_ = error.what();
      }
      condition_.notify_one();
    }
  }

  // Best-effort STREAMOFF must never prevent mmap and fd release.
  void close() noexcept {
    closing_ = true;
    condition_.notify_all();
    if (thread_.joinable()) thread_.join();
    if (streaming_) {
      int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      ioctl(fd_, VIDIOC_STREAMOFF, &type);
      streaming_ = false;
    }
    for (const auto &buffer : buffers_) munmap(buffer.address, buffer.size);
    buffers_.clear();
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }
};
}  // namespace correction_native

// Preserve capture parameters and source stamps while publishing the configured detection pixels.
int main(int argc, char **argv) {
  using namespace correction_native;
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, stop_signal);
  std::signal(SIGTERM, stop_signal);
  auto node = std::make_shared<rclcpp::Node>("correction_uvc_camera");
  int result = 0;
  try {
    const auto device = node->declare_parameter("video_device", std::string("/dev/video0"));
    const int width = node->declare_parameter("image_width", 1920);
    const int height = node->declare_parameter("image_height", 1080);
    const int fps = node->declare_parameter("framerate", 30);
    const double publish_fps = node->declare_parameter("publish_fps", 30.);
    const double image_scale = node->declare_parameter("image_scale", 1.);
    const auto frame_id =
        node->declare_parameter("frame_id", std::string("correction_camera_optical_frame"));
    const auto topic =
        node->declare_parameter("image_topic", std::string("/correction_service/image_raw"));
    const double max_age = node->declare_parameter("max_capture_age_ms", 200.) * 1e6;
    if (width <= 0 || height <= 0 || fps <= 0 || max_age <= 0 ||
        !std::isfinite(image_scale) || image_scale <= 0. || image_scale > 1.)
      throw std::invalid_argument("invalid UVC dimensions, fps, capture-age limit or image_scale");
    if (node->get_parameter("use_sim_time").as_bool())
      throw std::invalid_argument("UVC hardware capture requires the system ROS clock");
    CaptureRateLimiter limiter(publish_fps);
    auto publisher =
        node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(1).best_effort());
    Capture capture(device, width, height, fps);
    RCLCPP_INFO(node->get_logger(),
                "UVC mmap started: %s %dx%d MJPEG requested_fps=%d negotiated_fps=(%u,%u) "
                "publish_fps=%.3f timestamp=V4L2_CLOCK_MONOTONIC decoder=OpenCV native=true "
                "image_scale=%.4f output_size=%dx%d",
                device.c_str(), width, height, fps, capture.negotiated_num, capture.negotiated_den,
                publish_fps, image_scale, cvRound(width * image_scale), cvRound(height * image_scale));
    std::uint64_t count = 0, stale = 0, nonmonotonic = 0, rate_limited = 0, last_count = 0;
    std::int64_t last_stamp = 0, report_time = monotonic_ns(), last_frame = report_time;
    double age_sum = 0, age_max = 0;
    while (!stopping && rclcpp::ok()) {
      auto sample = capture.latest();
      const auto mono = monotonic_ns();
      if (!sample) {
        if (mono - last_frame > 3'000'000'000LL)
          throw std::runtime_error("UVC stream produced no complete frame for 3 seconds");
        continue;
      }
      last_frame = mono;
      auto [stamp, age] = capture_stamp(sample->buffer, mono, node->now().nanoseconds());
      if (age > max_age)
        ++stale;
      else if (stamp <= last_stamp)
        ++nonmonotonic;
      else if (!limiter.accept(mono - age))
        ++rate_limited;
      else {
        const auto frame = cv::imdecode(sample->jpeg, cv::IMREAD_GRAYSCALE);
        if (frame.empty() || frame.rows != height || frame.cols != width)
          throw std::runtime_error("UVC MJPEG decoder returned an invalid image/size");
        const auto output = resize_gray(frame, image_scale);
        age += monotonic_ns() - mono;
        if (age > max_age)
          ++stale;
        else {
          auto image = std::make_unique<sensor_msgs::msg::Image>();
          image->header.stamp = rclcpp::Time(stamp);
          image->header.frame_id = frame_id;
          image->height = output.rows;
          image->width = output.cols;
          image->encoding = "mono8";
          image->step = output.cols;
          image->data.assign(output.datastart, output.dataend);
          publisher->publish(std::move(image));
          last_stamp = stamp;
          ++count;
          age_sum += age / 1e6;
          age_max = std::max(age_max, age / 1e6);
        }
      }
      const auto now = monotonic_ns();
      if (now - report_time >= 5'000'000'000LL) {
        const double elapsed = (now - report_time) / 1e9;
        const auto captured = capture.received();
        RCLCPP_INFO(node->get_logger(),
                    "capture_rate=%.3f output_rate=%.3f stale=%lu nonmonotonic=%lu "
                    "rate_limited=%lu age_mean_ms=%.3f age_max_ms=%.3f",
                    (captured - last_count) / elapsed, count / elapsed, stale, nonmonotonic,
                    rate_limited, count ? age_sum / count : 0., age_max);
        report_time = now;
        last_count = captured;
        count = stale = nonmonotonic = rate_limited = 0;
        age_sum = age_max = 0;
      }
    }
  } catch (const std::exception &error) {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what());
    result = 1;
  }
  node.reset();
  rclcpp::shutdown();
  return result;
}
