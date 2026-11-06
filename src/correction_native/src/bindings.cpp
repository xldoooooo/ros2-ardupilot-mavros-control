// Convert only selected observations and on-demand raw state across the Python boundary.
#include <cstring>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "correction_native/capture_clock.hpp"
#include "correction_native/image.hpp"
#include "correction_native/odometry.hpp"
#include "correction_native/vision.hpp"

namespace py = pybind11;
using namespace correction_native;

// Borrow contiguous input pixels for the duration of a GIL-free native detection.
static cv::Mat gray_view(const py::array_t<std::uint8_t, py::array::c_style> &pixels) {
  if (pixels.ndim() != 2) throw std::invalid_argument("AprilTag 输入必须是 mono8 图像");
  return {static_cast<int>(pixels.shape(0)), static_cast<int>(pixels.shape(1)), CV_8UC1,
          const_cast<std::uint8_t *>(pixels.data())};
}

// Expose independent data-plane operations; Python continues to own every transaction.
PYBIND11_MODULE(_correction_native, module) {
  module.attr("opencv_version") = CV_VERSION;
  module.def("resize_gray", [](py::array_t<std::uint8_t, py::array::c_style> pixels, double scale) {
    const auto image = gray_view(pixels);
    cv::Mat result;
    {
      py::gil_scoped_release release;
      result = resize_gray(image, scale);
    }
    py::array_t<std::uint8_t> output({result.rows, result.cols});
    std::memcpy(output.mutable_data(), result.data, result.total());
    return output;
  });
  py::class_<CaptureRateLimiter>(module, "CaptureRateLimiter")
      .def(py::init<double>())
      .def("accept", &CaptureRateLimiter::accept);
  module.def("capture_stamp", [](unsigned flags, std::int64_t sec, std::int64_t usec,
                                 std::int64_t mono, std::int64_t ros) {
    v4l2_buffer buffer{};
    buffer.flags = flags;
    buffer.timestamp.tv_sec = sec;
    buffer.timestamp.tv_usec = usec;
    return capture_stamp(buffer, mono, ros);
  });
  py::class_<RawState>(module, "RawState")
      .def_readonly("received", &RawState::received)
      .def_readonly("header", &RawState::header)
      .def_readonly("arrival", &RawState::arrival)
      .def_readonly("transform", &RawState::transform)
      .def_readonly("session", &RawState::session)
      .def_readonly("valid", &RawState::valid);
  py::class_<OdometryCapture>(module, "OdometryCapture")
      .def(py::init<double, double, double, double, std::string, std::function<void()>>())
      .def("start", &OdometryCapture::start)
      .def("prepare", &OdometryCapture::prepare, py::call_guard<py::gil_scoped_release>())
      .def("close", &OdometryCapture::close, py::call_guard<py::gil_scoped_release>())
      .def("shutdown", &OdometryCapture::shutdown, py::call_guard<py::gil_scoped_release>())
      .def("clear", &OdometryCapture::clear)
      .def_property_readonly("running", &OdometryCapture::running)
      .def("snapshot", &OdometryCapture::snapshot)
      .def("faults",
           [](OdometryCapture &capture) {
             py::list values;
             for (const auto &fault : capture.faults())
               values.append(py::make_tuple(fault.reason, fault.invalidate));
             return values;
           })
      .def("add",
           [](OdometryCapture &capture, py::bytes data, std::int64_t arrival, double monotonic) {
             capture.add(deserialize(static_cast<std::string>(data)), arrival, monotonic);
           })
      .def("match", [](OdometryCapture &capture, std::int64_t stamp) -> py::object {
        Match result;
        {
          py::gil_scoped_release release;
          result = capture.match(stamp);
        }
        if (!result.message) return py::none();
        return py::make_tuple(py::bytes(serialize(*result.message)), result.delta_ms,
                              result.time_source);
      });
  py::class_<Detector>(module, "Detector")
      .def(py::init<double, bool>())
      .def("detect",
           [](const Detector &detector, py::array_t<std::uint8_t, py::array::c_style> pixels,
              const std::set<int> &selected) {
             const auto image = gray_view(pixels);
             Corners result;
             {
               py::gil_scoped_release release;
               result = detector.detect(image, selected);
             }
             py::list corners;
             for (const auto &marker : result.points) {
               py::array_t<float> points({1, 4, 2});
               auto out = points.mutable_unchecked<3>();
               for (int j = 0; j < 4; ++j) {
                 out(0, j, 0) = marker[j].x;
                 out(0, j, 1) = marker[j].y;
               }
               corners.append(std::move(points));
             }
             return py::make_tuple(result.ids, corners);
           });
  module.def(
      "estimate_pose",
      [](py::array_t<double, py::array::c_style | py::array::forcecast> pixels, double size,
         py::array_t<double, py::array::c_style | py::array::forcecast> matrix,
         py::array_t<double, py::array::c_style | py::array::forcecast> distortion) -> py::object {
        if (pixels.size() != 8 || matrix.size() != 9)
          throw std::invalid_argument("invalid square PnP input");
        std::vector<cv::Point2d> points;
        for (int j = 0; j < 4; ++j)
          points.emplace_back(pixels.data()[2 * j], pixels.data()[2 * j + 1]);
        const cv::Mat k(3, 3, CV_64F, const_cast<double *>(matrix.data()));
        const cv::Mat d(1, distortion.size(), CV_64F, const_cast<double *>(distortion.data()));
        Pose result;
        {
          py::gil_scoped_release release;
          result = estimate_pose(points, size, k, d);
        }
        if (!result.valid) return py::none();
        py::array_t<double> transform({4, 4});
        auto out = transform.mutable_unchecked<2>();
        for (int row = 0; row < 4; ++row)
          for (int col = 0; col < 4; ++col)
            out(row, col) = row == 3   ? (col == 3 ? 1. : 0.)
                            : col == 3 ? result.translation.at<double>(row)
                                       : result.rotation.at<double>(row, col);
        return py::make_tuple(std::move(transform), result.reprojection);
      });
}
