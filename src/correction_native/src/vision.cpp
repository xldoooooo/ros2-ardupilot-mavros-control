// Keep the Python detector's full image algorithm and numerical representation.
#include "correction_native/vision.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

#include "correction_native/image.hpp"

namespace correction_native {
// Preserve original dictionary and candidate thresholds, including printed
// outer-frame handling.
Detector::Detector(double scale, bool refine, bool track_roi)
    : scale_(scale),
      refine_(refine),
      track_roi_(track_roi),
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
      dictionary_(cv::makePtr<cv::aruco::Dictionary>(
          cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11))),
#else
      dictionary_(
          cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11)),
#endif
      parameters_(cv::makePtr<cv::aruco::DetectorParameters>()) {
  parameters_->markerBorderBits = 1;
  parameters_->minMarkerPerimeterRate = .01;
  parameters_->maxMarkerPerimeterRate = .9;
  // Keep the encoded border separate from the extra printed paper frame.
  parameters_->minMarkerDistanceRate = .02;
  parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
}

// The margin covers ordinary hover sway; larger motion always recovers on this
// same frame.
constexpr double ROI_MIN_MARGIN_PX = 96.;
// Adaptive thresholding and subpixel windows must not touch an artificial crop
// boundary.
constexpr int ROI_CONTEXT_PX = 32;

// Clear only tracking state; dictionary and detector parameters remain
// unchanged.
void Detector::reset() {
  roi_ = {};
  image_size_ = {};
  selected_.clear();
  previous_points_.clear();
  stats_ = {};
}

// Apply perimeter limits in original-image units, independent of the crop's
// dimensions.
Corners Detector::search(const cv::Mat &image, const cv::Rect &region) const {
  auto parameters = cv::makePtr<cv::aruco::DetectorParameters>(*parameters_);
  const double ratio = static_cast<double>(std::max(image.cols, image.rows)) /
                       std::max(region.width, region.height);
  parameters->minMarkerPerimeterRate *= ratio;
  parameters->maxMarkerPerimeterRate *= ratio;
  Corners result;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::aruco::ArucoDetector detector(*dictionary_, *parameters);
  detector.detectMarkers(image(region), result.points, result.ids);
#else
  cv::aruco::detectMarkers(image(region), dictionary_, result.points,
                           result.ids, parameters);
#endif
  for (auto &marker : result.points)
    for (auto &point : marker) point += cv::Point2f(region.x, region.y);
  return result;
}

// Track the requested target with a half-tag margin and extra
// translation/rotation allowance.
void Detector::update_region(const Corners &corners,
                             const std::set<int> &selected, cv::Size size) {
  std::vector<cv::Point2f> points;
  for (std::size_t index = 0; index < corners.ids.size(); ++index)
    if (selected.count(corners.ids[index]))
      points.insert(points.end(), corners.points[index].begin(),
                    corners.points[index].end());
  if (points.empty()) {
    roi_ = {};
    previous_points_.clear();
    return;
  }
  float left = points[0].x, right = left, top = points[0].y, bottom = top;
  double motion_x = 0, motion_y = 0;
  cv::Point2f shift{};
  for (std::size_t index = 0; index < points.size(); ++index) {
    left = std::min(left, points[index].x);
    right = std::max(right, points[index].x);
    top = std::min(top, points[index].y);
    bottom = std::max(bottom, points[index].y);
    if (points.size() == previous_points_.size()) {
      const auto delta = points[index] - previous_points_[index];
      shift += delta;
      motion_x = std::max(motion_x, static_cast<double>(std::abs(delta.x)));
      motion_y = std::max(motion_y, static_cast<double>(std::abs(delta.y)));
    }
  }
  shift *= 1.f / points.size();
  const double margin_x =
      std::max(ROI_MIN_MARGIN_PX, (right - left) * .5) + 2 * motion_x;
  const double margin_y =
      std::max(ROI_MIN_MARGIN_PX, (bottom - top) * .5) + 2 * motion_y;
  const int x =
      std::max(0, static_cast<int>(std::floor(left + shift.x - margin_x)));
  const int y =
      std::max(0, static_cast<int>(std::floor(top + shift.y - margin_y)));
  const int end_x = std::min(
      size.width, static_cast<int>(std::ceil(right + shift.x + margin_x)));
  const int end_y = std::min(
      size.height, static_cast<int>(std::ceil(bottom + shift.y + margin_y)));
  roi_ = end_x > x && end_y > y ? cv::Rect(x, y, end_x - x, end_y - y)
                                : cv::Rect();
  previous_points_ = std::move(points);
}

// Search the tracked region, recover immediately if needed, then refine on
// original pixels.
Corners Detector::detect(const cv::Mat &gray, const std::set<int> &selected) {
  const auto image = resize_gray(gray, scale_);
  if (image.size() != image_size_ || selected != selected_) reset();
  image_size_ = image.size();
  selected_ = selected;
  const cv::Rect full(0, 0, image.cols, image.rows);
  const bool use_roi = track_roi_ && roi_.area() > 0 && roi_ != full;
  stats_ = {};
  stats_.used_roi = use_roi;
  stats_.region = use_roi ? roi_ : full;
  stats_.pixels = stats_.region.area();
  Corners result = search(image, stats_.region);
  bool found = false, clipped = false;
  for (std::size_t index = 0; index < result.ids.size(); ++index) {
    if (!selected.count(result.ids[index])) continue;
    found = true;
    for (const auto &point : result.points[index]) {
      const auto &r = stats_.region;
      if ((r.x > 0 && point.x < r.x + ROI_CONTEXT_PX) ||
          (r.y > 0 && point.y < r.y + ROI_CONTEXT_PX) ||
          (r.br().x < image.cols && point.x > r.br().x - ROI_CONTEXT_PX) ||
          (r.br().y < image.rows && point.y > r.br().y - ROI_CONTEXT_PX))
        clipped = true;
    }
  }
  if (use_roi && (!found || clipped)) {
    result = search(image, full);
    stats_.fallback = true;
    stats_.pixels += full.area();
    stats_.region = full;
  }
  for (std::size_t index = 0; index < result.ids.size(); ++index) {
    if (!refine_ || !selected.count(result.ids[index])) continue;
    auto &points = result.points[index];
    float shortest = std::numeric_limits<float>::infinity();
    for (int corner = 0; corner < 4; ++corner) {
      const auto delta = points[corner] - points[(corner + 3) % 4];
      shortest =
          std::min(shortest, std::sqrt(delta.x * delta.x + delta.y * delta.y));
    }
    const int radius =
        std::max(1, std::min(7, static_cast<int>(shortest / 16)));
    cv::cornerSubPix(
        image, points, {radius, radius}, {-1, -1},
        {cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 40, .01});
  }
  if (track_roi_) update_region(result, selected, image.size());
  return result;
}

// Keep full JPEG decoding and every source pixel in C++; a failed ROI can reuse
// this same frame.
Corners Detector::detect_jpeg(const cv::Mat &jpeg, int width, int height,
                              const std::set<int> &selected) {
  const auto started = std::chrono::steady_clock::now();
  const auto image = cv::imdecode(jpeg, cv::IMREAD_GRAYSCALE);
  const auto decoded = std::chrono::steady_clock::now();
  if (image.empty() || image.cols != width || image.rows != height)
    throw std::runtime_error(
        "UVC MJPEG decoder returned an invalid image/size");
  auto result = detect(image, selected);
  stats_.decode_ms =
      std::chrono::duration<double, std::milli>(decoded - started).count();
  return result;
}

// Match IPPE_SQUARE ordering, distortion handling and RMS in detector pixel
// units.
Pose estimate_pose(const std::vector<cv::Point2d> &points, double size,
                   const cv::Mat &matrix, const cv::Mat &distortion) {
  const double half = size / 2;
  const std::vector<cv::Point3d> object = {
      {-half, half, 0}, {half, half, 0}, {half, -half, 0}, {-half, -half, 0}};
  cv::Mat rotation_vector, translation;
  if (!cv::solvePnP(object, points, matrix, distortion, rotation_vector,
                    translation, false, cv::SOLVEPNP_IPPE_SQUARE))
    return {};
  if (!cv::checkRange(translation) || translation.at<double>(2) <= 0.)
    return {};
  cv::Mat rotation;
  cv::Rodrigues(rotation_vector, rotation);
  std::vector<cv::Point2d> projected;
  cv::projectPoints(object, rotation_vector, translation, matrix, distortion,
                    projected);
  double residual = 0;
  for (int i = 0; i < 4; ++i) {
    const auto delta = projected[i] - points[i];
    residual += delta.dot(delta);
  }
  const double rms = std::sqrt(residual / 4);
  if (!cv::checkRange(rotation) || !std::isfinite(rms)) return {};
  return {rotation, translation, rms, true};
}
}  // namespace correction_native
