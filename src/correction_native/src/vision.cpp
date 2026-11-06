// Keep the Python detector's full image algorithm and numerical representation.
#include "correction_native/vision.hpp"
#include "correction_native/image.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>

namespace correction_native {
// Preserve original dictionary and candidate thresholds, including printed
// outer-frame handling.
Detector::Detector(double scale, bool refine)
    : scale_(scale), refine_(refine),
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
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  detector_ = cv::makePtr<cv::aruco::ArucoDetector>(*dictionary_, *parameters_);
#endif
}

// Search all IDs, but refine only selected tags on the original scaled pixel
// grid.
Corners Detector::detect(const cv::Mat &gray,
                         const std::set<int> &selected) const {
  const auto image = resize_gray(gray, scale_);
  Corners result;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  detector_->detectMarkers(image, result.points, result.ids);
#else
  cv::aruco::detectMarkers(image, dictionary_, result.points, result.ids,
                           parameters_);
#endif
  for (std::size_t i = 0; i < result.ids.size(); ++i) {
    if (!refine_ || !selected.count(result.ids[i]))
      continue;
    auto &points = result.points[i];
    float shortest = std::numeric_limits<float>::infinity();
    for (int j = 0; j < 4; ++j) {
      const auto delta = points[j] - points[(j + 3) % 4];
      shortest =
          std::min(shortest, std::sqrt(delta.x * delta.x + delta.y * delta.y));
    }
    // Eight cells including borders: half a cell, retaining the historical 7px
    // cap.
    const int radius =
        std::max(1, std::min(7, static_cast<int>(shortest / 16)));
    cv::cornerSubPix(
        image, points, {radius, radius}, {-1, -1},
        {cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 40, .01});
  }
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
  if (!cv::checkRange(rotation) || !std::isfinite(rms))
    return {};
  return {rotation, translation, rms, true};
}
} // namespace correction_native
