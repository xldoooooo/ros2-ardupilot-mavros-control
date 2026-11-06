// Existing AprilTag dictionary, pixel grid, adaptive refinement and IPPE square
// PnP.
#pragma once
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif
#include <opencv2/calib3d.hpp>
#include <set>

namespace correction_native {
struct Corners {
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> points;
};
struct Pose {
  cv::Mat rotation, translation;
  double reprojection = 0;
  bool valid = false;
};
class Detector {
public:
  // Preserve all existing detector parameters without introducing a second
  // configuration.
  Detector(double scale, bool refine);
  // Search the same scaled image and refine only configured IDs.
  Corners detect(const cv::Mat &gray, const std::set<int> &selected) const;

private:
  double scale_;
  bool refine_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::Ptr<cv::aruco::ArucoDetector> detector_;
#endif
};
// Solve metric square pose with distortion applied exactly once.
Pose estimate_pose(const std::vector<cv::Point2d> &points, double size,
                   const cv::Mat &matrix, const cv::Mat &distortion);
} // namespace correction_native
