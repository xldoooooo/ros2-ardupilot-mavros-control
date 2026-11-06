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
struct SearchStats {
  cv::Rect region;
  bool used_roi = false, fallback = false;
  std::size_t pixels = 0;
  double decode_ms = 0;
};
class Detector {
 public:
  // Preserve all existing detector parameters without introducing a second
  // configuration.
  Detector(double scale, bool refine, bool track_roi = false);
  // Search the same scaled image and refine only configured IDs.
  Corners detect(const cv::Mat &gray, const std::set<int> &selected);
  // Decode the selected original JPEG in native memory, then search without a
  // raw ROS copy.
  Corners detect_jpeg(const cv::Mat &jpeg, int width, int height,
                      const std::set<int> &selected);
  // A new task cannot reuse another camera interval's tracking window.
  void reset();
  // Report the actual scanned area and same-frame recovery for performance
  // analysis.
  SearchStats statistics() const { return stats_; }

 private:
  double scale_;
  bool refine_, track_roi_;
  cv::Rect roi_;
  cv::Size image_size_;
  std::set<int> selected_;
  std::vector<cv::Point2f> previous_points_;
  SearchStats stats_;
  // Preserve full-image perimeter thresholds and restore original coordinates
  // before refinement.
  Corners search(const cv::Mat &image, const cv::Rect &region) const;
  // Keep enough context for a moving/rotating hover target, using observed
  // corner displacement.
  void update_region(const Corners &corners, const std::set<int> &selected,
                     cv::Size size);
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
};
// Solve metric square pose with distortion applied exactly once.
Pose estimate_pose(const std::vector<cv::Point2d> &points, double size,
                   const cv::Mat &matrix, const cv::Mat &distortion);
}  // namespace correction_native
