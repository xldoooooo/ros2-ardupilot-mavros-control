// Share the exact detection resize between camera publication and standalone detection.
#pragma once
#include <opencv2/imgproc.hpp>

namespace correction_native {
// Preserve the original fx/fy rounding, INTER_AREA pixels and no-resize path at scale 1.
inline cv::Mat resize_gray(const cv::Mat &gray, double scale) {
  if (scale == 1.) return gray;
  cv::Mat image;
  cv::resize(gray, image, {}, scale, scale, cv::INTER_AREA);
  return image;
}
}  // namespace correction_native
