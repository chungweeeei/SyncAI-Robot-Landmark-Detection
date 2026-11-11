#include "syncai_landmark_detection/camera_exposure/region_brightness.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace syncai_landmark_detection
{
namespace camera_exposure
{

std::optional<RegionBrightness> measureRegion(
  const cv::Mat & gray, const std::vector<cv::Point2f> & corners, const RegionParams & params)
{
  if (gray.empty() || gray.type() != CV_8UC1 || corners.size() != 4) {
    return std::nullopt;
  }
  cv::Point2f centre(0.0F, 0.0F);
  for (const auto & c : corners) {
    centre += c;
  }
  centre *= 0.25F;
  std::vector<cv::Point> grown;
  grown.reserve(corners.size());
  for (const auto & c : corners) {
    const cv::Point2f p = centre + (c - centre) * static_cast<float>(params.scale);
    grown.emplace_back(cvRound(p.x), cvRound(p.y));
  }
  const cv::Rect box = cv::boundingRect(grown) & cv::Rect(cv::Point(0, 0), gray.size());
  if (box.empty()) {
    return std::nullopt;
  }
  // The mask is the box's size, not the frame's: this runs on every frame.
  cv::Mat mask = cv::Mat::zeros(box.size(), CV_8UC1);
  for (auto & p : grown) {
    p -= box.tl();
  }
  cv::fillConvexPoly(mask, grown, cv::Scalar(255));

  std::array<int, 256> histogram{};
  int total = 0;
  const cv::Mat region = gray(box);
  for (int y = 0; y < region.rows; ++y) {
    const uchar * pixel = region.ptr<uchar>(y);
    const uchar * inside = mask.ptr<uchar>(y);
    for (int x = 0; x < region.cols; ++x) {
      if (inside[x]) {
        ++histogram[pixel[x]];
        ++total;
      }
    }
  }
  if (total == 0) {
    return std::nullopt;
  }

  RegionBrightness brightness;
  // The lowest level with 95 % of the region at or below it.
  const int rank = static_cast<int>(std::ceil(0.95 * total));
  int below = 0;
  for (int level = 0; level < 256; ++level) {
    below += histogram[static_cast<std::size_t>(level)];
    if (below >= rank) {
      brightness.p95 = level;
      break;
    }
  }
  int saturated = 0;
  for (int level = std::clamp(params.saturated_level, 0, 255); level < 256; ++level) {
    saturated += histogram[static_cast<std::size_t>(level)];
  }
  brightness.saturated = static_cast<double>(saturated) / total;
  return brightness;
}

RegionBrightness brighter(const RegionBrightness & a, const RegionBrightness & b)
{
  RegionBrightness out;
  out.p95 = std::max(a.p95, b.p95);
  out.saturated = std::max(a.saturated, b.saturated);
  return out;
}

}  // namespace camera_exposure
}  // namespace syncai_landmark_detection
