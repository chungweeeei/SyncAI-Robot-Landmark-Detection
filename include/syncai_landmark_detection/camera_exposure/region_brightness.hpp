#ifndef SYNCAI_LANDMARK_DETECTION__CAMERA_EXPOSURE__REGION_BRIGHTNESS_HPP_
#define SYNCAI_LANDMARK_DETECTION__CAMERA_EXPOSURE__REGION_BRIGHTNESS_HPP_

// How bright one landmark's region of a grey frame is: what ExposurePolicy
// steers on. ROS-free (OpenCV only); test/test_region_brightness.cpp.

#include <opencv2/core.hpp>

#include <optional>
#include <vector>

#include "syncai_landmark_detection/camera_exposure/exposure_policy.hpp"

namespace syncai_landmark_detection
{
namespace camera_exposure
{

struct RegionParams
{
  // The quad is grown about its centre by this before it is measured. A
  // marker's corners are its black border's; what burns out first, and takes
  // the border with it, is the white rim of the print outside them (the
  // dock's has ~4 px of it at 1 m). 1.25 adds one cell of an 8 x 8 marker
  // (6 x 6 code + border) on every side.
  double scale{1.25};
  // A pixel at this grey level or above counts as saturated. Below 255: the
  // camera's JPEG rounds a burnt-out area to a few levels under it.
  int saturated_level{250};
};

/**
 * @brief The brightness of the convex quad `corners` (image pixels, any
 * winding) in `gray`, grown by params.scale and clipped to the frame.
 * @return nullopt when gray is not 8-bit single-channel, corners is not four
 * points, or nothing of the region is inside the frame
 */
std::optional<RegionBrightness> measureRegion(
  const cv::Mat & gray, const std::vector<cv::Point2f> & corners, const RegionParams & params = {});

/**
 * @brief The region that should decide when several landmarks are in one
 * frame: the brighter of each number. One exposure serves them all, and a
 * burnt-out marker is lost while a dim one still reads.
 */
RegionBrightness brighter(const RegionBrightness & a, const RegionBrightness & b);

}  // namespace camera_exposure
}  // namespace syncai_landmark_detection

#endif  // SYNCAI_LANDMARK_DETECTION__CAMERA_EXPOSURE__REGION_BRIGHTNESS_HPP_
