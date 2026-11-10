#include "syncai_landmark_detection/plugins/aruco_detector/marker_geometry.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace syncai_landmark_detection
{
namespace aruco
{

namespace
{

// The coefficient count of each model sensor_msgs/distortion_models.hpp
// names; 0 for a model OpenCV here cannot undistort.
std::size_t distortionLength(const std::string & model)
{
  if (model == "plumb_bob") {
    return 5;
  }
  if (model == "rational_polynomial") {
    return 8;
  }
  if (model == "equidistant") {
    return 4;
  }
  return 0;
}

// The object-point order SOLVEPNP_IPPE_SQUARE requires (y up, z out), which is
// cv::aruco's corner order.
std::vector<cv::Point3f> markerObjectPoints(double marker_size_m)
{
  // Single precision on purpose (charger, 2026-10-01): with double image
  // points OpenCV 4.5.4's SOLVEPNP_IPPE_SQUARE returned a wrong rotation (its
  // own error estimate 36 px) on a square seen head-on, where float32 points
  // solved it to 0.001 px.
  const auto h = static_cast<float>(marker_size_m / 2.0);
  return {{-h, h, 0.0F}, {h, h, 0.0F}, {h, -h, 0.0F}, {-h, -h, 0.0F}};
}

// RMS over the four corners [px].
double reprojectionErrorPx(
  const std::vector<cv::Point3f> & object_points, const cv::Vec3d & rvec, const cv::Vec3d & tvec,
  const cv::Matx33d & k, const std::vector<cv::Point2f> & image_points)
{
  std::vector<cv::Point2f> reprojected;
  cv::projectPoints(object_points, rvec, tvec, k, cv::noArray(), reprojected);
  double squared = 0.0;
  for (std::size_t i = 0; i < reprojected.size(); ++i) {
    const auto d = reprojected[i] - image_points[i];
    squared += d.x * d.x + d.y * d.y;
  }
  return std::sqrt(squared / static_cast<double>(reprojected.size()));
}

bool finite(const cv::Vec3d & v)
{
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

}  // namespace

std::string validateCameraModel(const CameraModel & model)
{
  if (!(model.k(0, 0) > 0.0) || !(model.k(1, 1) > 0.0)) {
    return "camera_info K has no focal length (an uncalibrated camera?)";
  }
  if (model.width <= 0 || model.height <= 0) {
    return "camera_info has no resolution";
  }
  const std::size_t expected = distortionLength(model.distortion_model);
  if (expected == 0) {
    return "camera_info distortion_model '" + model.distortion_model +
           "' is not one of plumb_bob, rational_polynomial, equidistant";
  }
  if (model.d.size() != expected) {
    return "camera_info has " + std::to_string(model.d.size()) + " distortion coefficients, " +
           model.distortion_model + " takes " + std::to_string(expected);
  }
  return "";
}

std::vector<cv::Point2f> undistortCorners(
  const std::vector<cv::Point2f> & corners, const CameraModel & model)
{
  if (corners.empty() || std::all_of(model.d.begin(), model.d.end(), [](double c) {
        return c == 0.0;
      })) {
    return corners;
  }
  std::vector<cv::Point2f> out;
  if (model.distortion_model == "equidistant") {
    const cv::Vec4d d(model.d[0], model.d[1], model.d[2], model.d[3]);
    cv::fisheye::undistortPoints(corners, out, model.k, d, cv::noArray(), model.k);
  } else {
    cv::undistortPoints(corners, out, model.k, model.d, cv::noArray(), model.k);
  }
  return out;
}

std::vector<MarkerPose> estimateCandidates(
  const std::vector<cv::Point2f> & corners, double marker_size_m, const cv::Matx33d & k)
{
  std::vector<MarkerPose> candidates;
  if (corners.size() != 4U || marker_size_m <= 0.0 || !(k(0, 0) > 0.0)) {
    return candidates;
  }
  const std::vector<cv::Point3f> object_points = markerObjectPoints(marker_size_m);

  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  try {
    if (
      cv::solvePnPGeneric(
        object_points, corners, k, cv::noArray(), rvecs, tvecs, false, cv::SOLVEPNP_IPPE_SQUARE) <
      1) {
      return candidates;
    }
  } catch (const cv::Exception &) {
    return candidates;
  }

  for (std::size_t i = 0; i < rvecs.size() && i < tvecs.size(); ++i) {
    const cv::Vec3d rvec(rvecs[i].at<double>(0), rvecs[i].at<double>(1), rvecs[i].at<double>(2));
    const cv::Vec3d tvec(tvecs[i].at<double>(0), tvecs[i].at<double>(1), tvecs[i].at<double>(2));
    if (!(tvec[2] > 0.0) || !finite(tvec) || !finite(rvec)) {
      continue;
    }
    MarkerPose pose;
    pose.translation_m = tvec;
    cv::Rodrigues(rvec, pose.rotation);
    pose.reprojection_error_px = reprojectionErrorPx(object_points, rvec, tvec, k, corners);
    candidates.push_back(pose);
  }
  std::sort(candidates.begin(), candidates.end(), [](const MarkerPose & a, const MarkerPose & b) {
    return a.reprojection_error_px < b.reprojection_error_px;
  });
  return candidates;
}

MarkerPose refine(
  const MarkerPose & pose, const std::vector<cv::Point2f> & corners, double marker_size_m,
  const cv::Matx33d & k)
{
  if (corners.size() != 4U || marker_size_m <= 0.0 || !(k(0, 0) > 0.0)) {
    return pose;
  }
  const std::vector<cv::Point3f> object_points = markerObjectPoints(marker_size_m);
  cv::Vec3d rvec;
  cv::Rodrigues(pose.rotation, rvec);
  cv::Vec3d tvec = pose.translation_m;
  try {
    cv::solvePnPRefineLM(object_points, corners, k, cv::noArray(), rvec, tvec);
  } catch (const cv::Exception &) {
    return pose;
  }
  if (!(tvec[2] > 0.0) || !finite(tvec) || !finite(rvec)) {
    return pose;
  }
  MarkerPose refined;
  refined.translation_m = tvec;
  cv::Rodrigues(rvec, refined.rotation);
  refined.reprojection_error_px = reprojectionErrorPx(object_points, rvec, tvec, k, corners);
  return refined;
}

cv::Vec3d markerNormal(const MarkerPose & pose)
{
  return cv::Vec3d(pose.rotation(0, 2), pose.rotation(1, 2), pose.rotation(2, 2));
}

double normalTiltRad(const MarkerPose & pose)
{
  return std::asin(std::min(1.0, std::abs(pose.rotation(1, 2))));
}

CameraInMarker observeFromMarker(const MarkerPose & pose)
{
  const cv::Vec3d & t = pose.translation_m;
  const cv::Matx33d & r = pose.rotation;
  // The camera in the marker's frame: z along the normal towards the viewer.
  const cv::Vec3d camera_in_marker = -(r.t() * t);
  const cv::Vec3d normal = markerNormal(pose);

  CameraInMarker out;
  out.x_m = camera_in_marker[2];
  // The offset across the axis on the ground plane, from the normal and the
  // translation rather than the marker's own x (see the header).
  const double ground_normal = std::hypot(normal[0], normal[2]);
  out.y_m = ground_normal > 0.0 ? (t[2] * normal[0] - t[0] * normal[2]) / ground_normal : 0.0;
  out.rz_rad = -std::atan2(normal[0], -normal[2]);
  return out;
}

cv::Matx33d landmarkRotation(const MarkerPose & pose)
{
  const cv::Vec3d x = cv::normalize(markerNormal(pose));
  // The camera's up, -y of the optical frame, with its component along the
  // normal removed. A normal pointing straight up or down (a print on the
  // floor or the ceiling, never a dock) leaves nothing; the camera's forward
  // stands in so the frame is still a rotation.
  cv::Vec3d up(0.0, -1.0, 0.0);
  cv::Vec3d z = up - up.dot(x) * x;
  if (cv::norm(z) < 1e-6) {
    const cv::Vec3d forward(0.0, 0.0, 1.0);
    z = forward - forward.dot(x) * x;
  }
  z = cv::normalize(z);
  const cv::Vec3d y = z.cross(x);
  return cv::Matx33d(x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]);
}

int pickMarker(
  const std::vector<int> & ids, const std::vector<std::vector<cv::Point2f>> & corners, int id)
{
  // Below this area ratio a nested outline is another one, not a duplicate.
  constexpr double kNestedAreaRatio = 0.85;
  int best = -1;
  double best_area = 0.0;
  for (std::size_t i = 0; i < ids.size() && i < corners.size(); ++i) {
    if (ids[i] != id) {
      continue;
    }
    const double area = cv::contourArea(corners[i]);
    if (best < 0 || area > best_area) {
      best = static_cast<int>(i);
      best_area = area;
    }
  }
  int inner = -1;
  double inner_area = 0.0;
  for (std::size_t i = 0; best >= 0 && i < ids.size() && i < corners.size(); ++i) {
    if (ids[i] != id || static_cast<int>(i) == best) {
      continue;
    }
    const double area = cv::contourArea(corners[i]);
    const cv::Point2f centre =
      (corners[i][0] + corners[i][1] + corners[i][2] + corners[i][3]) * 0.25F;
    if (
      area < kNestedAreaRatio * best_area && area > inner_area &&
      cv::pointPolygonTest(corners[best], centre, false) > 0.0) {
      inner = static_cast<int>(i);
      inner_area = area;
    }
  }
  return inner >= 0 ? inner : best;
}

}  // namespace aruco
}  // namespace syncai_landmark_detection
