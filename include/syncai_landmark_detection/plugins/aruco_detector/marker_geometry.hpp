#ifndef SYNCAI_LANDMARK_DETECTION__PLUGINS__ARUCO_DETECTOR__MARKER_GEOMETRY_HPP_
#define SYNCAI_LANDMARK_DETECTION__PLUGINS__ARUCO_DETECTOR__MARKER_GEOMETRY_HPP_

// The geometry of one square marker seen by one camera: undistorting its
// corners, the PnP pose solutions, and the planar quantities the solution
// choice is made on. ROS-free (OpenCV only), so test/test_marker_geometry.cpp
// covers it without a node.
//
// Ported from SyncAI-Robot-Charger's MarkerPose.{hpp,cpp} (2026-11), which
// took it from charger-main's ArucoDet.cpp. What changed: the intrinsics come
// from the camera's own calibration (camera_info) at the frame's resolution,
// so the charger's scaling of a 1920x1200 calibration and its field-of-view
// pinhole are gone, and the distortion model is the calibration's
// (equidistant or plumb_bob / rational_polynomial), not a fixed fisheye.

#include <opencv2/core.hpp>

#include <optional>
#include <string>
#include <vector>

namespace syncai_landmark_detection
{
namespace aruco
{

/**
 * @brief A calibrated camera, as sensor_msgs/CameraInfo describes it: K, the
 * distortion model and its coefficients, and the resolution K belongs to.
 */
struct CameraModel
{
  cv::Matx33d k;
  std::string distortion_model;
  std::vector<double> d;
  int width{0};
  int height{0};
};

/**
 * @brief Why a calibration cannot be used, or an empty string when it can.
 *
 * Checked once per camera_info rather than per frame. A K with no focal
 * length or a distortion vector of the wrong length for its model would give
 * a pose that is wrong without any error, which is worse than no pose.
 */
std::string validateCameraModel(const CameraModel & model);

/**
 * @brief The corners as a distortion-free pinhole with the same K would have
 * seen them. Only the four corners are undistorted, never the image: four
 * points cost nothing, a 1280x720 remap every frame does.
 *
 * equidistant (the fisheye model) goes through cv::fisheye::undistortPoints,
 * plumb_bob and rational_polynomial through cv::undistortPoints. An all-zero
 * distortion vector returns the corners unchanged.
 */
std::vector<cv::Point2f> undistortCorners(
  const std::vector<cv::Point2f> & corners, const CameraModel & model);

/**
 * @brief One PnP solution: the marker in the camera optical frame (x right,
 * y down, z forward). OpenCV's marker frame: origin at the centre, x along the
 * print's top edge, y up the print, z out of its face towards the viewer.
 */
struct MarkerPose
{
  cv::Vec3d translation_m;
  cv::Matx33d rotation;
  double reprojection_error_px{0.0};
};

/**
 * @brief Every SOLVEPNP_IPPE_SQUARE solution (at most two) on undistorted
 * corners in cv::aruco's order, in front of the camera, sorted by
 * reprojection error, best first. Empty when it fails.
 *
 * A square seen from afar has two poses that reproject almost equally well,
 * mirror images about the viewing ray; both are kept, and which one is real
 * is SolutionSelector's question.
 */
std::vector<MarkerPose> estimateCandidates(
  const std::vector<cv::Point2f> & corners, double marker_size_m, const cv::Matx33d & k);

/**
 * @brief pose refined by Levenberg-Marquardt (cv::solvePnPRefineLM) on the
 * same corners, its reprojection error recomputed; pose itself when that fails.
 *
 * IPPE_SQUARE takes the rotation from the homography at the marker centre,
 * unrefined: near head-on a shape misfit of a fraction of a pixel tilts it.
 * Against Point-LIO (charger runs 77/78, 2026-10-22) IPPE read Rz +1.7 deg
 * high at the dock and LM within 0.2 deg; beyond 0.8 m LM was off by > 3 deg
 * on 24-73 % of frames. So refinement is used near the marker only.
 */
MarkerPose refine(
  const MarkerPose & pose, const std::vector<cv::Point2f> & corners, double marker_size_m,
  const cv::Matx33d & k);

/** @brief The marker normal (out of its face) in the camera frame. */
cv::Vec3d markerNormal(const MarkerPose & pose);

/**
 * @brief How far the normal leans out of the camera's horizontal plane
 * [rad]: ~0 for an upright print seen by a level camera.
 */
double normalTiltRad(const MarkerPose & pose);

/**
 * @brief The camera on the ground plane, in a frame at the marker centre.
 *   x_m    distance from the marker plane along its normal (> 0 in front)
 *   y_m    offset from the marker's axis; + = the camera stands left of it,
 *          seen from the camera facing the marker
 *   rz_rad camera heading relative to facing the marker squarely; + = left
 *
 * The quantities every rule of the solution choice is stated in (the
 * charger's measurements are in them). Taken from the normal and the
 * translation, not from the marker's own x axis: cv::aruco numbers the corners
 * from the print's top-left, so a print mounted turned by 180 degrees flips
 * its x while the normal stays put. Signs checked on the robot (charger,
 * 2026-10-01).
 */
struct CameraInMarker
{
  double x_m{0.0};
  double y_m{0.0};
  double rz_rad{0.0};
};

CameraInMarker observeFromMarker(const MarkerPose & pose);

/**
 * @brief The landmark frame of DetectLandmark.srv in the camera frame:
 * +x out of the marker's face (its normal), +z up, y completing the frame.
 *
 * "Up" is the camera's own up (-y of the optical frame) made orthogonal to
 * the normal, not the print's y axis, for the same reason as
 * observeFromMarker: a print turned in its plane must not turn the landmark.
 * The camera's tilt therefore only rolls the landmark about its normal; the
 * normal itself, the part a dock pose is made from, is exact.
 */
cv::Matx33d landmarkRotation(const MarkerPose & pose);

/**
 * @brief Which of the detected markers is marker `id`: its index in ids, or
 * -1.
 *
 * Found more than once, the largest; but one clearly smaller (area below 0.85
 * of the largest) with its centre inside it is taken instead. Two real cases
 * behind it (charger, 2026-10): the 3D-printed dock board often decodes the
 * same marker twice, two outlines ~1 px apart whose order changes frame to
 * frame (at 1.5 m Rz sd 2.2 deg, Y sd 5 cm; the larger one alone: 0.16 deg,
 * 4 mm); and a backlit dock's thin white line around the print also decodes,
 * ~1 cell out (X read 20 % short), in ~97 % of frames. A duplicate's area
 * ratio is ~0.99, the white line's ~0.69.
 */
int pickMarker(
  const std::vector<int> & ids, const std::vector<std::vector<cv::Point2f>> & corners, int id);

}  // namespace aruco
}  // namespace syncai_landmark_detection

#endif  // SYNCAI_LANDMARK_DETECTION__PLUGINS__ARUCO_DETECTOR__MARKER_GEOMETRY_HPP_
