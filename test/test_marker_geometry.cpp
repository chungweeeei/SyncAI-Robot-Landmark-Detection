// The ArUco geometry core on synthetic markers: a known pose projected
// through a known camera, solved back.

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>

#include <cmath>
#include <vector>

#include "syncai_landmark_detection/plugins/aruco_detector/marker_geometry.hpp"

namespace aruco = syncai_landmark_detection::aruco;

namespace
{

constexpr double kSize = 0.088;  // the charger dock's print

cv::Matx33d cameraK()
{
  // robot01's camera at 1280x720 is about this (the charger's calibration
  // scaled: fx 700.38 fy 698.84 cx 632.78 cy 366.13).
  return cv::Matx33d(700.0, 0.0, 632.0, 0.0, 699.0, 366.0, 0.0, 0.0, 1.0);
}

// A marker facing the camera, turned by yaw about the vertical, at (tx, ty, tz)
// in the optical frame: rz = yaw, x_m = tz and y_m = tx when yaw = 0.
//
// ty is 3 cm by default, not 0, for the tests that solve a pose: with the
// marker centre exactly on the camera's horizontal axis and noise-free
// corners, OpenCV 4.5.4's SOLVEPNP_IPPE_SQUARE (and SOLVEPNP_IPPE) return
// two wrong solutions, both ~43 px off (checked 2026-11-10 in the Dev
// Container image). Any real corner noise lifts it: a degenerate synthetic
// input, not a case the robot meets.
aruco::MarkerPose markerAt(double tx, double tz, double yaw, double ty = 0.03)
{
  const cv::Matx33d facing(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
  const cv::Matx33d turn(
    std::cos(yaw), 0.0, std::sin(yaw), 0.0, 1.0, 0.0, -std::sin(yaw), 0.0, std::cos(yaw));
  aruco::MarkerPose pose;
  pose.rotation = turn * facing;
  pose.translation_m = cv::Vec3d(tx, ty, tz);
  return pose;
}

std::vector<cv::Point3f> objectPoints()
{
  const auto h = static_cast<float>(kSize / 2.0);
  return {{-h, h, 0.0F}, {h, h, 0.0F}, {h, -h, 0.0F}, {-h, -h, 0.0F}};
}

std::vector<cv::Point2f> project(
  const aruco::MarkerPose & pose, const cv::Matx33d & k, const std::vector<double> & d = {})
{
  cv::Vec3d rvec;
  cv::Rodrigues(pose.rotation, rvec);
  std::vector<cv::Point2f> out;
  cv::projectPoints(objectPoints(), rvec, pose.translation_m, k, d, out);
  return out;
}

}  // namespace

TEST(MarkerGeometry, SolvesAKnownPoseBack)
{
  const auto truth = markerAt(0.05, 1.0, 0.2);
  const auto corners = project(truth, cameraK());
  const auto candidates = aruco::estimateCandidates(corners, kSize, cameraK());
  ASSERT_FALSE(candidates.empty());
  // Sorted by reprojection error; the true pose reprojects exactly.
  EXPECT_LT(candidates.front().reprojection_error_px, 0.01);
  EXPECT_LT(cv::norm(candidates.front().translation_m - truth.translation_m), 1e-3);
  for (std::size_t i = 1; i < candidates.size(); ++i) {
    EXPECT_GE(candidates[i].reprojection_error_px, candidates[i - 1].reprojection_error_px);
  }
}

TEST(MarkerGeometry, ObserveFromMarkerSigns)
{
  // Head-on, the marker 5 cm right of the optical axis: the camera stands
  // 5 cm left of the marker's axis (+y), 1 m in front, square to it.
  const auto ahead = aruco::observeFromMarker(markerAt(0.05, 1.0, 0.0));
  EXPECT_NEAR(ahead.x_m, 1.0, 1e-9);
  EXPECT_NEAR(ahead.y_m, 0.05, 1e-9);
  EXPECT_NEAR(ahead.rz_rad, 0.0, 1e-9);
  // Turned: rz follows the turn.
  EXPECT_NEAR(aruco::observeFromMarker(markerAt(0.0, 1.0, 0.3)).rz_rad, 0.3, 1e-9);
  EXPECT_NEAR(aruco::normalTiltRad(markerAt(0.0, 1.0, 0.3)), 0.0, 1e-9);
}

TEST(MarkerGeometry, LandmarkRotationIsOutUpLeft)
{
  // Head-on: +x out of the face is towards the camera (-z optical), +z up is
  // -y optical, and +y is the camera's right.
  const cv::Matx33d r = aruco::landmarkRotation(markerAt(0.0, 1.0, 0.0));
  EXPECT_NEAR(r(2, 0), -1.0, 1e-9);
  EXPECT_NEAR(r(1, 2), -1.0, 1e-9);
  EXPECT_NEAR(r(0, 1), 1.0, 1e-9);
  EXPECT_NEAR(cv::determinant(r), 1.0, 1e-9);
  // A print turned in its own plane does not turn the landmark.
  auto rolled = markerAt(0.0, 1.0, 0.0);
  const cv::Matx33d roll(-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0);
  rolled.rotation = rolled.rotation * roll;
  const cv::Matx33d r2 = aruco::landmarkRotation(rolled);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(r2(i, j), r(i, j), 1e-9);
    }
  }
}

TEST(MarkerGeometry, UndistortsPlumbBobAndEquidistant)
{
  const auto pose = markerAt(0.3, 1.0, 0.1);
  const auto ideal = project(pose, cameraK());

  aruco::CameraModel pinhole{cameraK(), "plumb_bob", {-0.1, 0.02, 0.001, -0.001, 0.0}, 1280, 720};
  ASSERT_EQ(aruco::validateCameraModel(pinhole), "");
  const auto plumb = aruco::undistortCorners(project(pose, cameraK(), pinhole.d), pinhole);
  for (std::size_t i = 0; i < ideal.size(); ++i) {
    EXPECT_NEAR(plumb[i].x, ideal[i].x, 0.05);
    EXPECT_NEAR(plumb[i].y, ideal[i].y, 0.05);
  }

  aruco::CameraModel fisheye{
    cameraK(), "equidistant", {-0.0467, -0.0637, 0.0707, -0.0265}, 1280, 720};
  ASSERT_EQ(aruco::validateCameraModel(fisheye), "");
  cv::Vec3d rvec;
  cv::Rodrigues(pose.rotation, rvec);
  std::vector<cv::Point2f> distorted;
  std::vector<cv::Point3f> object = objectPoints();
  cv::fisheye::projectPoints(
    object, distorted, rvec, pose.translation_m, cameraK(),
    cv::Vec4d(fisheye.d[0], fisheye.d[1], fisheye.d[2], fisheye.d[3]));
  const auto fish = aruco::undistortCorners(distorted, fisheye);
  for (std::size_t i = 0; i < ideal.size(); ++i) {
    EXPECT_NEAR(fish[i].x, ideal[i].x, 0.05);
    EXPECT_NEAR(fish[i].y, ideal[i].y, 0.05);
  }

  // All-zero coefficients: unchanged.
  aruco::CameraModel none{cameraK(), "plumb_bob", {0.0, 0.0, 0.0, 0.0, 0.0}, 1280, 720};
  EXPECT_EQ(aruco::undistortCorners(ideal, none), ideal);
}

TEST(MarkerGeometry, RejectsUnusableCalibrations)
{
  EXPECT_NE(
    aruco::validateCameraModel({cv::Matx33d::zeros(), "plumb_bob", {0, 0, 0, 0, 0}, 1280, 720}),
    "");
  EXPECT_NE(aruco::validateCameraModel({cameraK(), "fisheye", {0, 0, 0, 0}, 1280, 720}), "");
  EXPECT_NE(aruco::validateCameraModel({cameraK(), "equidistant", {0, 0, 0, 0, 0}, 1280, 720}), "");
  EXPECT_NE(aruco::validateCameraModel({cameraK(), "plumb_bob", {0, 0, 0, 0, 0}, 0, 0}), "");
}

TEST(MarkerGeometry, RefineKeepsAnExactPose)
{
  const auto truth = markerAt(-0.02, 0.4, -0.1);
  const auto corners = project(truth, cameraK());
  const auto candidates = aruco::estimateCandidates(corners, kSize, cameraK());
  ASSERT_FALSE(candidates.empty());
  const auto refined = aruco::refine(candidates.front(), corners, kSize, cameraK());
  EXPECT_LT(refined.reprojection_error_px, 0.01);
  EXPECT_LT(cv::norm(refined.translation_m - truth.translation_m), 1e-3);
}

namespace
{

std::vector<cv::Point2f> square(float cx, float cy, float half)
{
  return {
    {cx - half, cy - half}, {cx + half, cy - half}, {cx + half, cy + half}, {cx - half, cy + half}};
}

}  // namespace

TEST(MarkerGeometry, PickMarkerTakesTheLargerDuplicate)
{
  // The 3D board's double outline: two of the same id, area ratio ~0.99.
  const std::vector<int> ids{42, 7, 42};
  const std::vector<std::vector<cv::Point2f>> corners{
    square(100, 100, 49.7F), square(400, 100, 50), square(100, 100, 50)};
  EXPECT_EQ(aruco::pickMarker(ids, corners, 42), 2);
  EXPECT_EQ(aruco::pickMarker(ids, corners, 7), 1);
  EXPECT_EQ(aruco::pickMarker(ids, corners, 5), -1);
}

TEST(MarkerGeometry, PickMarkerTakesTheMarkerInsideAWhiteLine)
{
  // The backlit dock's white line decoding as a larger marker around the
  // real one: area ratio ~0.69, centre inside.
  const std::vector<int> ids{42, 42};
  const std::vector<std::vector<cv::Point2f>> corners{square(300, 200, 60), square(300, 200, 50)};
  EXPECT_EQ(aruco::pickMarker(ids, corners, 42), 1);
}
