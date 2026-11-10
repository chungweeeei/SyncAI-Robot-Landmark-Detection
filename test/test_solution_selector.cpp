// SolutionSelector on hand-made pose solutions: the choice between two, the
// near-marker refinement and the gate against mirror jumps.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "syncai_landmark_detection/plugins/aruco_detector/solution_selector.hpp"

namespace aruco = syncai_landmark_detection::aruco;

namespace
{

// A marker facing the camera, turned by yaw, at (tx, 0, tz): observed from it,
// x_m = tz, rz = yaw, and y_m = tx when yaw = 0 (see test_marker_geometry).
aruco::MarkerPose markerAt(double tx, double tz, double yaw, double error_px)
{
  const cv::Matx33d facing(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
  const cv::Matx33d turn(
    std::cos(yaw), 0.0, std::sin(yaw), 0.0, 1.0, 0.0, -std::sin(yaw), 0.0, std::cos(yaw));
  aruco::MarkerPose pose;
  pose.rotation = turn * facing;
  pose.translation_m = cv::Vec3d(tx, 0.0, tz);
  pose.reprojection_error_px = error_px;
  return pose;
}

const auto kIdentity = [](const aruco::MarkerPose & p) { return p; };

aruco::SelectorParams noRefine()
{
  aruco::SelectorParams p;
  p.refine_lm = false;
  return p;
}

}  // namespace

TEST(SolutionSelector, OneSolutionIsTaken)
{
  aruco::SolutionSelector s(noRefine());
  const auto out = s.select(0.0, {markerAt(0.0, 1.0, 0.1, 0.2)}, kIdentity);
  ASSERT_TRUE(out);
  EXPECT_EQ(out->choice, aruco::Choice::Single);
  EXPECT_FALSE(out->ambiguous);
  EXPECT_FALSE(out->gated);
}

TEST(SolutionSelector, ImpossibleSolutionsGiveNoPose)
{
  aruco::SolutionSelector s(noRefine());
  // Turned 70 deg, beyond max_rz (60).
  EXPECT_FALSE(s.select(0.0, {markerAt(0.0, 1.0, 70.0 * CV_PI / 180.0, 0.2)}, kIdentity));
}

TEST(SolutionSelector, ADecisiveImagePicksTheBetterSolution)
{
  aruco::SolutionSelector s(noRefine());
  // 1.0 >= 3 x 0.1 + 0.3.
  const auto out =
    s.select(0.0, {markerAt(0.0, 1.5, 0.1, 0.1), markerAt(0.0, 1.5, -0.1, 1.0)}, kIdentity);
  ASSERT_TRUE(out);
  EXPECT_EQ(out->choice, aruco::Choice::Decisive);
  EXPECT_FALSE(out->ambiguous);
  EXPECT_NEAR(aruco::observeFromMarker(out->pose).rz_rad, 0.1, 1e-9);
}

TEST(SolutionSelector, UndecidedSolutionsFollowThePreviousNormalAndSaySo)
{
  aruco::SolutionSelector s(noRefine());
  const auto a = markerAt(0.0, 1.5, 0.1, 0.2);
  const auto b = markerAt(0.0, 1.5, -0.1, 0.25);
  // No hint yet: the lower error, flagged ambiguous.
  auto out = s.select(0.0, {a, b}, kIdentity);
  ASSERT_TRUE(out);
  EXPECT_EQ(out->choice, aruco::Choice::Hint);
  EXPECT_TRUE(out->ambiguous);
  EXPECT_NEAR(aruco::observeFromMarker(out->pose).rz_rad, 0.1, 1e-9);
  // Next frame the errors swap order: the previous normal keeps the same side.
  auto b2 = b;
  b2.reprojection_error_px = 0.15;
  out = s.select(0.033, {b2, a}, kIdentity);
  ASSERT_TRUE(out);
  EXPECT_TRUE(out->ambiguous);
  EXPECT_NEAR(aruco::observeFromMarker(out->pose).rz_rad, 0.1, 1e-9);
}

TEST(SolutionSelector, NearTheMarkerMeetingRefinementsEndTheAmbiguity)
{
  aruco::SolutionSelector s;  // refine_lm on
  const auto a = markerAt(0.0, 0.4, 0.05, 0.2);
  const auto b = markerAt(0.0, 0.4, -0.05, 0.25);
  // Both refine to a: they meet, so the image is decisive.
  const auto out = s.select(0.0, {a, b}, [&](const aruco::MarkerPose &) { return a; });
  ASSERT_TRUE(out);
  EXPECT_TRUE(out->refined);
  EXPECT_EQ(out->choice, aruco::Choice::Merged);
  EXPECT_FALSE(out->ambiguous);
}

TEST(SolutionSelector, FarFromTheMarkerTheIppePoseStands)
{
  aruco::SolutionSelector s;  // refine_lm on
  const auto a = markerAt(0.0, 1.0, 0.05, 0.2);
  int refinements = 0;
  const auto out = s.select(0.0, {a}, [&](const aruco::MarkerPose &) {
    ++refinements;
    return a;
  });
  ASSERT_TRUE(out);
  EXPECT_FALSE(out->refined);
  // 1.0 m is well past refine_exit_x_m (0.65): not even tried.
  EXPECT_EQ(refinements, 0);
}

TEST(SolutionSelector, JustOutsideTheZoneTheRefinementIsStillTried)
{
  aruco::SolutionSelector s;  // refine_lm on
  // IPPE at 0.68 m, inside the skip margin; the refined pose is at 0.55 m and
  // enters the zone (below refine_max_x_m 0.6).
  const auto a = markerAt(0.0, 0.68, 0.05, 0.2);
  const auto refined = markerAt(0.0, 0.55, 0.05, 0.2);
  int refinements = 0;
  const auto out = s.select(0.0, {a}, [&](const aruco::MarkerPose &) {
    ++refinements;
    return refined;
  });
  ASSERT_TRUE(out);
  EXPECT_GE(refinements, 1);
  EXPECT_TRUE(out->refined);
}

TEST(SolutionSelector, TheGateDropsAMirrorJumpUntilItPersists)
{
  aruco::SolutionSelector s(noRefine());
  ASSERT_FALSE(s.select(0.0, {markerAt(0.0, 1.0, 0.0, 0.2)}, kIdentity)->gated);
  // 0.5 m across: beyond gate_y (0.302). Dropped four times, the fifth in a
  // row is real motion.
  const auto jumped = markerAt(0.5, 1.0, 0.0, 0.2);
  for (int i = 1; i <= 4; ++i) {
    EXPECT_TRUE(s.select(0.033 * i, {jumped}, kIdentity)->gated) << i;
  }
  EXPECT_FALSE(s.select(0.033 * 5, {jumped}, kIdentity)->gated);
  // And it is the new reference.
  EXPECT_FALSE(s.select(0.033 * 6, {jumped}, kIdentity)->gated);
}

TEST(SolutionSelector, AGatedMirrorDoesNotBecomeTheHint)
{
  aruco::SolutionSelector s(noRefine());
  const double yaw = 15.0 * CV_PI / 180.0;  // the two sides 30 deg apart: beyond gate_rz (20)
  const auto a = markerAt(0.0, 1.0, yaw, 0.2);
  const auto mirror = markerAt(0.0, 1.0, -yaw, 0.2);
  ASSERT_FALSE(s.select(0.0, {a}, kIdentity)->gated);
  // One frame decisively for the mirror: dropped by the gate.
  auto a_bad = a;
  a_bad.reprojection_error_px = 1.0;  // 1.0 >= 3 x 0.2 + 0.3
  auto out = s.select(0.033, {mirror, a_bad}, kIdentity);
  ASSERT_TRUE(out);
  EXPECT_EQ(out->choice, aruco::Choice::Decisive);
  EXPECT_TRUE(out->gated);
  // The next frame is undecided, the mirror listed first: the hint is still
  // the accepted side, so a is picked and passes.
  auto mirror_low = mirror;
  mirror_low.reprojection_error_px = 0.18;
  out = s.select(0.066, {mirror_low, a}, kIdentity);
  ASSERT_TRUE(out);
  EXPECT_EQ(out->choice, aruco::Choice::Hint);
  EXPECT_FALSE(out->gated);
  EXPECT_NEAR(aruco::observeFromMarker(out->pose).rz_rad, yaw, 1e-9);
}

TEST(SolutionSelector, TheGateStartsOverAfterAGap)
{
  aruco::SolutionSelector s(noRefine());
  ASSERT_FALSE(s.select(0.0, {markerAt(0.0, 1.0, 0.0, 0.2)}, kIdentity)->gated);
  // Nothing accepted for more than gate_reset_sec (0.5): no reference left.
  EXPECT_FALSE(s.select(0.6, {markerAt(0.5, 1.0, 0.0, 0.2)}, kIdentity)->gated);
}

TEST(SolutionSelector, ResetForgetsTheReference)
{
  aruco::SolutionSelector s(noRefine());
  ASSERT_FALSE(s.select(0.0, {markerAt(0.0, 1.0, 0.0, 0.2)}, kIdentity)->gated);
  s.reset();
  EXPECT_FALSE(s.select(0.033, {markerAt(0.5, 1.0, 0.0, 0.2)}, kIdentity)->gated);
}
