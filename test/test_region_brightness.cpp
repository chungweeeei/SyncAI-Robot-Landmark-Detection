// measureRegion on hand-painted grey frames: the percentile, the saturated
// share, the grown quad and the frame's edge.

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>

#include "syncai_landmark_detection/camera_exposure/region_brightness.hpp"

namespace exposure = syncai_landmark_detection::camera_exposure;

namespace
{

std::vector<cv::Point2f> square(float x, float y, float side)
{
  return {{x, y}, {x + side, y}, {x + side, y + side}, {x, y + side}};
}

exposure::RegionParams exact()
{
  exposure::RegionParams params;
  params.scale = 1.0;
  return params;
}

}  // namespace

TEST(RegionBrightness, P95IsTheWhiteCellsLevelWhateverShareIsBlack)
{
  // A marker is mostly black: 70 % at 30, 30 % at 200.
  cv::Mat gray(200, 200, CV_8UC1, cv::Scalar(30));
  cv::rectangle(gray, cv::Rect(50, 50, 100, 30), cv::Scalar(200), cv::FILLED);
  const auto b = exposure::measureRegion(gray, square(50.0F, 50.0F, 99.0F), exact());
  ASSERT_TRUE(b.has_value());
  EXPECT_DOUBLE_EQ(b->p95, 200.0);
  EXPECT_DOUBLE_EQ(b->saturated, 0.0);
}

TEST(RegionBrightness, SaturatedIsTheShareAtTheLevelOrAbove)
{
  cv::Mat gray(200, 200, CV_8UC1, cv::Scalar(150));
  // Half of the 100 x 100 region burnt out, at the JPEG's 252 rather than 255.
  cv::rectangle(gray, cv::Rect(50, 50, 100, 50), cv::Scalar(252), cv::FILLED);
  const auto b = exposure::measureRegion(gray, square(50.0F, 50.0F, 99.0F), exact());
  ASSERT_TRUE(b.has_value());
  EXPECT_NEAR(b->saturated, 0.5, 0.01);
  EXPECT_DOUBLE_EQ(b->p95, 252.0);

  exposure::RegionParams strict = exact();
  strict.saturated_level = 255;
  const auto none = exposure::measureRegion(gray, square(50.0F, 50.0F, 99.0F), strict);
  ASSERT_TRUE(none.has_value());
  EXPECT_DOUBLE_EQ(none->saturated, 0.0);
}

TEST(RegionBrightness, TheGrownQuadTakesInThePrintsWhiteRim)
{
  // A black marker with a burnt-out rim just outside its corners.
  cv::Mat gray(200, 200, CV_8UC1, cv::Scalar(80));
  cv::rectangle(gray, cv::Rect(55, 55, 90, 90), cv::Scalar(255), cv::FILLED);
  cv::rectangle(gray, cv::Rect(60, 60, 80, 80), cv::Scalar(20), cv::FILLED);
  const auto corners = square(60.0F, 60.0F, 79.0F);
  const auto inside = exposure::measureRegion(gray, corners, exact());
  ASSERT_TRUE(inside.has_value());
  EXPECT_DOUBLE_EQ(inside->saturated, 0.0);
  const auto grown = exposure::measureRegion(gray, corners);  // 1.25
  ASSERT_TRUE(grown.has_value());
  EXPECT_GT(grown->saturated, 0.1);
}

TEST(RegionBrightness, ARegionOverTheFramesEdgeIsMeasuredOnWhatIsInside)
{
  cv::Mat gray(100, 100, CV_8UC1, cv::Scalar(120));
  const auto b = exposure::measureRegion(gray, square(80.0F, 80.0F, 60.0F), exact());
  ASSERT_TRUE(b.has_value());
  EXPECT_DOUBLE_EQ(b->p95, 120.0);
}

TEST(RegionBrightness, NothingToMeasureIsNotABrightness)
{
  cv::Mat gray(100, 100, CV_8UC1, cv::Scalar(120));
  EXPECT_FALSE(exposure::measureRegion(gray, square(300.0F, 300.0F, 20.0F)).has_value());
  EXPECT_FALSE(exposure::measureRegion(cv::Mat(), square(10.0F, 10.0F, 20.0F)).has_value());
  EXPECT_FALSE(exposure::measureRegion(gray, {{1.0F, 1.0F}, {5.0F, 5.0F}}).has_value());
  cv::Mat colour(100, 100, CV_8UC3, cv::Scalar(1, 2, 3));
  EXPECT_FALSE(exposure::measureRegion(colour, square(10.0F, 10.0F, 20.0F)).has_value());
}

TEST(RegionBrightness, SeveralLandmarksAreJudgedByTheBrightest)
{
  exposure::RegionBrightness dim;
  dim.p95 = 90.0;
  dim.saturated = 0.0;
  exposure::RegionBrightness burnt;
  burnt.p95 = 255.0;
  burnt.saturated = 0.4;
  const auto both = exposure::brighter(dim, burnt);
  EXPECT_DOUBLE_EQ(both.p95, 255.0);
  EXPECT_DOUBLE_EQ(both.saturated, 0.4);
}
