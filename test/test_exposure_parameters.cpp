// declareExposurePolicyParameters() on a plain rclcpp node: what a params file
// sets is what ExposurePolicy gets, the ladder above all, whose first rung is
// where every activation starts.

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "syncai_landmark_detection/camera_exposure/exposure_parameters.hpp"
#include "syncai_landmark_detection/camera_exposure/exposure_policy.hpp"

namespace exposure = syncai_landmark_detection::camera_exposure;

namespace
{

const char * const kPrefix = "ArUco";

// The name a params file gives a key of the ArUco plugin's exposure.
std::string fullName(const std::string & key)
{
  return std::string(kPrefix) + ".exposure." + key;
}

// A ladder of the given length whose every rung differs from the defaults'.
std::vector<int64_t> distinctLadder(std::size_t length)
{
  std::vector<int64_t> ladder;
  for (std::size_t i = 0; i < length; ++i) {
    ladder.push_back(5000 + 7 * static_cast<int64_t>(i));
  }
  return ladder;
}

// The ladder as ExposureParams holds it.
std::vector<int> asInts(const std::vector<int64_t> & values)
{
  return std::vector<int>(values.begin(), values.end());
}

class ExposureParametersTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  // A fresh node each time, so no test sees another's declarations.
  static rclcpp::Node::SharedPtr makeNode(const std::vector<rclcpp::Parameter> & overrides = {})
  {
    static int count = 0;
    return std::make_shared<rclcpp::Node>(
      "exposure_parameters_test_" + std::to_string(count++),
      rclcpp::NodeOptions().parameter_overrides(overrides));
  }
};

}  // namespace

TEST_F(ExposureParametersTest, DefaultsWhenTheParamsFileSetsNothing)
{
  const auto params = exposure::declareExposurePolicyParameters(makeNode(), kPrefix);
  const exposure::ExposureParams defaults;
  EXPECT_EQ(params.ladder_us, defaults.ladder_us);
  EXPECT_EQ(params.min_us, defaults.min_us);
  EXPECT_EQ(params.max_us, defaults.max_us);
  EXPECT_EQ(params.settle_frames, defaults.settle_frames);
  EXPECT_EQ(params.miss_frames, defaults.miss_frames);
  EXPECT_EQ(params.lost_frames, defaults.lost_frames);
  EXPECT_DOUBLE_EQ(params.target_p95, defaults.target_p95);
  EXPECT_DOUBLE_EQ(params.lock_p95_min, defaults.lock_p95_min);
  EXPECT_DOUBLE_EQ(params.lock_p95_max, defaults.lock_p95_max);
  EXPECT_DOUBLE_EQ(params.lock_max_saturated, defaults.lock_max_saturated);
  EXPECT_DOUBLE_EQ(params.unlock_max_saturated, defaults.unlock_max_saturated);
  EXPECT_DOUBLE_EQ(params.unlock_p95_min, defaults.unlock_p95_min);
  EXPECT_EQ(params.unlock_frames, defaults.unlock_frames);
  EXPECT_DOUBLE_EQ(params.max_step, defaults.max_step);
}

TEST_F(ExposureParametersTest, TheParamsFileLadderArrivesInOrder)
{
  // The robot's params file's ladder: the search must start at its first rung.
  const std::vector<int64_t> ladder{8333, 4166, 2083, 1000, 16600};
  const auto params = exposure::declareExposurePolicyParameters(
    makeNode({rclcpp::Parameter(fullName("ladder_us"), ladder)}), kPrefix);
  EXPECT_EQ(params.ladder_us, asInts(ladder));
  exposure::ExposurePolicy policy(params);
  EXPECT_EQ(policy.start(), 8300);  // 8333 rounded to the camera's 100 us step
}

TEST_F(ExposureParametersTest, EveryScalarIsRead)
{
  const auto params = exposure::declareExposurePolicyParameters(
    makeNode({
      rclcpp::Parameter(fullName("min_us"), 200),
      rclcpp::Parameter(fullName("max_us"), 12000),
      rclcpp::Parameter(fullName("settle_frames"), 4),
      rclcpp::Parameter(fullName("miss_frames"), 6),
      rclcpp::Parameter(fullName("lost_frames"), 17),
      rclcpp::Parameter(fullName("target_p95"), 185.0),
      rclcpp::Parameter(fullName("lock_p95_min"), 150.0),
      rclcpp::Parameter(fullName("lock_p95_max"), 215.0),
      rclcpp::Parameter(fullName("lock_max_saturated"), 0.03),
      rclcpp::Parameter(fullName("unlock_max_saturated"), 0.12),
      rclcpp::Parameter(fullName("unlock_p95_min"), 90.0),
      rclcpp::Parameter(fullName("unlock_frames"), 5),
      rclcpp::Parameter(fullName("max_step"), 1.5),
    }),
    kPrefix);
  EXPECT_EQ(params.min_us, 200);
  EXPECT_EQ(params.max_us, 12000);
  EXPECT_EQ(params.settle_frames, 4);
  EXPECT_EQ(params.miss_frames, 6);
  EXPECT_EQ(params.lost_frames, 17);
  EXPECT_DOUBLE_EQ(params.target_p95, 185.0);
  EXPECT_DOUBLE_EQ(params.lock_p95_min, 150.0);
  EXPECT_DOUBLE_EQ(params.lock_p95_max, 215.0);
  EXPECT_DOUBLE_EQ(params.lock_max_saturated, 0.03);
  EXPECT_DOUBLE_EQ(params.unlock_max_saturated, 0.12);
  EXPECT_DOUBLE_EQ(params.unlock_p95_min, 90.0);
  EXPECT_EQ(params.unlock_frames, 5);
  EXPECT_DOUBLE_EQ(params.max_step, 1.5);
}

TEST_F(ExposureParametersTest, AnotherDetectorsParametersAreNotRead)
{
  const auto params = exposure::declareExposurePolicyParameters(
    makeNode({rclcpp::Parameter("Other.exposure.ladder_us", distinctLadder(3))}), kPrefix);
  EXPECT_EQ(params.ladder_us, exposure::ExposureParams().ladder_us);
}

TEST_F(ExposureParametersTest, AWronglyTypedLadderIsRefusedAtDeclaration)
{
  // A params file with fractional rungs: refused here, so the server ends the node with the reason.
  EXPECT_THROW(
    exposure::declareExposurePolicyParameters(
      makeNode({rclcpp::Parameter(fullName("ladder_us"), std::vector<double>{8333.0, 4166.5})}),
      kPrefix),
    rclcpp::exceptions::InvalidParameterTypeException);
}

TEST_F(ExposureParametersTest, ASecondCallReadsWhatTheFirstDeclared)
{
  // Declared once, then only read: what a second plugin instance on the node would see.
  const auto node = makeNode({rclcpp::Parameter(fullName("ladder_us"), distinctLadder(4))});
  const auto first = exposure::declareExposurePolicyParameters(node, kPrefix);
  const auto second = exposure::declareExposurePolicyParameters(node, kPrefix);
  EXPECT_EQ(first.ladder_us, asInts(distinctLadder(4)));
  EXPECT_EQ(second.ladder_us, first.ladder_us);
}

TEST_F(ExposureParametersTest, StressLaddersOfEveryLengthReadExactlyUnderAllocationChurn)
{
  // Freed memory of the ladder's size is reused between reads, so a read that
  // outlives the parameter it came from returns the reuse, not the ladder.
  const auto node = makeNode();
  exposure::declareExposurePolicyParameters(node, kPrefix);
  for (int round = 0; round < 10; ++round) {
    for (std::size_t length = 1; length <= 100; ++length) {
      const auto ladder = distinctLadder(length);
      ASSERT_TRUE(node->set_parameter(rclcpp::Parameter(fullName("ladder_us"), ladder)).successful);
      {
        // Same-size blocks allocated and freed, filled with a value no ladder holds.
        std::vector<std::vector<int64_t>> churn;
        for (int block = 0; block < 4; ++block) {
          churn.emplace_back(length, -1);
        }
      }
      const auto params = exposure::declareExposurePolicyParameters(node, kPrefix);
      ASSERT_EQ(params.ladder_us, asInts(ladder)) << "round " << round << ", length " << length;
    }
  }
}
