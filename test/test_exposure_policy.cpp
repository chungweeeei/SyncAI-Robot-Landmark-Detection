// ExposurePolicy against hand-made region brightnesses and a simulated camera:
// the ladder while the landmark is out of sight, the convergence once it is
// in sight, and the lock that holds the exposure through an approach.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <random>
#include <stdexcept>
#include <vector>

#include "syncai_landmark_detection/camera_exposure/exposure_policy.hpp"

namespace exposure = syncai_landmark_detection::camera_exposure;
using Event = exposure::ExposureStep::Event;
using State = exposure::ExposurePolicy::State;

namespace
{

const std::optional<exposure::RegionBrightness> kUnseen = std::nullopt;

exposure::RegionBrightness region(double p95, double saturated)
{
  exposure::RegionBrightness r;
  r.p95 = p95;
  r.saturated = saturated;
  return r;
}

// Feeds frames until the policy says something, at most `limit` of them.
exposure::ExposureStep runUntilEvent(
  exposure::ExposurePolicy & policy, const std::optional<exposure::RegionBrightness> & seen,
  int limit, int * frames = nullptr)
{
  for (int i = 1; i <= limit; ++i) {
    const auto step = policy.update(seen);
    if (step.event != Event::NONE) {
      if (frames) {
        *frames = i;
      }
      return step;
    }
  }
  return {};
}

// A marker whose brightness follows the exposure more slowly than linearly,
// as the charger measured on this sensor (whole-frame means 108 / 70 / 42 /
// 21 at 8333 / 4000 / 2000 / 1000 us), and that reads only while its white
// cells are neither burnt into the border nor lost in the noise. A written
// exposure shows `latency` frames later.
class SimulatedCamera
{
public:
  SimulatedCamera(double level_at_8333, int latency, int initial_us)
  : level_at_8333_(level_at_8333), pending_(static_cast<std::size_t>(latency), initial_us)
  {
    pending_.push_back(initial_us);
  }

  void write(int exposure_us) { pending_.back() = exposure_us; }

  // The next frame: the region when the marker reads in it.
  std::optional<exposure::RegionBrightness> frame()
  {
    const int exposure_us = pending_.front();
    pending_.pop_front();
    pending_.push_back(pending_.empty() ? exposure_us : pending_.back());
    shown_us_ = exposure_us;
    const double raw = level_at_8333_ * std::pow(exposure_us / 8333.0, 0.8);
    exposure::RegionBrightness r;
    r.p95 = std::min(raw, 255.0);
    // Past white the cells bloom: the further over, the more of the region.
    r.saturated = raw <= 250.0 ? 0.0 : std::min(0.6, (raw - 250.0) / 250.0);
    last_ = r;
    const bool readable = r.saturated < 0.3 && r.p95 > 40.0;
    return readable ? std::optional<exposure::RegionBrightness>(r) : std::nullopt;
  }

  int shownUs() const { return shown_us_; }
  const exposure::RegionBrightness & last() const { return last_; }

private:
  double level_at_8333_;
  std::deque<int> pending_;
  int shown_us_{0};
  exposure::RegionBrightness last_;
};

}  // namespace

TEST(ExposurePolicy, StartsOnTheFirstRungRoundedToTheCamerasStep)
{
  exposure::ExposurePolicy policy;
  EXPECT_EQ(policy.start(), 8300);
  EXPECT_EQ(policy.exposureUs(), 8300);
  EXPECT_EQ(policy.state(), State::SEARCHING);
}

TEST(ExposurePolicy, FramesFromBeforeAChangeDecideNothing)
{
  exposure::ExposureParams params;
  params.settle_frames = 3;
  exposure::ExposurePolicy policy(params);
  policy.start();
  // Burnt out in every settle frame: still nothing, they show the old exposure.
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(policy.update(region(255.0, 0.5)).event, Event::NONE) << i;
  }
  const auto step = policy.update(region(255.0, 0.5));
  EXPECT_EQ(step.event, Event::SET);
  EXPECT_EQ(step.exposure_us, 4200);  // 8300 / 2, rounded to 100 us
  // And the frames after that change settle again.
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(policy.update(region(255.0, 0.5)).event, Event::NONE) << i;
  }
}

TEST(ExposurePolicy, OutOfSightItWalksTheLadderAndSaysWhenASweepFoundNothing)
{
  exposure::ExposurePolicy policy;
  policy.start();
  const std::vector<int> expected = {4200, 2100, 1000, 16600, 8300, 4200};
  for (std::size_t i = 0; i < expected.size(); ++i) {
    int frames = 0;
    const auto step = runUntilEvent(policy, kUnseen, 100, &frames);
    ASSERT_EQ(step.event, Event::SET) << i;
    EXPECT_EQ(step.exposure_us, expected[i]) << i;
    // settle_frames ignored, then miss_frames without the landmark.
    EXPECT_EQ(frames, 3 + 5) << i;
    // Five rungs tried: the fifth step is the one that closes the sweep.
    EXPECT_EQ(step.sweep_finished, i == 4) << i;
    EXPECT_EQ(policy.state(), State::SEARCHING);
  }
}

TEST(ExposurePolicy, ASightingBetweenRungsStartsTheSweepCountAgain)
{
  exposure::ExposureParams params;
  params.settle_frames = 0;
  params.miss_frames = 1;
  exposure::ExposurePolicy policy(params);
  policy.start();
  for (int i = 0; i < 3; ++i) {
    EXPECT_FALSE(policy.update(kUnseen).sweep_finished);
  }
  EXPECT_EQ(policy.update(region(190.0, 0.0)).event, Event::LOCKED);
}

TEST(ExposurePolicy, InTheBandItLocksAtOnce)
{
  exposure::ExposureParams params;
  params.settle_frames = 0;
  exposure::ExposurePolicy policy(params);
  policy.start();
  const auto step = policy.update(region(200.0, 0.01));
  EXPECT_EQ(step.event, Event::LOCKED);
  EXPECT_EQ(step.exposure_us, 8300);
  EXPECT_EQ(policy.state(), State::LOCKED);
}

TEST(ExposurePolicy, ADimRegionIsScaledTowardsTheTargetByAtMostMaxStep)
{
  exposure::ExposureParams params;
  params.settle_frames = 0;
  params.ladder_us = {2000};
  exposure::ExposurePolicy policy(params);
  policy.start();
  // 190 / 150 of 2000 us.
  auto step = policy.update(region(150.0, 0.0));
  ASSERT_EQ(step.event, Event::SET);
  EXPECT_EQ(step.exposure_us, 2500);
  // 190 / 20 would be 9.5x: one step is capped at 2x.
  step = policy.update(region(20.0, 0.0));
  ASSERT_EQ(step.event, Event::SET);
  EXPECT_EQ(step.exposure_us, 5000);
}

TEST(ExposurePolicy, ItConvergesOnMarkersFromDimToSunlit)
{
  // The white cells' level at 8333 us, from a marker darker than the room to
  // one in direct sun (several times over white). Not further: a marker that
  // is still unreadable at the ladder's shortest rung is never metered, so
  // nothing brings the exposure down to it (the ladder is what bounds this).
  for (const double level : {60.0, 110.0, 190.0, 300.0, 600.0, 1500.0}) {
    for (const int latency : {0, 1, 3}) {
      exposure::ExposurePolicy policy;
      SimulatedCamera camera(level, latency, 23400);  // auto exposure's value on robot01
      camera.write(policy.start());
      int frames = 0;
      while (policy.state() != State::LOCKED && frames < 300) {
        const auto step = policy.update(camera.frame());
        if (step.event == Event::SET) {
          camera.write(step.exposure_us);
        }
        ++frames;
      }
      ASSERT_EQ(policy.state(), State::LOCKED) << "level " << level << " latency " << latency;
      // Ten seconds at 30 Hz is the worst a caller should wait; most take a few steps.
      EXPECT_LT(frames, 300);
      // Locked on what the camera shows, not on a frame from before the change.
      EXPECT_EQ(camera.shownUs(), policy.exposureUs()) << "level " << level;
      const auto & r = camera.last();
      const bool at_limit = policy.exposureUs() == 100 || policy.exposureUs() == 16600;
      if (!at_limit) {
        EXPECT_GE(r.p95, 160.0) << "level " << level << " latency " << latency;
        EXPECT_LE(r.p95, 220.0) << "level " << level << " latency " << latency;
        EXPECT_LE(r.saturated, 0.02) << "level " << level << " latency " << latency;
      }
      // Held from here: the same scene never moves it again.
      for (int i = 0; i < 100; ++i) {
        EXPECT_EQ(policy.update(camera.frame()).event, Event::NONE);
      }
    }
  }
}

TEST(ExposurePolicy, LockedItHoldsBetweenTheLockBandAndTheUnlockMargins)
{
  exposure::ExposureParams params;
  params.settle_frames = 0;
  exposure::ExposurePolicy policy(params);
  policy.start();
  ASSERT_EQ(policy.update(region(190.0, 0.0)).event, Event::LOCKED);
  // Outside the band it arrived in (160-220, 2 %), inside the margins it
  // leaves on (100, 10 %): the robot walking up to the marker.
  for (int i = 0; i < 200; ++i) {
    EXPECT_EQ(policy.update(region(120.0, 0.0)).event, Event::NONE);
    EXPECT_EQ(policy.update(region(240.0, 0.08)).event, Event::NONE);
  }
  EXPECT_EQ(policy.state(), State::LOCKED);
  EXPECT_EQ(policy.exposureUs(), 8300);
}

TEST(ExposurePolicy, LockedItLeavesOnlyAfterSeveralBurntFramesInARow)
{
  exposure::ExposureParams params;
  params.settle_frames = 0;
  exposure::ExposurePolicy policy(params);
  policy.start();
  ASSERT_EQ(policy.update(region(190.0, 0.0)).event, Event::LOCKED);
  // Two burnt frames, one good one: a glint, not a change of light.
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(policy.update(region(255.0, 0.3)).event, Event::NONE);
    EXPECT_EQ(policy.update(region(255.0, 0.3)).event, Event::NONE);
    EXPECT_EQ(policy.update(region(190.0, 0.0)).event, Event::NONE);
  }
  EXPECT_EQ(policy.update(region(255.0, 0.3)).event, Event::NONE);
  EXPECT_EQ(policy.update(region(255.0, 0.3)).event, Event::NONE);
  const auto step = policy.update(region(255.0, 0.3));
  ASSERT_EQ(step.event, Event::SET);
  EXPECT_EQ(step.exposure_us, 4200);
  EXPECT_EQ(policy.state(), State::SEARCHING);
}

TEST(ExposurePolicy, LockedAndTooDarkItLengthensTheExposure)
{
  exposure::ExposureParams params;
  params.settle_frames = 0;
  params.ladder_us = {4000};
  exposure::ExposurePolicy policy(params);
  policy.start();
  ASSERT_EQ(policy.update(region(190.0, 0.0)).event, Event::LOCKED);
  EXPECT_EQ(policy.update(region(80.0, 0.0)).event, Event::NONE);
  EXPECT_EQ(policy.update(region(80.0, 0.0)).event, Event::NONE);
  const auto step = policy.update(region(80.0, 0.0));
  ASSERT_EQ(step.event, Event::SET);
  EXPECT_EQ(step.exposure_us, 8000);  // 190 / 80 capped at 2x
}

TEST(ExposurePolicy, LockedAndLostItSearchesFromTheRungNearestTheHeldExposure)
{
  exposure::ExposureParams params;
  params.settle_frames = 0;
  exposure::ExposurePolicy policy(params);
  policy.start();
  // Walk to 2100 us and lock there.
  ASSERT_EQ(policy.update(region(255.0, 0.5)).exposure_us, 4200);
  ASSERT_EQ(policy.update(region(255.0, 0.5)).exposure_us, 2100);
  ASSERT_EQ(policy.update(region(190.0, 0.0)).event, Event::LOCKED);
  // lost_frames - 1 misses: still held.
  for (int i = 0; i < 14; ++i) {
    EXPECT_EQ(policy.update(kUnseen).event, Event::NONE) << i;
  }
  const auto step = policy.update(kUnseen);
  ASSERT_EQ(step.event, Event::SET);
  // 2100 is a rung and has just failed: the next one, not back to 8300.
  EXPECT_EQ(step.exposure_us, 1000);
  EXPECT_EQ(policy.state(), State::SEARCHING);
  // And the sweep goes on from there.
  EXPECT_EQ(runUntilEvent(policy, kUnseen, 100).exposure_us, 16600);
}

TEST(ExposurePolicy, AMissWhileLockedForgetsTheBurntFramesBeforeIt)
{
  exposure::ExposureParams params;
  params.settle_frames = 0;
  exposure::ExposurePolicy policy(params);
  policy.start();
  ASSERT_EQ(policy.update(region(190.0, 0.0)).event, Event::LOCKED);
  EXPECT_EQ(policy.update(region(255.0, 0.3)).event, Event::NONE);
  EXPECT_EQ(policy.update(region(255.0, 0.3)).event, Event::NONE);
  EXPECT_EQ(policy.update(kUnseen).event, Event::NONE);
  EXPECT_EQ(policy.update(region(255.0, 0.3)).event, Event::NONE);
  EXPECT_EQ(policy.state(), State::LOCKED);
}

TEST(ExposurePolicy, AtALimitItLocksOnWhatItHasAndSaysSo)
{
  exposure::ExposureParams params;
  params.settle_frames = 0;
  params.ladder_us = {16600};
  exposure::ExposurePolicy policy(params);
  policy.start();
  // Dark at the longest exposure there is: nothing to change, the marker reads.
  auto step = policy.update(region(90.0, 0.0));
  EXPECT_EQ(step.event, Event::LOCKED);
  EXPECT_NE(step.reason.find("longest"), std::string::npos) << step.reason;
  // And it does not leave a lock it could only arrive at again.
  for (int i = 0; i < 50; ++i) {
    EXPECT_EQ(policy.update(region(90.0, 0.0)).event, Event::NONE);
  }
  EXPECT_EQ(policy.state(), State::LOCKED);

  params.ladder_us = {100};
  exposure::ExposurePolicy shortest(params);
  shortest.start();
  step = shortest.update(region(255.0, 0.2));
  EXPECT_EQ(step.event, Event::LOCKED);
  EXPECT_NE(step.reason.find("shortest"), std::string::npos) << step.reason;
}

TEST(ExposurePolicy, TheReasonCarriesTheNumbersThatDecided)
{
  exposure::ExposureParams params;
  params.settle_frames = 0;
  exposure::ExposurePolicy policy(params);
  policy.start();
  const auto burnt = policy.update(region(255.0, 0.31));
  EXPECT_NE(burnt.reason.find("31.0 % saturated"), std::string::npos) << burnt.reason;
  EXPECT_NE(burnt.reason.find("p95 255"), std::string::npos) << burnt.reason;
  exposure::ExposurePolicy unseen(params);
  unseen.start();
  const auto missed = runUntilEvent(unseen, kUnseen, 100);
  EXPECT_NE(missed.reason.find("not seen for 5 frames at 8300 us"), std::string::npos)
    << missed.reason;
}

TEST(ExposurePolicy, StartForgetsTheLastActivation)
{
  exposure::ExposureParams params;
  params.settle_frames = 0;
  exposure::ExposurePolicy policy(params);
  policy.start();
  ASSERT_EQ(policy.update(region(255.0, 0.5)).exposure_us, 4200);
  ASSERT_EQ(policy.update(region(190.0, 0.0)).event, Event::LOCKED);
  EXPECT_EQ(policy.start(), 8300);
  EXPECT_EQ(policy.state(), State::SEARCHING);
  EXPECT_EQ(policy.searchFrames(), 0);
}

TEST(ExposurePolicy, ParametersThatCannotWorkAreRefusedByName)
{
  const auto refused = [](const exposure::ExposureParams & params, const char * name) {
    try {
      exposure::ExposurePolicy policy(params);
    } catch (const std::invalid_argument & e) {
      return std::string(e.what()).find(name) != std::string::npos;
    }
    return false;
  };
  exposure::ExposureParams params;
  params.ladder_us.clear();
  EXPECT_TRUE(refused(params, "ladder_us"));
  params = {};
  params.ladder_us = {8333, 0};
  EXPECT_TRUE(refused(params, "ladder_us"));
  params = {};
  params.min_us = 50;
  EXPECT_TRUE(refused(params, "min_us"));
  params = {};
  params.max_us = 50;
  EXPECT_TRUE(refused(params, "max_us"));
  params = {};
  params.target_p95 = 230.0;
  EXPECT_TRUE(refused(params, "target_p95"));
  params = {};
  params.unlock_p95_min = 170.0;
  EXPECT_TRUE(refused(params, "unlock_p95_min"));
  params = {};
  params.unlock_max_saturated = 0.01;
  EXPECT_TRUE(refused(params, "unlock_max_saturated"));
  params = {};
  params.max_step = 1.0;
  EXPECT_TRUE(refused(params, "max_step"));
  params = {};
  params.miss_frames = 0;
  EXPECT_TRUE(refused(params, "miss_frames"));
}

TEST(ExposurePolicy, WhateverItIsShownItStaysInsideTheCamerasRangeAndWaitsOutEveryChange)
{
  std::mt19937 rng(20261111);
  std::uniform_real_distribution<double> level(0.0, 255.0);
  std::uniform_real_distribution<double> share(0.0, 1.0);
  const exposure::ExposureParams params;
  exposure::ExposurePolicy policy(params);
  policy.start();
  int since_set = params.settle_frames;  // start() is a change too
  int sets = 0;
  for (int i = 0; i < 100000; ++i) {
    std::optional<exposure::RegionBrightness> seen;
    if (share(rng) < 0.7) {
      // Mostly dark, sometimes burnt: both ends of the range get exercised.
      seen = region(level(rng), share(rng) < 0.3 ? share(rng) : 0.0);
    }
    const int before_us = policy.exposureUs();
    const auto step = policy.update(seen);
    const int us = policy.exposureUs();
    ASSERT_GE(us, params.min_us) << i;
    ASSERT_LE(us, params.max_us) << i;
    ASSERT_EQ(us % 100, 0) << i;
    if (step.event == Event::SET) {
      ASSERT_EQ(step.exposure_us, us) << i;
      ASSERT_NE(us, before_us) << i;
      // Never from a frame exposed before the last change.
      ASSERT_EQ(since_set, 0) << i;
      since_set = params.settle_frames;
      ++sets;
    } else {
      ASSERT_EQ(us, before_us) << i;
      since_set = std::max(0, since_set - 1);
    }
  }
  EXPECT_GT(sets, 100);
}
