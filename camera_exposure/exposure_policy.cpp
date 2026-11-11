#include "syncai_landmark_detection/camera_exposure/exposure_policy.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>

namespace syncai_landmark_detection
{
namespace camera_exposure
{

namespace
{

// The camera's step: V4L2's exposure_time_absolute counts 100 us, and the
// feeder rounds exposure_override_us to it. Rounded here too, so the value
// held and logged is the one the sensor has.
constexpr int kStepUs = 100;

std::string describe(const RegionBrightness & region)
{
  char text[64];
  std::snprintf(
    text, sizeof(text), "region p95 %.0f, %.1f %% saturated", region.p95, region.saturated * 100.0);
  return text;
}

void require(bool ok, const char * what)
{
  if (!ok) {
    throw std::invalid_argument(std::string("exposure: ") + what);
  }
}

}  // namespace

ExposurePolicy::ExposurePolicy(const ExposureParams & params) : params_(params)
{
  require(!params_.ladder_us.empty(), "ladder_us is empty");
  require(
    std::all_of(params_.ladder_us.begin(), params_.ladder_us.end(), [](int us) { return us > 0; }),
    "every ladder_us entry must be > 0");
  require(params_.min_us >= kStepUs, "min_us must be >= 100 (the camera's step)");
  require(params_.max_us >= params_.min_us, "max_us must be >= min_us");
  require(params_.settle_frames >= 0, "settle_frames must be >= 0");
  require(params_.miss_frames >= 1, "miss_frames must be >= 1");
  require(params_.lost_frames >= 1, "lost_frames must be >= 1");
  require(params_.unlock_frames >= 1, "unlock_frames must be >= 1");
  require(
    params_.lock_p95_min < params_.target_p95 && params_.target_p95 < params_.lock_p95_max,
    "target_p95 must lie inside lock_p95_min .. lock_p95_max");
  // A lock that leaves on the margin it arrived with flips on every frame's noise.
  require(
    params_.unlock_p95_min <= params_.lock_p95_min,
    "unlock_p95_min must be <= lock_p95_min (the lock leaves on a wider margin)");
  require(
    params_.lock_max_saturated >= 0.0 &&
      params_.lock_max_saturated <= params_.unlock_max_saturated &&
      params_.unlock_max_saturated <= 1.0,
    "0 <= lock_max_saturated <= unlock_max_saturated <= 1");
  require(params_.max_step > 1.0, "max_step must be > 1");
  start();
}

int ExposurePolicy::start()
{
  state_ = State::SEARCHING;
  rung_ = 0;
  rungs_missed_ = 0;
  exposure_us_ = quantize(params_.ladder_us.front());
  settle_left_ = params_.settle_frames;
  missed_ = 0;
  out_of_band_ = 0;
  search_frames_ = 0;
  return exposure_us_;
}

ExposureStep ExposurePolicy::update(const std::optional<RegionBrightness> & region)
{
  if (state_ == State::SEARCHING) {
    ++search_frames_;
  }
  // Exposed before the last change reached the sensor: neither a sighting nor
  // a miss in one of these says anything about the exposure asked for.
  if (settle_left_ > 0) {
    --settle_left_;
    return {};
  }

  if (state_ == State::LOCKED) {
    if (!region) {
      out_of_band_ = 0;
      if (++missed_ < params_.lost_frames) {
        return {};
      }
      const int held_us = exposure_us_;
      state_ = State::SEARCHING;
      search_frames_ = 0;
      missed_ = 0;
      rungs_missed_ = 0;
      rung_ = rungAfterLoss();
      return set(
        params_.ladder_us[rung_], "not seen for " + std::to_string(params_.lost_frames) +
                                    " frames at the locked " + std::to_string(held_us) + " us");
    }
    missed_ = 0;
    const bool past_margin =
      region->saturated > params_.unlock_max_saturated || region->p95 < params_.unlock_p95_min;
    // Past the margin with nowhere to go (dark at max_us already) stays
    // locked: leaving would only arrive at the same exposure again.
    const auto step = past_margin ? adjust(*region) : std::nullopt;
    if (!step) {
      out_of_band_ = 0;
      return {};
    }
    if (++out_of_band_ < params_.unlock_frames) {
      return {};
    }
    const int held_us = exposure_us_;
    state_ = State::SEARCHING;
    search_frames_ = 0;
    out_of_band_ = 0;
    return set(
      step->exposure_us, step->reason + " for " + std::to_string(params_.unlock_frames) +
                           " frames, leaving the lock at " + std::to_string(held_us) + " us");
  }

  if (!region) {
    if (++missed_ < params_.miss_frames) {
      return {};
    }
    missed_ = 0;
    const int tried_us = exposure_us_;
    rung_ = (rung_ + 1) % params_.ladder_us.size();
    const bool sweep_finished = ++rungs_missed_ >= params_.ladder_us.size();
    if (sweep_finished) {
      rungs_missed_ = 0;
    }
    return set(
      params_.ladder_us[rung_],
      "not seen for " + std::to_string(params_.miss_frames) + " frames at " +
        std::to_string(tried_us) + " us",
      sweep_finished);
  }
  missed_ = 0;
  rungs_missed_ = 0;
  if (const auto step = adjust(*region)) {
    return set(step->exposure_us, step->reason);
  }

  state_ = State::LOCKED;
  out_of_band_ = 0;
  ExposureStep locked;
  locked.event = ExposureStep::Event::LOCKED;
  locked.exposure_us = exposure_us_;
  locked.reason = describe(*region);
  const bool in_band = region->saturated <= params_.lock_max_saturated &&
                       region->p95 >= params_.lock_p95_min && region->p95 <= params_.lock_p95_max;
  if (!in_band) {
    // The marker reads, and the exposure cannot follow it further.
    locked.reason +=
      exposure_us_ <= params_.min_us ? ", at the shortest exposure" : ", at the longest exposure";
  }
  return locked;
}

std::optional<ExposureStep> ExposurePolicy::adjust(const RegionBrightness & region) const
{
  double factor = 1.0;
  std::string reason = describe(region);
  if (region.saturated > params_.lock_max_saturated) {
    // Burnt out: p95 is pinned at 255 and says nothing about how far over the
    // exposure is, so halve and look again.
    factor = 0.5;
    char text[48];
    std::snprintf(text, sizeof(text), " (over %.1f %%)", params_.lock_max_saturated * 100.0);
    reason += text;
  } else if (region.p95 < params_.lock_p95_min || region.p95 > params_.lock_p95_max) {
    factor = std::clamp(
      params_.target_p95 / std::max(region.p95, 1.0), 1.0 / params_.max_step, params_.max_step);
    char text[48];
    std::snprintf(
      text, sizeof(text), " (band %.0f-%.0f)", params_.lock_p95_min, params_.lock_p95_max);
    reason += text;
  } else {
    return std::nullopt;
  }
  const int next_us = quantize(exposure_us_ * factor);
  if (next_us == exposure_us_) {
    return std::nullopt;
  }
  ExposureStep step;
  step.event = ExposureStep::Event::SET;
  step.exposure_us = next_us;
  step.reason = std::move(reason);
  return step;
}

ExposureStep ExposurePolicy::set(int exposure_us, std::string reason, bool sweep_finished)
{
  ExposureStep step;
  step.sweep_finished = sweep_finished;
  step.reason = std::move(reason);
  const int next_us = quantize(exposure_us);
  step.exposure_us = next_us;
  // A rung that rounds to the exposure already set (a one-rung ladder) is
  // nothing to write, and nothing to settle from.
  if (next_us != exposure_us_) {
    step.event = ExposureStep::Event::SET;
    exposure_us_ = next_us;
    settle_left_ = params_.settle_frames;
  }
  return step;
}

int ExposurePolicy::quantize(double exposure_us) const
{
  const double clamped = std::clamp(
    exposure_us, static_cast<double>(params_.min_us), static_cast<double>(params_.max_us));
  const int rounded = static_cast<int>(std::lround(clamped / kStepUs)) * kStepUs;
  return std::clamp(rounded, params_.min_us, params_.max_us);
}

std::size_t ExposurePolicy::rungAfterLoss() const
{
  // The rung nearest the exposure that last read the marker, as a ratio: the
  // light has more likely changed a little than a lot. That exposure itself
  // has just failed for lost_frames, so a rung equal to it is skipped.
  std::size_t nearest = 0;
  double nearest_ratio = 0.0;
  for (std::size_t i = 0; i < params_.ladder_us.size(); ++i) {
    const double ratio = std::abs(
      std::log(static_cast<double>(quantize(params_.ladder_us[i])) / std::max(exposure_us_, 1)));
    if (i == 0 || ratio < nearest_ratio) {
      nearest = i;
      nearest_ratio = ratio;
    }
  }
  if (quantize(params_.ladder_us[nearest]) == exposure_us_) {
    nearest = (nearest + 1) % params_.ladder_us.size();
  }
  return nearest;
}

}  // namespace camera_exposure
}  // namespace syncai_landmark_detection
