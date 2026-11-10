#include "syncai_landmark_detection/plugins/aruco_detector/solution_selector.hpp"

#include <cmath>

namespace syncai_landmark_detection
{
namespace aruco
{

namespace
{

// How far past refine_exit_x_m the IPPE pose may be and still be refined. The
// refinement settles the rotation, the part two solutions disagree on; the
// distance it hardly moves, so 5 cm is room, not a measured bound. Too small
// would only skip a refinement that could have entered the zone.
constexpr double kRefineSkipMargin_m = 0.05;

double wrap(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

}  // namespace

std::optional<Selection> SolutionSelector::select(
  double t_sec, const std::vector<MarkerPose> & candidates,
  const std::function<MarkerPose(const MarkerPose &)> & refine_fn)
{
  // 1 Geometry: drop the impossible solutions.
  std::vector<MarkerPose> ok;
  std::optional<MarkerPose> rejected;
  for (const auto & c : candidates) {
    const auto seen = observeFromMarker(c);
    if (
      seen.x_m > 0.0 && std::abs(seen.rz_rad) <= params_.max_rz_rad &&
      normalTiltRad(c) <= params_.max_tilt_rad) {
      ok.push_back(c);
    } else if (!rejected) {
      rejected = c;
    }
  }
  if (ok.empty()) {
    miss(t_sec);
    return std::nullopt;
  }

  // 2 One left: that one. Two: a decisive image picks; otherwise the previous
  // normal does (the hint), and the pick is ambiguous.
  Selection out;
  std::optional<MarkerPose> alternative;
  std::size_t index = 0;
  if (ok.size() == 1U) {
    out.choice = Choice::Single;
    alternative = rejected;
  } else {
    const bool decisive =
      ok[1].reprojection_error_px >=
      params_.decisive_ratio * ok[0].reprojection_error_px + params_.decisive_margin_px;
    if (decisive) {
      out.choice = Choice::Decisive;
    } else {
      out.choice = Choice::Hint;
      out.ambiguous = true;
      if (
        last_normal_ &&
        markerNormal(ok[1]).dot(*last_normal_) > markerNormal(ok[0]).dot(*last_normal_)) {
        index = 1;
      }
    }
    alternative = ok[1 - index];
  }
  out.pose = ok[index];

  // 3 Near the marker: the LM-refined pose, and two refined solutions that
  // meet are one pose -- the ambiguity is gone.
  if (
    params_.refine_lm &&
    observeFromMarker(out.pose).x_m > params_.refine_exit_x_m + kRefineSkipMargin_m) {
    // Clearly outside the zone on the IPPE pose already: the refined one
    // would be too, and decideRefined() would throw both refinements away.
    // That is most of an approach, two solvePnPRefineLM a frame for nothing.
    refine_zone_ = false;
  } else if (params_.refine_lm) {
    const MarkerPose refined = refine_fn(out.pose);
    std::optional<MarkerPose> refined_alt;
    if (alternative) {
      refined_alt = refine_fn(*alternative);
    }
    if (decideRefined(refined, refined_alt, out)) {
      out.pose = refined;
      out.refined = true;
    }
  } else {
    refine_zone_ = false;
  }

  // 4 The gate against mirror jumps.
  out.gated = !gate(observeFromMarker(out.pose), t_sec);

  // The hint follows accepted poses only. Taken before the gate, a dropped
  // mirror became the hint: the next undecided frames chose the mirror by it,
  // were dropped too, and after gate_relock of them the gate re-locked onto
  // the mirror branch as if the robot had moved.
  if (!out.gated) {
    last_normal_ = markerNormal(out.pose);
    last_normal_t_ = t_sec;
  }
  return out;
}

bool SolutionSelector::decideRefined(
  const MarkerPose & refined, const std::optional<MarkerPose> & refined_alt, Selection & out)
{
  const auto seen = observeFromMarker(refined);
  refine_zone_ =
    refine_zone_ ? seen.x_m <= params_.refine_exit_x_m : seen.x_m < params_.refine_max_x_m;
  if (!refine_zone_) {
    return false;
  }
  if (refined_alt) {
    const auto other = observeFromMarker(*refined_alt);
    if (
      std::abs(wrap(other.rz_rad - seen.rz_rad)) < params_.merge_rz_rad &&
      std::abs(other.y_m - seen.y_m) < params_.merge_y_m) {
      if (out.choice == Choice::Hint) {
        out.choice = Choice::Merged;
      }
      out.ambiguous = false;
    }
  }
  return true;
}

bool SolutionSelector::gate(const CameraInMarker & seen, double t_sec)
{
  if (!accepted_ || t_sec - accepted_t_ > params_.gate_reset_sec) {
    accepted_ = seen;
    accepted_t_ = t_sec;
    gate_rejects_ = 0;
    return true;
  }
  if (
    std::abs(seen.y_m - accepted_->y_m) > params_.gate_y_m ||
    std::abs(wrap(seen.rz_rad - accepted_->rz_rad)) > params_.gate_rz_rad) {
    if (++gate_rejects_ < params_.gate_relock) {
      return false;
    }
    // A run of them: the robot really moved; start over here.
  }
  accepted_ = seen;
  accepted_t_ = t_sec;
  gate_rejects_ = 0;
  return true;
}

void SolutionSelector::miss(double t_sec)
{
  if (last_normal_ && t_sec - last_normal_t_ > params_.hint_hold_sec) {
    last_normal_.reset();
  }
}

void SolutionSelector::reset()
{
  last_normal_.reset();
  refine_zone_ = false;
  accepted_.reset();
  gate_rejects_ = 0;
}

}  // namespace aruco
}  // namespace syncai_landmark_detection
