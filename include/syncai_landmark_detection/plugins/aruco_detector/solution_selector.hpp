#ifndef SYNCAI_LANDMARK_DETECTION__PLUGINS__ARUCO_DETECTOR__SOLUTION_SELECTOR_HPP_
#define SYNCAI_LANDMARK_DETECTION__PLUGINS__ARUCO_DETECTOR__SOLUTION_SELECTOR_HPP_

// Which of a marker's two PnP solutions is the real one, and whether a
// sighting is trusted at all: the decision SyncAI-Robot-Charger's
// detector_node made in selectBranch / refineNear / smoothPose, kept per
// landmark across frames. ROS-free; test/test_solution_selector.cpp.
//
// Ported with every threshold the charger measured (2026-10). Two parts were
// left behind on purpose (decided 2026-11-09):
// - The motion-based choice (the charger's BranchTracker: gyro + Point-LIO
//   odometry telling the mirror solutions apart while the robot moves). Where
//   the image cannot tell them apart this picks by the previous normal and
//   says so (ambiguous), and the caller decides whether to use the pose.
// - The exponential smoothing. Smoothing in the camera frame lags and bends
//   while the robot moves; it belongs to whoever holds the pose in a fixed
//   frame. Only the gate against mirror jumps stays here.

#include <functional>
#include <optional>
#include <vector>

#include "syncai_landmark_detection/plugins/aruco_detector/marker_geometry.hpp"

namespace syncai_landmark_detection
{
namespace aruco
{

/** @brief The thresholds, all the charger's (config/detector.yaml, 2026-10). */
struct SelectorParams
{
  // Geometry: a solution with the camera behind the marker, turned more than
  // this, or the print leaning more than this out of upright is impossible.
  double max_rz_rad{60.0 * CV_PI / 180.0};
  double max_tilt_rad{45.0 * CV_PI / 180.0};
  // Decisive image: the other solution's reprojection error is at least
  // ratio x the better one's + margin.
  double decisive_ratio{3.0};
  double decisive_margin_px{0.3};
  // Near the marker the LM-refined pose replaces IPPE's: in below max_x, out
  // again only beyond exit_x (hysteresis, on the refined pose's X). There, two
  // refined solutions within merge_rz / merge_y of each other are one pose,
  // so the image is decisive: below 0.6 m they met on 98-100 % of frames.
  bool refine_lm{true};
  double refine_max_x_m{0.6};
  double refine_exit_x_m{0.65};
  double merge_rz_rad{0.5 * CV_PI / 180.0};
  double merge_y_m{0.003};
  // The previous pose's normal picks between two undecided solutions; kept
  // this long after the last pose, so one missed frame cannot let the mirror
  // in (it did on 2026-10-01: Y and Rz jumped to the other branch).
  double hint_hold_sec{1.0};
  // Gate: a pose this far (Y or Rz) from the last accepted one is the mirror
  // solution, not motion, and is dropped, unless relock of them come in a row
  // (the robot did move). Nothing accepted for reset_sec: start over.
  double gate_y_m{0.302};
  double gate_rz_rad{20.0 * CV_PI / 180.0};
  int gate_relock{5};
  double gate_reset_sec{0.5};
};

/** @brief How the published solution was picked, for the log. */
enum class Choice {
  Single,    // one solution, or one left after the geometry check
  Decisive,  // the image told the two apart by reprojection error
  Merged,    // near the marker the two refined solutions met
  Hint,      // undecided: the one closest to the previous normal (ambiguous)
};

/** @brief One frame's outcome for one landmark. */
struct Selection
{
  MarkerPose pose;        // the IPPE solution, or its LM refinement near the marker
  bool ambiguous{false};  // two valid solutions the image could not tell apart
  bool gated{false};      // dropped by the gate: do not use the pose
  bool refined{false};    // pose is the LM-refined one
  Choice choice{Choice::Single};
};

class SolutionSelector
{
public:
  explicit SolutionSelector(const SelectorParams & params = {}) : params_(params) {}

  /**
   * @param t_sec The frame's capture time [s]; only differences are used
   * @param candidates estimateCandidates()'s solutions for this frame
   * @param refine_fn LM-refines a solution on this frame's corners
   * @return nullopt when no solution passes the geometry check; otherwise the
   * pick, with gated set when the gate dropped it
   */
  std::optional<Selection> select(
    double t_sec, const std::vector<MarkerPose> & candidates,
    const std::function<MarkerPose(const MarkerPose &)> & refine_fn);

  /** @brief A frame without the marker: ages the hint. */
  void miss(double t_sec);

  /** @brief Forget everything (a new activation: possibly another dock). */
  void reset();

private:
  bool decideRefined(
    const MarkerPose & refined, const std::optional<MarkerPose> & refined_alt, Selection & out);
  bool gate(const CameraInMarker & seen, double t_sec);

  SelectorParams params_;
  std::optional<cv::Vec3d> last_normal_;
  double last_normal_t_{0.0};
  bool refine_zone_{false};
  std::optional<CameraInMarker> accepted_;
  double accepted_t_{0.0};
  int gate_rejects_{0};
};

}  // namespace aruco
}  // namespace syncai_landmark_detection

#endif  // SYNCAI_LANDMARK_DETECTION__PLUGINS__ARUCO_DETECTOR__SOLUTION_SELECTOR_HPP_
