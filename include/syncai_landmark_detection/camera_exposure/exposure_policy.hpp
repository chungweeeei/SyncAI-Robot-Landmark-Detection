#ifndef SYNCAI_LANDMARK_DETECTION__CAMERA_EXPOSURE__EXPOSURE_POLICY_HPP_
#define SYNCAI_LANDMARK_DETECTION__CAMERA_EXPOSURE__EXPOSURE_POLICY_HPP_

// Which exposure the camera should have so that a detector can read its
// landmark: decided from how bright the landmark's own region of the image
// is, one frame at a time. ROS-free and OpenCV-free;
// test/test_exposure_policy.cpp.
//
// Why a detector decides this at all: the camera's auto exposure meters the
// whole frame and goes no shorter than 8333 us. The dock's marker is 0.4 % of
// the frame, so wherever it is brighter than its surroundings it burns out --
// on robot01, 2026-11-11, 51 % of the marker's pixels at 250 and above, the
// black cells grey at ~150 and the black border, which is what cv::aruco
// finds the quad by, gone. The detector is the one that knows where the
// landmark is. It only decides: syncai_camera_feeder, the camera's one
// opener, writes the exposure (feeder_exposure_client.hpp).
//
// Two states.
// - SEARCHING: with the landmark out of sight, step through a ladder of
//   exposures (there is nothing to meter); with it in sight, halve the
//   exposure while its region is burnt out, else scale it towards a target
//   brightness, until the region is inside the lock band.
// - LOCKED: hold the exposure. SyncAI-Robot-Charger held one exposure for a
//   whole alignment, and for a reason that still stands: a change of exposure
//   moves where the corners are found (blooming, the threshold), and the
//   caller is steering on that pose. It leaves only when the region is burnt
//   out or dark by a wider margin than it entered with, several frames in a
//   row, or the landmark has been out of sight for lost_frames. The band it
//   holds inside is what keeps the exposure still while the robot walks up
//   to the marker and it grows in the image.
//
// Every number below is a first value from before the camera was measured
// (how many frames a change takes to show, how the marker's brightness
// follows the exposure): they are parameters, and the comments say what each
// was reasoned from.

#include <optional>
#include <string>
#include <vector>

namespace syncai_landmark_detection
{
namespace camera_exposure
{

/** @brief How bright a landmark's region of one frame is (region_brightness.hpp). */
struct RegionBrightness
{
  // The 95th percentile of the region's grey levels, 0-255: the white cells'
  // level, whatever share of the region is black.
  double p95{0.0};
  // The share of the region's pixels at saturated_level or above, 0-1.
  double saturated{0.0};
};

struct ExposureParams
{
  // The exposures tried, in order, while the landmark is out of sight [us].
  // The first is where every activation starts. 8333 = 1/120 s, the charger's
  // start (read every frame at 1.3 m, lights on and off, 2026-10-08); the
  // halvings below it are for a lit marker and for sun on it (0/10 at 8333,
  // 10/10 at 4000 and below); 16600 last, for a marker darker than the room.
  // The shortest rung bounds what is found at all: a marker unreadable there
  // is never metered, so nothing takes the exposure below it.
  std::vector<int> ladder_us{8333, 4166, 2083, 1000, 16600};
  // What the feeder clamps exposure_override_us to: one V4L2 step, and one
  // frame period at 60 fps (longer is a slower camera, not a brighter one).
  int min_us{100};
  int max_us{16600};
  // Frames ignored after every change: the ones already exposed, or in the
  // pipeline, when the new value reached the sensor. Unmeasured.
  int settle_frames{3};
  // Frames without the landmark, after settling, before the next rung. With
  // settle_frames, one sweep of the five rungs is 40 frames, 1.3 s at 30 Hz.
  int miss_frames{5};
  // LOCKED: frames without the landmark before searching again. Longer than
  // miss_frames: a held exposure that read the marker a moment ago is more
  // likely right than any rung, and a robot walking past an obstruction
  // loses a few frames.
  int lost_frames{15};
  // The brightness the white cells are steered to, and the band around it
  // that counts as arrived. 190 leaves the whites clear of 255 and the black
  // cells near the floor; the band is wide enough that one multiplicative
  // step (below) lands inside it.
  double target_p95{190.0};
  double lock_p95_min{160.0};
  double lock_p95_max{220.0};
  // More of the region than this at saturated_level: burnt out, halve.
  double lock_max_saturated{0.02};
  // LOCKED leaves on a wider margin than SEARCHING arrives with, held for
  // unlock_frames frames in a row (one glint is not a reason).
  double unlock_max_saturated{0.10};
  double unlock_p95_min{100.0};
  int unlock_frames{3};
  // One step changes the exposure by at most this factor either way: the
  // sensor's response is not linear (the charger's whole-frame means at 8333
  // / 4000 / 2000 / 1000 us were 108 / 70 / 42 / 21), so a larger step from
  // one measurement overshoots.
  double max_step{2.0};
};

/** @brief What one frame decided. */
struct ExposureStep
{
  enum class Event {
    NONE,    // nothing to do
    SET,     // write exposure_us to the camera
    LOCKED,  // the region is inside the band: exposure_us is held from here
  };
  Event event{Event::NONE};
  int exposure_us{0};
  // The numbers that decided it, for the log: "region 31 % saturated (p95 255)".
  std::string reason;
  // SET only: this step wrapped the ladder, a whole sweep without the landmark.
  bool sweep_finished{false};
};

class ExposurePolicy
{
public:
  enum class State { SEARCHING, LOCKED };

  /** @throw std::invalid_argument naming the parameter that cannot work */
  explicit ExposurePolicy(const ExposureParams & params = {});

  /**
   * @brief Start over, at the ladder's first rung.
   * @return The exposure to write before the first frame is judged
   */
  int start();

  /**
   * @brief One frame while active.
   * @param region The landmark's region when it was found in the frame,
   * nullopt when it was not
   */
  ExposureStep update(const std::optional<RegionBrightness> & region);

  State state() const { return state_; }
  /** @brief The exposure last decided [us], rounded as the feeder rounds it. */
  int exposureUs() const { return exposure_us_; }
  /** @brief Frames since start(), or since LOCKED was last left: how long the search took. */
  int searchFrames() const { return search_frames_; }

private:
  // The exposure this region asks for, nullopt when it asks for none: inside
  // the band, or already at the limit it would move towards.
  std::optional<ExposureStep> adjust(const RegionBrightness & region) const;
  ExposureStep set(int exposure_us, std::string reason, bool sweep_finished = false);
  int quantize(double exposure_us) const;
  // The ladder rung to try after losing the landmark at exposure_us_.
  std::size_t rungAfterLoss() const;

  ExposureParams params_;
  State state_{State::SEARCHING};
  int exposure_us_{0};
  std::size_t rung_{0};
  std::size_t rungs_missed_{0};  // rungs in a row that never showed the landmark
  int settle_left_{0};           // frames still to ignore after the last change
  int missed_{0};                // frames in a row without the landmark, after settling
  int out_of_band_{0};           // LOCKED: frames in a row past the unlock margins
  int search_frames_{0};
};

}  // namespace camera_exposure
}  // namespace syncai_landmark_detection

#endif  // SYNCAI_LANDMARK_DETECTION__CAMERA_EXPOSURE__EXPOSURE_POLICY_HPP_
