#ifndef SYNCAI_LANDMARK_DETECTION__CAMERA_EXPOSURE__FEEDER_EXPOSURE_CLIENT_HPP_
#define SYNCAI_LANDMARK_DETECTION__CAMERA_EXPOSURE__FEEDER_EXPOSURE_CLIENT_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>

#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

namespace syncai_landmark_detection
{
namespace camera_exposure
{

/**
 * @class FeederExposureClient
 * @brief Writes the camera's exposure through syncai_camera_feeder's
 * `exposure_override_us` parameter (microseconds; 0 gives the camera back to
 * the feeder's configured exposure).
 *
 * A detector never opens the camera: the feeder is its one opener, and its
 * one writer. This is the whole of what a detector knows about it -- a node
 * name and one integer in microseconds; the V4L2 unit and the UVC menu's
 * polarity stay in the feeder.
 *
 * Nothing here blocks or throws: set() and release() are called from a sensor
 * callback and from the server's DetectLandmark, under the plugin's lock. A
 * write that fails never fails a detection; the detector goes on reading the
 * frames at whatever exposure they have, and this says why in the log:
 * - no parameter service at begin() (no feeder: a bag replay, a Dev
 *   Container), or no answer within the timeout: one WARN, and no more
 *   writes until the next begin();
 * - the feeder refuses a value: a WARN with its reason, writes go on;
 * - release() fails: an ERROR naming the exposure the camera is left at and
 *   the command that gives it back, since nothing else will until the feeder
 *   restarts.
 */
class FeederExposureClient
{
public:
  /**
   * @param node The node that hosts the detector
   * @param callback_group Where the answers and the timeout check run
   * @param feeder_node The feeder's node name, relative to this node's
   * namespace ("syncai_camera_feeder")
   * @param timeout_sec How long an answer may take
   * @param owner The detector's name, for the log
   */
  FeederExposureClient(
    const rclcpp::Node::SharedPtr & node, rclcpp::CallbackGroup::SharedPtr callback_group,
    const std::string & feeder_node, double timeout_sec, std::string owner);
  ~FeederExposureClient();

  /**
   * @brief A new activation: forget the last one's failure.
   * @return false when the feeder's parameter service is not there
   */
  bool begin();

  /**
   * @brief Ask for a manual exposure of exposure_us.
   * @return false when this activation has stopped writing
   */
  bool set(int exposure_us);

  /** @brief Give the camera back, if this activation wrote to it. */
  void release();

  /** @brief Whether this activation still writes. */
  bool usable() const;

  /** @brief Why it does not, for a caller's own message; empty while usable. */
  std::string status() const;

private:
  using SetParameters = rcl_interfaces::srv::SetParameters;

  struct Pending
  {
    int64_t request_id{0};
    int exposure_us{0};
    bool release{false};
    std::chrono::steady_clock::time_point sent_at;
  };

  // Caller holds mutex_.
  bool send(int exposure_us, bool release);
  void onResponse(uint64_t key, rclcpp::Client<SetParameters>::SharedFuture future);
  void onTimeoutCheck();
  void giveUp(const std::string & why);
  std::string restoreHint() const;

  rclcpp::Logger logger_;
  std::string owner_;
  std::string feeder_node_;
  std::chrono::duration<double> timeout_;
  rclcpp::Client<SetParameters>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  mutable std::mutex mutex_;  // everything below
  bool usable_{false};
  std::string status_{"not started"};
  // The exposure the camera was last asked for by this activation; 0 when it
  // has not been written to.
  int written_us_{0};
  uint64_t next_key_{0};
  std::map<uint64_t, Pending> pending_;
};

}  // namespace camera_exposure
}  // namespace syncai_landmark_detection

#endif  // SYNCAI_LANDMARK_DETECTION__CAMERA_EXPOSURE__FEEDER_EXPOSURE_CLIENT_HPP_
