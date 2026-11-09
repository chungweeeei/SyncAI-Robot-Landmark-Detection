#ifndef SYNCAI_LANDMARK_DETECTION__LANDMARK_DETECTOR_HPP_
#define SYNCAI_LANDMARK_DETECTION__LANDMARK_DETECTOR_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace syncai_landmark_detection
{

/**
 * @brief One sighting of one landmark, as the plugin saw it.
 *
 * In the sensor's own frame (pose.header.frame_id, e.g.
 * <robot_id>/camera_optical_frame) and stamped with the capture time of the
 * input it came from -- not the time it was processed: the robot moves in
 * between, and the server transforms the pose into the caller's target_frame
 * at that stamp. A plugin never transforms.
 */
struct Sighting
{
  // The landmark's centre, +x out of its face (towards a robot in front of
  // it), +z up: DetectLandmark.srv's convention, kept by every method.
  geometry_msgs::msg::PoseStamped pose;
  // Inputs in a row with this landmark, this one included; reset by an input
  // without it.
  uint16_t consecutive{0};
  // An alternative pose the plugin has not ruled out yet (ArUco: the mirrored
  // PnP solution near the marker's axis at ~1 m). The pose is its best choice
  // even then.
  bool ambiguous{false};
  // The method's fit residual in pixels (ArUco: the corners' reprojection
  // error); -1 when it has none.
  double reprojection_error_px{-1.0};
};

/** @brief Whether the plugin's sensor is delivering, and why not when it is not. */
struct SensorStatus
{
  enum class State {
    // Activated, no input yet: DetectLandmark's SENSOR_STARTING.
    STARTING,
    // Inputs arriving.
    STREAMING,
    // No usable input for the plugin's own timeout, or no calibration to use
    // it with: DetectLandmark's SENSOR_UNAVAILABLE.
    UNAVAILABLE,
  };
  State state{State::STARTING};
  // Goes into DetectLandmark's error_msg, so it names the cause: "no
  // camera_info (no calibration file?)", "no frame for 1.0 s".
  std::string reason;
};

/**
 * @class LandmarkDetector
 * @brief Abstract interface for landmark detection methods, loaded with
 * pluginlib by the landmark_detector server (detector_plugins, `<id>.plugin`),
 * the way syncai_nav_core's planners and controllers are.
 *
 * One plugin per method (ArUco first). A plugin owns its sensor subscriptions
 * -- a camera method the image and camera_info, a lidar method a point cloud --
 * and keeps the newest sighting of each landmark registered with it. It only
 * detects: the server answers DetectLandmark (registry lookup, max_age, the
 * transform to target_frame, age), and nothing here commands motion or judges
 * a sighting good enough.
 *
 * Like syncai_nav_core's plugins, a plain rclcpp::Node: initialize() does the
 * full setup and teardown happens in the destructor. Unlike them it has
 * activate() / deactivate(), because a planner or a controller works from
 * start-up while a detector reads its sensor only while a caller polls it
 * (DetectLandmark wakes it; the server puts it back after idle_timeout), so an
 * idle robot decodes no images. They start and stop the sensor; they are not
 * lifecycle transitions.
 *
 * Threading: the server runs a MultiThreadedExecutor. A plugin puts its
 * subscriptions on a callback group of its own, so a frame being decoded never
 * delays a DetectLandmark answer, and guards its sightings with its own mutex:
 * sensorStatus() and latest() are called from the service while its callbacks
 * run.
 */
class LandmarkDetector
{
public:
  using Ptr = std::shared_ptr<LandmarkDetector>;

  virtual ~LandmarkDetector() {}

  /**
   * @param node The node that hosts the landmark detector server
   * @param name The name of this detector instance (parameter namespace)
   * @param tf A pointer to the server's TF buffer, for a method that needs the
   * robot's own motion (ArUco's mirror-branch choice); most need not use it
   *
   * Declares the plugin's parameters under `name.`. Does not subscribe: the
   * plugin is in standby until activate().
   */
  virtual void initialize(
    const rclcpp::Node::SharedPtr & node, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf) = 0;

  /**
   * @brief Register one landmark of the server's registry that names this
   * plugin. Called after initialize(), once per landmark.
   *
   * Reads the landmark's own parameters under `landmark_id.` (ArUco: the
   * dictionary, the marker id and the printed size, which belongs to the
   * landmark: two prints of different sizes under one detector-wide size give
   * a wrong distance with no error).
   *
   * @throw std::invalid_argument when the plugin cannot detect it (an unknown
   * dictionary, an id outside it); the server then refuses to start, as for a
   * plugin that fails to load.
   */
  virtual void addLandmark(const std::string & landmark_id) = 0;

  /**
   * @brief Start reading the sensor, and forget every sighting and every
   * per-landmark history: what was seen before the last deactivate() may be
   * from another dock. Must return at once: the server calls it from
   * DetectLandmark.
   */
  virtual void activate() = 0;

  /** @brief Stop reading the sensor. Must return at once. */
  virtual void deactivate() = 0;

  /** @brief The sensor's state. Thread-safe and non-blocking. */
  virtual SensorStatus sensorStatus() const = 0;

  /**
   * @brief The newest sighting of landmark_id since activate(); nullopt when
   * there is none. Thread-safe and non-blocking.
   *
   * Whether the pose is filtered over several inputs is the plugin's choice;
   * its stamp is that of the newest input it includes.
   */
  virtual std::optional<Sighting> latest(const std::string & landmark_id) const = 0;
};

}  // namespace syncai_landmark_detection

#endif  // SYNCAI_LANDMARK_DETECTION__LANDMARK_DETECTOR_HPP_
