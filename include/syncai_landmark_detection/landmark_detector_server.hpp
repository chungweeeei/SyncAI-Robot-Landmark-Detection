#ifndef SYNCAI_LANDMARK_DETECTION__LANDMARK_DETECTOR_SERVER_HPP_
#define SYNCAI_LANDMARK_DETECTION__LANDMARK_DETECTOR_SERVER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_common/srv/detect_landmark.hpp"
#include "syncai_landmark_detection/landmark_detector.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace syncai_landmark_detection
{

/**
 * @class LandmarkDetectorServer
 * @brief The landmark_detector node: loads the detector plugins, keeps the
 * landmark registry, and answers syncai_common/srv/DetectLandmark on
 * `detect_landmark`.
 *
 * Built like SyncAI-Robot-Workspace's planner_server: a plain rclcpp::Node
 * whose constructor only declares parameters, and a configure() main() calls
 * once the node is owned by a shared_ptr (shared_from_this() for the plugins).
 *
 * What the server does and the plugins do not: the registry lookup, waking a
 * plugin on a call and putting it back to standby after idle_timeout, the
 * max_age judgement, and the transform of the plugin's sighting (sensor frame,
 * capture stamp) into the caller's target_frame at that stamp.
 */
class LandmarkDetectorServer : public rclcpp::Node
{
public:
  explicit LandmarkDetectorServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LandmarkDetectorServer() override;

  /** @brief Load the plugins and the registry, start the service. Exits on a bad config. */
  void configure();

private:
  using DetectLandmark = syncai_common::srv::DetectLandmark;

  void onDetect(
    const std::shared_ptr<DetectLandmark::Request> request,
    std::shared_ptr<DetectLandmark::Response> response);
  void onStandbyTimer();

  pluginlib::ClassLoader<LandmarkDetector> loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> detector_ids_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string odom_frame_;
  double default_max_age_{0.3};
  double idle_timeout_{5.0};

  // Plugin id -> plugin; landmark id -> plugin id. The plugins' activation
  // state and last call are touched only from callback_group_ (the service
  // and the standby timer), so they need no lock of their own.
  std::map<std::string, LandmarkDetector::Ptr> detectors_;
  std::map<std::string, std::string> landmark_detector_;
  std::map<std::string, bool> active_;
  std::map<std::string, rclcpp::Time> last_call_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Service<DetectLandmark>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr standby_timer_;
};

}  // namespace syncai_landmark_detection

#endif  // SYNCAI_LANDMARK_DETECTION__LANDMARK_DETECTOR_SERVER_HPP_
