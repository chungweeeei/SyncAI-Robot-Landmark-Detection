#include "syncai_landmark_detection/landmark_detector_server.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "syncai_landmark_detection/node_utils.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace syncai_landmark_detection
{

LandmarkDetectorServer::LandmarkDetectorServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("landmark_detector", options),
  loader_("syncai_landmark_detection", "syncai_landmark_detection::LandmarkDetector"),
  default_ids_{"ArUco"},
  default_types_{"syncai_landmark_detection/ArucoDetector"}
{
  RCLCPP_INFO(get_logger(), "[LandmarkDetectorServer][%s] Creating", __func__);

  // DetectLandmark's max_age when the caller gives <= 0.
  declare_parameter("default_max_age", 0.3);
  // A detector nobody has asked for this long goes back to standby (stops
  // reading its sensor). The charge tree polls at a few Hz, so 5 s is "the
  // dock is over", not a gap between two calls.
  declare_parameter("idle_timeout", 5.0);
  declare_parameter("detector_plugins", default_ids_);
  // The registry: each entry names its detector in `<landmark>.detector` and
  // carries that plugin's parameters for it under `<landmark>.`.
  declare_parameter("landmarks", std::vector<std::string>{});

  get_parameter("detector_plugins", detector_ids_);
  if (detector_ids_ == default_ids_) {
    for (std::size_t i = 0; i < default_ids_.size(); ++i) {
      declare_parameter_if_not_declared(
        this, default_ids_[i] + ".plugin", rclcpp::ParameterValue(default_types_[i]));
    }
  }
}

LandmarkDetectorServer::~LandmarkDetectorServer()
{
  standby_timer_.reset();
  service_.reset();
  detectors_.clear();
}

void LandmarkDetectorServer::configure()
{
  RCLCPP_INFO(get_logger(), "[LandmarkDetectorServer][%s] Configuring", __func__);
  auto node = shared_from_this();

  default_max_age_ = get_parameter("default_max_age").as_double();
  idle_timeout_ = get_parameter("idle_timeout").as_double();

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, true);

  for (const auto & id : detector_ids_) {
    try {
      const std::string type = get_plugin_type_param(node, id);
      LandmarkDetector::Ptr detector = loader_.createUniqueInstance(type);
      RCLCPP_INFO(
        get_logger(), "[LandmarkDetectorServer][%s] Created detector plugin %s of type %s",
        __func__, id.c_str(), type.c_str());
      detector->initialize(node, id, tf_);
      detectors_.emplace(id, Detector{detector, false, std::nullopt});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "[LandmarkDetectorServer][%s] Failed to create detector %s: %s", __func__,
        id.c_str(), ex.what());
      exit(-1);
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        get_logger(), "[LandmarkDetectorServer][%s] Detector %s refused its parameters: %s",
        __func__, id.c_str(), ex.what());
      exit(-1);
    }
  }

  // A registry entry the server cannot honour is a configuration error, and
  // a dock against it would fail at the robot instead: refuse to start.
  const auto landmarks = get_parameter("landmarks").as_string_array();
  for (const auto & landmark : landmarks) {
    declare_parameter_if_not_declared(
      node, landmark + ".detector", rclcpp::ParameterValue(std::string("")));
    const std::string detector = get_parameter(landmark + ".detector").as_string();
    const auto it = detectors_.find(detector);
    if (it == detectors_.end()) {
      RCLCPP_FATAL(
        get_logger(),
        "[LandmarkDetectorServer][%s] Landmark %s names detector '%s', which is not "
        "in detector_plugins",
        __func__, landmark.c_str(), detector.c_str());
      exit(-1);
    }
    try {
      it->second.plugin->addLandmark(landmark);
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        get_logger(), "[LandmarkDetectorServer][%s] Landmark %s refused by %s: %s", __func__,
        landmark.c_str(), detector.c_str(), ex.what());
      exit(-1);
    }
    landmark_detector_[landmark] = detector;
  }
  if (landmarks.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "[LandmarkDetectorServer][%s] No landmarks registered: every call answers "
      "UNKNOWN_LANDMARK",
      __func__);
  }

  // The service and the standby timer share one mutually exclusive group, so
  // a plugin is never activated and deactivated at once; the plugins' sensor
  // callbacks run on their own groups (main() spins a MultiThreadedExecutor).
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_ = create_service<DetectLandmark>(
    "detect_landmark",
    [this](
      const std::shared_ptr<DetectLandmark::Request> request,
      std::shared_ptr<DetectLandmark::Response> response) {
      onDetect(request, response);
      logAnswer(*request, *response);
    },
    rmw_qos_profile_services_default, callback_group_);
  standby_timer_ = create_wall_timer(
    std::chrono::seconds(1), [this]() { onStandbyTimer(); }, callback_group_);

  RCLCPP_INFO(
    get_logger(),
    "[LandmarkDetectorServer][%s] Ready: %zu detector(s), %zu landmark(s), "
    "idle_timeout %.1f s",
    __func__, detectors_.size(), landmark_detector_.size(), idle_timeout_);
}

void LandmarkDetectorServer::onDetect(
  const std::shared_ptr<DetectLandmark::Request> request,
  std::shared_ptr<DetectLandmark::Response> response)
{
  using Response = DetectLandmark::Response;
  response->reprojection_error_px = -1.0F;

  const auto entry = landmark_detector_.find(request->landmark_id);
  if (entry == landmark_detector_.end()) {
    std::string known;
    for (const auto & [id, detector] : landmark_detector_) {
      (void)detector;
      known += (known.empty() ? "" : ", ") + id;
    }
    response->error_code = Response::UNKNOWN_LANDMARK;
    response->error_msg = "'" + request->landmark_id + "' is not in the registry (" +
                          (known.empty() ? "it is empty" : known) + ")";
    return;
  }
  const std::string & detector_id = entry->second;
  Detector & state = detectors_.at(detector_id);
  const auto & detector = state.plugin;
  const rclcpp::Time now = this->now();
  state.last_call = now;

  // A call is what wakes a detector: it then needs a frame or two first.
  if (!state.active) {
    detector->activate();
    state.active = true;
    response->error_code = Response::SENSOR_STARTING;
    response->error_msg = detector_id + " woken by this call; ask again";
    return;
  }

  const SensorStatus status = detector->sensorStatus();
  if (status.state == SensorStatus::State::STARTING) {
    response->error_code = Response::SENSOR_STARTING;
    response->error_msg = detector_id + ": " + status.reason;
    return;
  }
  if (status.state == SensorStatus::State::UNAVAILABLE) {
    response->error_code = Response::SENSOR_UNAVAILABLE;
    response->error_msg = detector_id + ": " + status.reason;
    return;
  }

  const auto sighting = detector->latest(request->landmark_id);
  const double max_age = request->max_age > 0.0F ? request->max_age : default_max_age_;
  // Which of the method's own reasons the newest frame had for no sighting
  // (not found, found and gated, the exposure still searching): without it a
  // burnt-out marker and one out of view are the same "not seen".
  const auto why = [&]() {
    const std::string reason = detector->missReason(request->landmark_id);
    return reason.empty() ? std::string() : ": " + reason;
  };
  if (!sighting) {
    response->error_code = Response::NOT_DETECTED;
    response->error_msg = request->landmark_id + " not seen since " + detector_id + " woke" + why();
    return;
  }
  double age = (now - rclcpp::Time(sighting->pose.header.stamp)).seconds();
  if (age > max_age) {
    response->error_code = Response::NOT_DETECTED;
    response->error_msg = request->landmark_id + " last seen " + formatSeconds(age) +
                          " ago (max_age " + formatSeconds(max_age) + ")" + why();
    return;
  }
  // A stamp ahead of the node's clock: the feeder stamps from another clock,
  // or this one stepped back after the frame. Unbounded, a negative age passes
  // every max_age, and a sighting from before a clock step would be answered
  // as current until the plugin's frame_timeout -- or for good, while frames
  // without the marker keep arriving. Ahead by more than max_age, how old the
  // sighting is cannot be told: refused. Within it, it is as fresh as can be
  // said, age 0 (the srv's age is seconds since the stamp, never negative).
  if (age < -max_age) {
    response->error_code = Response::NOT_DETECTED;
    response->error_msg = request->landmark_id + " seen in an image stamped " +
                          formatSeconds(-age) + " ahead of this node's clock (max_age " +
                          formatSeconds(max_age) + "): the camera's and the node's clocks disagree";
    return;
  }
  age = std::max(age, 0.0);

  // No target_frame: the sensor's own frame, as seen (2026-11-10: needs no TF,
  // so it answers in every mode; a docking target's fixed frame is the
  // consumer's to choose). Any other: at the capture stamp, since the robot
  // has moved since the frame was taken.
  if (request->target_frame.empty()) {
    response->pose = sighting->pose;
  } else {
    try {
      response->pose =
        tf_->transform(sighting->pose, request->target_frame, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      response->error_code = Response::TRANSFORM_FAILED;
      response->error_msg = sighting->pose.header.frame_id + " -> " + request->target_frame +
                            " at the image stamp: " + ex.what();
      return;
    }
  }
  response->error_code = Response::NONE;
  response->age = static_cast<float>(age);
  response->consecutive = sighting->consecutive;
  response->ambiguous = sighting->ambiguous;
  response->reprojection_error_px = static_cast<float>(sighting->reprojection_error_px);
}

void LandmarkDetectorServer::logAnswer(
  const DetectLandmark::Request & request, const DetectLandmark::Response & response)
{
  using Response = DetectLandmark::Response;
  if (response.error_code == Response::NONE) {
    // A sighting: what the caller was handed, so a charge that steered
    // somewhere odd can be read back against the pose it steered on. INFO, so
    // it is in the node's stdout (and the session's multilog) without a log
    // level to remember; throttled to 1 s, because a tree polls at 10 Hz for
    // the whole approach and every answer would bury the rest of the log.
    const auto & p = response.pose.pose;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    const tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    constexpr double kDeg = 180.0 / M_PI;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[LandmarkDetectorServer][%s] '%s' in %s: xyz (%.3f, %.3f, %.3f) m, rpy (%.1f, %.1f, %.1f) "
      "deg, age %.3f s, consecutive %u, ambiguous %s, reprojection %.2f px",
      __func__, request.landmark_id.c_str(), response.pose.header.frame_id.c_str(), p.position.x,
      p.position.y, p.position.z, roll * kDeg, pitch * kDeg, yaw * kDeg, response.age,
      response.consecutive, response.ambiguous ? "yes" : "no", response.reprojection_error_px);
    return;
  }
  // SENSOR_STARTING is every wake-up's first answers, not a failure. The
  // rest go to the caller in error_msg, but the caller is a BT node polling
  // at its own rate that may log none of it; until 2026-11 this log had only
  // "activate" and "standby", and a charge that never saw its marker left no
  // reason behind. Throttled: a tree polls at 10 Hz for as long as it waits.
  if (response.error_code == Response::SENSOR_STARTING) {
    return;
  }
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 5000, "[LandmarkDetectorServer][%s] '%s' (code %u): %s", __func__,
    request.landmark_id.c_str(), response.error_code, response.error_msg.c_str());
}

void LandmarkDetectorServer::onStandbyTimer()
{
  const rclcpp::Time now = this->now();
  for (auto & [id, detector] : detectors_) {
    if (!detector.active) {
      continue;
    }
    if (!detector.last_call || (now - *detector.last_call).seconds() > idle_timeout_) {
      detector.plugin->deactivate();
      detector.active = false;
      RCLCPP_INFO(
        get_logger(), "[LandmarkDetectorServer][%s] %s: no call for %.1f s, standby", __func__,
        id.c_str(), idle_timeout_);
    }
  }
}

}  // namespace syncai_landmark_detection
