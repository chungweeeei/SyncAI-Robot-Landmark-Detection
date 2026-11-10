#include "syncai_landmark_detection/landmark_detector_server.hpp"

#include <cstdio>
#include <stdexcept>

#include "syncai_landmark_detection/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace syncai_landmark_detection
{

namespace
{

std::string seconds(double s)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.2f s", s);
  return buffer;
}

}  // namespace

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
      detectors_.emplace(id, detector);
      active_[id] = false;
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
      it->second->addLandmark(landmark);
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
      std::shared_ptr<DetectLandmark::Response> response) { onDetect(request, response); },
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
  const auto & detector = detectors_.at(detector_id);
  const rclcpp::Time now = this->now();
  last_call_[detector_id] = now;

  // A call is what wakes a detector: it then needs a frame or two first.
  if (!active_[detector_id]) {
    detector->activate();
    active_[detector_id] = true;
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
  if (!sighting) {
    response->error_code = Response::NOT_DETECTED;
    response->error_msg = request->landmark_id + " not seen since " + detector_id + " woke";
    return;
  }
  const double age = (now - rclcpp::Time(sighting->pose.header.stamp)).seconds();
  if (age > max_age) {
    response->error_code = Response::NOT_DETECTED;
    response->error_msg = request->landmark_id + " last seen " + seconds(age) + " ago (max_age " +
                          seconds(max_age) + ")";
    return;
  }

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

void LandmarkDetectorServer::onStandbyTimer()
{
  const rclcpp::Time now = this->now();
  for (auto & [id, active] : active_) {
    if (!active) {
      continue;
    }
    const auto last = last_call_.find(id);
    if (last == last_call_.end() || (now - last->second).seconds() > idle_timeout_) {
      detectors_.at(id)->deactivate();
      active = false;
      RCLCPP_INFO(
        get_logger(), "[LandmarkDetectorServer][%s] %s: no call for %.1f s, standby", __func__,
        id.c_str(), idle_timeout_);
    }
  }
}

}  // namespace syncai_landmark_detection
