#include "syncai_landmark_detection/plugins/aruco_detector/aruco_detector.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "syncai_landmark_detection/camera_exposure/exposure_parameters.hpp"
#include "syncai_landmark_detection/node_utils.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace syncai_landmark_detection
{

namespace
{

constexpr double kDegToRad = CV_PI / 180.0;

cv::Ptr<cv::aruco::Dictionary> dictionaryFromName(const std::string & name)
{
  static const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> names = {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
    {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
    {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
    {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
    {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
    {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
    {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
    {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
  };
  const auto it = names.find(name);
  if (it == names.end()) {
    return nullptr;
  }
  return cv::aruco::getPredefinedDictionary(it->second);
}

// cv::aruco on roi (corners back in frame pixels).
void detectIn(
  const cv::Mat & gray, const cv::Rect & roi, const cv::Ptr<cv::aruco::Dictionary> & dictionary,
  const cv::Ptr<cv::aruco::DetectorParameters> & parameters,
  std::vector<std::vector<cv::Point2f>> & corners, std::vector<int> & ids)
{
  corners.clear();
  ids.clear();
  cv::aruco::detectMarkers(gray(roi), dictionary, corners, ids, parameters);
  const cv::Point2f offset(static_cast<float>(roi.x), static_cast<float>(roi.y));
  for (auto & quad : corners) {
    for (auto & c : quad) {
      c += offset;
    }
  }
}

}  // namespace

ArucoDetector::~ArucoDetector()
{
  std::lock_guard<std::mutex> lock(mutex_);
  image_sub_.reset();
  camera_info_sub_.reset();
}

void ArucoDetector::initialize(
  const rclcpp::Node::SharedPtr & node, std::string name, std::shared_ptr<tf2_ros::Buffer> tf)
{
  (void)tf;  // no motion-based choice yet (solution_selector.hpp)
  node_ = node;
  name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  const auto dp = [&](const std::string & key, const rclcpp::ParameterValue & value) {
    declare_parameter_if_not_declared(node, name_ + "." + key, value);
  };
  const auto get = [&](const std::string & key) { return node->get_parameter(name_ + "." + key); };

  // The feeder's topics, relative so they take the robot_id namespace.
  dp("image_topic", rclcpp::ParameterValue(std::string("image_raw/compressed")));
  dp("camera_info_topic", rclcpp::ParameterValue(std::string("camera_info")));
  // No frame (or no camera_info) this long after activate(), or since the
  // last frame: SENSOR_UNAVAILABLE. The feeder publishes at 30 Hz.
  dp("frame_timeout", rclcpp::ParameterValue(1.0));
  // cornerSubPix's half window [px]. The charger's tuning (2026-10-08, A4
  // print and 3D board at ~1 m): OpenCV's default 5 reached across the
  // print's white rim onto the dark edge beyond it (Rz sd 7.7 deg on the 3D
  // board); 2 gave corner sd <= 0.09 px and Rz sd 0.27 deg; 3 and 1 were
  // worse. Not checked at the dock distance (0.3 m).
  dp("corner_refine_win_px", rclcpp::ParameterValue(2));
  // Search around where the group's markers were last seen first: their
  // bounding box grown by roi_margin x its larger side on every edge, at least
  // roi_min_size_px on each axis. The margin keeps the print's quiet zone in
  // the crop, which the adaptive threshold needs, and covers a frame's motion.
  dp("roi_enabled", rclcpp::ParameterValue(true));
  dp("roi_margin", rclcpp::ParameterValue(0.75));
  dp("roi_min_size_px", rclcpp::ParameterValue(160));
  // The solution choice and the gate (solution_selector.hpp has the why).
  const aruco::SelectorParams d;
  dp("max_rz_deg", rclcpp::ParameterValue(d.max_rz_rad / kDegToRad));
  dp("max_tilt_deg", rclcpp::ParameterValue(d.max_tilt_rad / kDegToRad));
  dp("decisive_ratio", rclcpp::ParameterValue(d.decisive_ratio));
  dp("decisive_margin_px", rclcpp::ParameterValue(d.decisive_margin_px));
  dp("refine_lm", rclcpp::ParameterValue(d.refine_lm));
  dp("refine_max_x_m", rclcpp::ParameterValue(d.refine_max_x_m));
  dp("refine_exit_x_m", rclcpp::ParameterValue(d.refine_exit_x_m));
  dp("merge_rz_deg", rclcpp::ParameterValue(d.merge_rz_rad / kDegToRad));
  dp("merge_y_m", rclcpp::ParameterValue(d.merge_y_m));
  dp("hint_hold_sec", rclcpp::ParameterValue(d.hint_hold_sec));
  dp("gate_y_m", rclcpp::ParameterValue(d.gate_y_m));
  dp("gate_rz_deg", rclcpp::ParameterValue(d.gate_rz_rad / kDegToRad));
  dp("gate_relock", rclcpp::ParameterValue(d.gate_relock));
  dp("gate_reset_sec", rclcpp::ParameterValue(d.gate_reset_sec));
  // The camera's exposure while active; the policy's own parameters are
  // declared with it (declareExposurePolicyParameters, below), none of them
  // measured on the camera yet.
  const camera_exposure::RegionParams r;
  dp("exposure.enabled", rclcpp::ParameterValue(true));
  // The feeder's node name, relative so it takes the robot_id namespace.
  dp("exposure.feeder_node", rclcpp::ParameterValue(std::string("syncai_camera_feeder")));
  // A parameter set on the same host answers in milliseconds; a second
  // without an answer is a feeder that is gone.
  dp("exposure.set_timeout", rclcpp::ParameterValue(1.0));
  dp("exposure.region_scale", rclcpp::ParameterValue(r.scale));
  dp("exposure.saturated_level", rclcpp::ParameterValue(r.saturated_level));

  image_topic_ = get("image_topic").as_string();
  camera_info_topic_ = get("camera_info_topic").as_string();
  frame_timeout_ = get("frame_timeout").as_double();
  corner_refine_win_px_ = static_cast<int>(get("corner_refine_win_px").as_int());
  if (corner_refine_win_px_ < 1) {
    throw std::invalid_argument(name_ + ".corner_refine_win_px must be >= 1");
  }
  roi_enabled_ = get("roi_enabled").as_bool();
  roi_margin_ = get("roi_margin").as_double();
  roi_min_size_px_ = static_cast<int>(get("roi_min_size_px").as_int());
  selector_params_.max_rz_rad = get("max_rz_deg").as_double() * kDegToRad;
  selector_params_.max_tilt_rad = get("max_tilt_deg").as_double() * kDegToRad;
  selector_params_.decisive_ratio = get("decisive_ratio").as_double();
  selector_params_.decisive_margin_px = get("decisive_margin_px").as_double();
  selector_params_.refine_lm = get("refine_lm").as_bool();
  selector_params_.refine_max_x_m = get("refine_max_x_m").as_double();
  selector_params_.refine_exit_x_m =
    std::max(selector_params_.refine_max_x_m, get("refine_exit_x_m").as_double());
  selector_params_.merge_rz_rad = get("merge_rz_deg").as_double() * kDegToRad;
  selector_params_.merge_y_m = get("merge_y_m").as_double();
  selector_params_.hint_hold_sec = get("hint_hold_sec").as_double();
  selector_params_.gate_y_m = get("gate_y_m").as_double();
  selector_params_.gate_rz_rad = get("gate_rz_deg").as_double() * kDegToRad;
  selector_params_.gate_relock = static_cast<int>(get("gate_relock").as_int());
  selector_params_.gate_reset_sec = get("gate_reset_sec").as_double();

  // Its own group, so a frame being decoded and searched never holds up the
  // server's DetectLandmark answer (landmark_detector.hpp, Threading).
  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  const camera_exposure::ExposureParams exposure =
    camera_exposure::declareExposurePolicyParameters(node, name_);
  region_params_.scale = get("exposure.region_scale").as_double();
  region_params_.saturated_level = static_cast<int>(get("exposure.saturated_level").as_int());
  if (!(region_params_.scale > 0.0)) {
    throw std::invalid_argument(name_ + ".exposure.region_scale must be > 0");
  }
  const bool exposure_enabled = get("exposure.enabled").as_bool();
  const std::string feeder_node = get("exposure.feeder_node").as_string();
  const double set_timeout = get("exposure.set_timeout").as_double();
  if (exposure_enabled) {
    try {
      // Checked even when nothing is in view yet: a ladder that cannot work
      // stops the node at start, not at the dock.
      exposure_policy_ = camera_exposure::ExposurePolicy(exposure);
    } catch (const std::invalid_argument & ex) {
      throw std::invalid_argument(name_ + "." + ex.what());
    }
    if (!(set_timeout > 0.0)) {
      throw std::invalid_argument(name_ + ".exposure.set_timeout must be > 0");
    }
    exposure_client_ = std::make_unique<camera_exposure::FeederExposureClient>(
      node, callback_group_, feeder_node, set_timeout, name_);
  }

  RCLCPP_INFO(
    logger_, "[ArucoDetector][%s] %s: %s + %s, frame_timeout %.1f s, roi %s", __func__,
    name_.c_str(), image_topic_.c_str(), camera_info_topic_.c_str(), frame_timeout_,
    roi_enabled_ ? "on" : "off");
  if (exposure_enabled) {
    // The whole ladder as read, so a start that is not the params file's first rung shows here.
    std::string ladder_text;
    for (const int us : exposure.ladder_us) {
      ladder_text += (ladder_text.empty() ? "" : ", ") + std::to_string(us);
    }
    RCLCPP_INFO(
      logger_,
      "[ArucoDetector][%s] %s: exposure through %s, ladder [%s] us, from %d us within %d-%d us, "
      "region p95 to %.0f (lock %.0f-%.0f, <= %.1f %% saturated)",
      __func__, name_.c_str(), feeder_node.c_str(), ladder_text.c_str(),
      exposure_policy_.exposureUs(), exposure.min_us, exposure.max_us, exposure.target_p95,
      exposure.lock_p95_min, exposure.lock_p95_max, exposure.lock_max_saturated * 100.0);
  } else {
    RCLCPP_INFO(
      logger_,
      "[ArucoDetector][%s] %s: exposure.enabled is false, the camera's exposure is left to %s",
      __func__, name_.c_str(), feeder_node.c_str());
  }
}

void ArucoDetector::addLandmark(const std::string & landmark_id)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("ArucoDetector::addLandmark before initialize");
  }
  const auto dp = [&](const std::string & key, const rclcpp::ParameterValue & value) {
    declare_parameter_if_not_declared(node, landmark_id + "." + key, value);
    return node->get_parameter(landmark_id + "." + key);
  };
  // The landmark's own: a second print of another size or dictionary is
  // another registry entry, never a change of a detector-wide value.
  const std::string dictionary_name =
    dp("dictionary", rclcpp::ParameterValue(std::string("DICT_6X6_250"))).as_string();
  const int marker_id = static_cast<int>(dp("marker_id", rclcpp::ParameterValue(-1)).as_int());
  // The printed side [m], black border to black border. X and Y scale
  // linearly with it, so a wrong value is a wrong distance with no error.
  const double marker_size = dp("marker_size", rclcpp::ParameterValue(0.0)).as_double();
  // A print in negative (white code on black; a backlit film, charger
  // 2026-10-13): cv::aruco then also tries every candidate inverted.
  const bool inverted = dp("marker_inverted", rclcpp::ParameterValue(false)).as_bool();

  const auto dictionary = dictionaryFromName(dictionary_name);
  if (!dictionary) {
    throw std::invalid_argument(
      landmark_id + ".dictionary '" + dictionary_name + "' is not a predefined ArUco dictionary");
  }
  if (marker_id < 0 || marker_id >= dictionary->bytesList.rows) {
    throw std::invalid_argument(
      landmark_id + ".marker_id " + std::to_string(marker_id) + " is not in " + dictionary_name +
      " (0-" + std::to_string(dictionary->bytesList.rows - 1) + ")");
  }
  if (!(marker_size > 0.0)) {
    throw std::invalid_argument(landmark_id + ".marker_size must be > 0 (the printed side, m)");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  // Same dictionary and id is the same marker whatever the polarity: the
  // inverted group's detectMarkers decodes normal prints too (cv::aruco tries
  // a candidate as printed first, then inverted), so one normal print would be
  // reported as both landmarks. Checked across both groups of the dictionary,
  // not only within the one this landmark joins.
  for (const char * polarity : {"", "/inverted"}) {
    const auto other_group = groups_.find(dictionary_name + polarity);
    if (other_group == groups_.end()) {
      continue;
    }
    for (const auto & other : other_group->second.landmarks) {
      if (landmarks_.at(other).marker_id == marker_id) {
        throw std::invalid_argument(
          landmark_id + " and " + other + " are the same marker (" + dictionary_name + " id " +
          std::to_string(marker_id) + "): one sighting cannot be told apart");
      }
    }
  }
  const std::string key = dictionary_name + (inverted ? "/inverted" : "");
  auto group = groups_.find(key);
  if (group == groups_.end()) {
    Group g;
    g.dictionary = dictionary;
    g.parameters = cv::aruco::DetectorParameters::create();
    // Sub-pixel corners: the pose (Rz above all, and Y with it) is set by
    // corner precision; unrefined, a 10 cm marker at 1 m picks the mirrored
    // solution (charger-main).
    g.parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    g.parameters->cornerRefinementWinSize = corner_refine_win_px_;
    // The dock print has a white rim ~4 px outside the marker; at the default
    // 0.05 the rim and the marker count as one and only the rim, which never
    // decodes, is kept (charger, 2026-09-29).
    g.parameters->minMarkerDistanceRate = 0.01;
    g.parameters->detectInvertedMarker = inverted;
    group = groups_.emplace(key, std::move(g)).first;
  }
  group->second.landmarks.push_back(landmark_id);
  Landmark landmark;
  landmark.marker_id = marker_id;
  landmark.marker_size_m = marker_size;
  landmark.selector = aruco::SolutionSelector(selector_params_);
  landmarks_.emplace(landmark_id, std::move(landmark));

  RCLCPP_INFO(
    logger_, "[ArucoDetector][%s] %s: %s id %d, %.3f m%s", __func__, landmark_id.c_str(),
    dictionary_name.c_str(), marker_id, marker_size, inverted ? ", inverted" : "");
}

void ArucoDetector::activate()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_) {
    return;
  }
  // What was seen before the last deactivate() may be another dock.
  for (auto & [id, landmark] : landmarks_) {
    (void)id;
    landmark.selector.reset();
    landmark.last_box.reset();
    landmark.latest.reset();
    landmark.consecutive = 0;
    landmark.miss.clear();
  }
  last_frame_at_.reset();
  frame_error_.clear();
  ++generation_;
  active_ = true;
  activated_at_ = clock_->now();

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  // Latched by the feeder (published once at its start): only a reliable,
  // transient-local subscription receives it, and it is cached, never paired
  // with an image by stamp -- its stamp is the feeder's start.
  camera_info_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::QoS(1).reliable().transient_local(),
    [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { onCameraInfo(msg); }, options);
  // The feeder publishes best effort; a reliable subscription would not match.
  // Depth 1, not SensorDataQoS's 5: a frame that takes longer than the 30 Hz
  // period (a whole-frame search on the Jetson) would otherwise leave a queue
  // of older frames, each sighting built from one several periods late -- past
  // max_age with the marker in view, or a pose lagging the robot. Only the
  // newest frame is worth searching.
  image_sub_ = node->create_subscription<sensor_msgs::msg::CompressedImage>(
    image_topic_, rclcpp::SensorDataQoS().keep_last(1),
    [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) { onImage(msg); }, options);
  RCLCPP_INFO(
    logger_, "[ArucoDetector][%s] %s: reading %s", __func__, name_.c_str(), image_topic_.c_str());

  // The exposure the search starts from, before the first frame is judged:
  // the frames until it shows are the policy's settle_frames.
  exposure_running_ = exposure_client_ && exposure_client_->begin();
  if (exposure_running_) {
    const int start_us = exposure_policy_.start();
    exposure_running_ = exposure_client_->set(start_us);
    if (exposure_running_) {
      RCLCPP_INFO(
        logger_, "[ArucoDetector][%s] %s: exposure searching from %d us", __func__, name_.c_str(),
        start_us);
    }
  }
}

void ArucoDetector::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) {
    return;
  }
  active_ = false;
  image_sub_.reset();
  camera_info_sub_.reset();
  // Back to the feeder's own exposure: the console and every other viewer of
  // the camera have had the detector's picture for as long as it was awake.
  if (exposure_client_) {
    exposure_client_->release();
  }
  exposure_running_ = false;
  RCLCPP_INFO(logger_, "[ArucoDetector][%s] %s: standby", __func__, name_.c_str());
}

SensorStatus ArucoDetector::sensorStatus() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  SensorStatus status;
  if (!active_) {
    status.reason = "not active";
    return status;
  }
  const rclcpp::Time now = clock_->now();
  const double since_activate = (now - activated_at_).seconds();
  const bool starting = since_activate < frame_timeout_;
  if (!camera_model_) {
    if (!camera_model_error_.empty()) {
      status.state = SensorStatus::State::UNAVAILABLE;
      status.reason = camera_model_error_;
    } else if (starting) {
      status.reason = "waiting for camera_info on " + camera_info_topic_;
    } else {
      status.state = SensorStatus::State::UNAVAILABLE;
      status.reason =
        "no camera_info (no calibration file?) on " + camera_info_topic_ +
        ": it is latched, so syncai_camera_feeder publishes none without its camera_info_file";
    }
    return status;
  }
  if (!last_frame_at_) {
    if (starting) {
      status.reason = "waiting for the first frame on " + image_topic_;
    } else {
      status.state = SensorStatus::State::UNAVAILABLE;
      status.reason = "no frame on " + image_topic_ + " for " + formatSeconds(since_activate);
    }
    return status;
  }
  const double since_frame = (now - *last_frame_at_).seconds();
  if (since_frame > frame_timeout_) {
    status.state = SensorStatus::State::UNAVAILABLE;
    status.reason = "no frame on " + image_topic_ + " for " + formatSeconds(since_frame);
    return status;
  }
  if (!frame_error_.empty()) {
    status.state = SensorStatus::State::UNAVAILABLE;
    status.reason = frame_error_;
    return status;
  }
  status.state = SensorStatus::State::STREAMING;
  return status;
}

std::optional<Sighting> ArucoDetector::latest(const std::string & landmark_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = landmarks_.find(landmark_id);
  if (it == landmarks_.end()) {
    return std::nullopt;
  }
  return it->second.latest;
}

std::string ArucoDetector::missReason(const std::string & landmark_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = landmarks_.find(landmark_id);
  if (it == landmarks_.end() || !active_ || it->second.miss.empty()) {
    return {};
  }
  return it->second.miss + "; " + exposureStatus();
}

std::string ArucoDetector::exposureStatus() const
{
  if (!exposure_client_) {
    return "exposure control off (exposure.enabled false)";
  }
  if (!exposure_running_) {
    return "exposure control off: " + exposure_client_->status();
  }
  const bool locked = exposure_policy_.state() == camera_exposure::ExposurePolicy::State::LOCKED;
  return std::string("exposure ") + (locked ? "locked at " : "searching at ") +
         std::to_string(exposure_policy_.exposureUs()) + " us";
}

void ArucoDetector::stepExposure(const std::optional<camera_exposure::RegionBrightness> & region)
{
  if (!exposure_running_) {
    return;
  }
  if (!exposure_client_->usable()) {
    exposure_running_ = false;  // the client said why
    return;
  }
  using Event = camera_exposure::ExposureStep::Event;
  const int before_us = exposure_policy_.exposureUs();
  const int search_frames = exposure_policy_.searchFrames() + 1;
  const auto step = exposure_policy_.update(region);
  if (step.sweep_finished) {
    // Every rung tried: the marker is not in view, or not readable at any of
    // them. Which, the frames have to say; this names what was tried.
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "[ArucoDetector][%s] %s: no marker at any exposure of the ladder (last %d us, %s); trying "
      "them again",
      __func__, name_.c_str(), before_us, step.reason.c_str());
  }
  if (step.event == Event::SET) {
    RCLCPP_INFO(
      logger_, "[ArucoDetector][%s] %s: exposure %d -> %d us: %s", __func__, name_.c_str(),
      before_us, step.exposure_us, step.reason.c_str());
    exposure_running_ = exposure_client_->set(step.exposure_us);
  } else if (step.event == Event::LOCKED) {
    RCLCPP_INFO(
      logger_, "[ArucoDetector][%s] %s: exposure locked at %d us after %d frame(s): %s", __func__,
      name_.c_str(), step.exposure_us, search_frames, step.reason.c_str());
  }
}

void ArucoDetector::onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  aruco::CameraModel model;
  model.k = cv::Matx33d(
    msg->k[0], msg->k[1], msg->k[2], msg->k[3], msg->k[4], msg->k[5], msg->k[6], msg->k[7],
    msg->k[8]);
  model.distortion_model = msg->distortion_model;
  model.d = msg->d;
  model.width = static_cast<int>(msg->width);
  model.height = static_cast<int>(msg->height);
  const std::string error = aruco::validateCameraModel(model);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!error.empty()) {
    camera_model_.reset();
    camera_model_error_ = error;
    RCLCPP_ERROR(logger_, "[ArucoDetector][%s] %s: %s", __func__, name_.c_str(), error.c_str());
    return;
  }
  camera_model_ = model;
  camera_model_error_.clear();
  RCLCPP_INFO(
    logger_, "[ArucoDetector][%s] %s: %dx%d %s, fx %.2f fy %.2f cx %.2f cy %.2f", __func__,
    name_.c_str(), model.width, model.height, model.distortion_model.c_str(), model.k(0, 0),
    model.k(1, 1), model.k(0, 2), model.k(1, 2));
}

void ArucoDetector::onImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg)
{
  // An exception out of a callback ends the whole node, mid-dock, and the
  // launch does not respawn it. A frame OpenCV throws on (cv::Exception from
  // the decode, the search, the undistortion or PnP) is one bad frame: say so
  // through sensorStatus() until a good one clears it. The landmarks keep
  // their state from before it: processImage() commits only a finished frame.
  try {
    processImage(msg);
  } catch (const std::exception & e) {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_error_ = "a frame on " + image_topic_ + " failed: " + e.what();
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 5000, "[ArucoDetector][%s] %s: %s", __func__, name_.c_str(),
      frame_error_.c_str());
  }
}

void ArucoDetector::processImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg)
{
  const rclcpp::Time arrived = clock_->now();
  // Straight to grey: cv::aruco works on grey anyway, and the colour planes
  // are never decoded. An empty buffer is a frame that does not decode, not a
  // call: imdecode asserts on one.
  const cv::Mat gray =
    msg->data.empty() ? cv::Mat() : cv::imdecode(msg->data, cv::IMREAD_GRAYSCALE);

  // Copy what a frame needs, work on the copies unlocked, then commit them:
  // the search and PnP (with up to two LM refinements per landmark) are the
  // expensive part, and latest() / sensorStatus() -- the server's answer --
  // must not wait behind them. Only this callback group's one callback at a
  // time writes a landmark's state, so nothing else changes it between the
  // copy and the commit except activate() / deactivate(), which the
  // generation and active_ checks at the commit catch.
  struct LandmarkWork
  {
    const std::string * id;
    int marker_id;
    double marker_size_m;
    aruco::SolutionSelector selector;
    uint16_t consecutive;
    std::optional<cv::Rect> box;
    std::optional<Sighting> sighting;  // set only when this frame gives one
    std::string miss;                  // why not, when it gives none
  };
  struct Work
  {
    const Group * group;
    std::optional<cv::Rect> roi;
    std::vector<LandmarkWork> landmarks;
  };
  std::vector<Work> work;
  aruco::CameraModel model;
  uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) {
      return;
    }
    last_frame_at_ = arrived;
    if (gray.empty()) {
      frame_error_ =
        "a frame on " + image_topic_ + " does not decode (format '" + msg->format + "')";
      return;
    }
    if (!camera_model_) {
      return;  // sensorStatus() says why
    }
    if (gray.cols != camera_model_->width || gray.rows != camera_model_->height) {
      // A calibration for another resolution parses fine and is wrong
      // everywhere (the AR0234's 720 is a crop of 800, which moves cy).
      frame_error_ = "frames are " + std::to_string(gray.cols) + "x" + std::to_string(gray.rows) +
                     ", camera_info is for " + std::to_string(camera_model_->width) + "x" +
                     std::to_string(camera_model_->height);
      return;
    }
    frame_error_.clear();
    model = *camera_model_;
    generation = generation_;
    for (const auto & [key, group] : groups_) {
      (void)key;
      Work w{&group, groupRoi(group, gray.size()), {}};
      for (const auto & id : group.landmarks) {
        const Landmark & landmark = landmarks_.at(id);
        w.landmarks.push_back(
          {&id, landmark.marker_id, landmark.marker_size_m, landmark.selector, landmark.consecutive,
           landmark.last_box, std::nullopt, std::string()});
      }
      work.push_back(std::move(w));
    }
  }

  const cv::Rect whole(cv::Point(0, 0), gray.size());
  const double t_sec = rclcpp::Time(msg->header.stamp).seconds();
  // The brightness of every marker found, whether or not its pose is used: a
  // marker the gate drops is still where the exposure has to be right.
  std::optional<camera_exposure::RegionBrightness> region;
  for (auto & w : work) {
    std::vector<std::vector<cv::Point2f>> group_corners;
    std::vector<int> group_ids;
    bool all_found = false;
    if (w.roi) {
      detectIn(gray, *w.roi, w.group->dictionary, w.group->parameters, group_corners, group_ids);
      all_found = std::all_of(w.landmarks.begin(), w.landmarks.end(), [&](const LandmarkWork & l) {
        return aruco::pickMarker(group_ids, group_corners, l.marker_id) >= 0;
      });
    }
    if (!all_found) {
      detectIn(gray, whole, w.group->dictionary, w.group->parameters, group_corners, group_ids);
    }

    for (auto & landmark : w.landmarks) {
      const int index = aruco::pickMarker(group_ids, group_corners, landmark.marker_id);
      if (index < 0) {
        landmark.box.reset();
        landmark.consecutive = 0;
        landmark.selector.miss(t_sec);
        landmark.miss = "marker id " + std::to_string(landmark.marker_id) +
                        " not found in the frame (" + std::to_string(group_ids.size()) +
                        " other marker(s) of its dictionary were)";
        continue;
      }
      const auto & found = group_corners[static_cast<std::size_t>(index)];
      landmark.box = cv::boundingRect(found);
      if (exposure_client_) {
        if (const auto b = camera_exposure::measureRegion(gray, found, region_params_)) {
          region = region ? camera_exposure::brighter(*region, *b) : *b;
        }
      }
      const auto corners = aruco::undistortCorners(found, model);
      const double size = landmark.marker_size_m;
      const auto candidates = aruco::estimateCandidates(corners, size, model.k);
      const auto selection = landmark.selector.select(
        t_sec, candidates,
        [&](const aruco::MarkerPose & p) { return aruco::refine(p, corners, size, model.k); });
      if (!selection || selection->gated) {
        // No possible solution, or a mirror jump: the last sighting stands
        // (it ages out at the server's max_age) and the run is broken.
        landmark.consecutive = 0;
        landmark.miss = "marker id " + std::to_string(landmark.marker_id) + " found, " +
                        (selection ? "its pose dropped by the gate against mirror jumps"
                                   : "neither PnP solution passes max_rz_deg / max_tilt_deg");
        continue;
      }
      Sighting sighting;
      sighting.pose.header = msg->header;  // the capture stamp, the camera's frame
      const auto & t = selection->pose.translation_m;
      sighting.pose.pose.position.x = t[0];
      sighting.pose.pose.position.y = t[1];
      sighting.pose.pose.position.z = t[2];
      const cv::Matx33d r = aruco::landmarkRotation(selection->pose);
      tf2::Quaternion q;
      tf2::Matrix3x3(
        r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2, 1), r(2, 2))
        .getRotation(q);
      sighting.pose.pose.orientation.x = q.x();
      sighting.pose.pose.orientation.y = q.y();
      sighting.pose.pose.orientation.z = q.z();
      sighting.pose.pose.orientation.w = q.w();
      if (landmark.consecutive < std::numeric_limits<uint16_t>::max()) {
        ++landmark.consecutive;
      }
      sighting.consecutive = landmark.consecutive;
      sighting.ambiguous = selection->ambiguous;
      sighting.reprojection_error_px = selection->pose.reprojection_error_px;
      landmark.sighting = std::move(sighting);
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || generation != generation_) {
    return;  // deactivated, or re-activated for possibly another dock, meanwhile
  }
  for (auto & w : work) {
    for (auto & l : w.landmarks) {
      Landmark & landmark = landmarks_.at(*l.id);
      landmark.selector = std::move(l.selector);
      landmark.consecutive = l.consecutive;
      landmark.last_box = l.box;
      landmark.miss = std::move(l.miss);
      if (l.sighting) {
        landmark.latest = std::move(l.sighting);
      }
    }
  }
  // Under the lock, the write to the feeder included (it does not block):
  // deactivate()'s release then cannot be overtaken by this frame's step.
  stepExposure(region);
}

std::optional<cv::Rect> ArucoDetector::groupRoi(const Group & group, const cv::Size & frame) const
{
  // Every landmark of the group seen in the last frame: around all of them.
  // One missing: the whole frame, so it can be found again.
  if (!roi_enabled_ || group.landmarks.empty()) {
    return std::nullopt;
  }
  std::optional<cv::Rect> roi;
  for (const auto & id : group.landmarks) {
    const auto & box = landmarks_.at(id).last_box;
    if (!box) {
      return std::nullopt;
    }
    const cv::Rect around = roiAround(*box, frame);
    roi = roi ? (*roi | around) : around;
  }
  return roi;
}

cv::Rect ArucoDetector::roiAround(const cv::Rect & box, const cv::Size & frame) const
{
  const int grow = cvRound(std::max(box.width, box.height) * roi_margin_);
  cv::Rect roi(box.x - grow, box.y - grow, box.width + 2 * grow, box.height + 2 * grow);
  const auto widen = [](int & origin, int & length, int minimum) {
    if (length < minimum) {
      origin -= (minimum - length) / 2;
      length = minimum;
    }
  };
  widen(roi.x, roi.width, roi_min_size_px_);
  widen(roi.y, roi.height, roi_min_size_px_);
  return roi & cv::Rect(cv::Point(0, 0), frame);
}

}  // namespace syncai_landmark_detection

PLUGINLIB_EXPORT_CLASS(
  syncai_landmark_detection::ArucoDetector, syncai_landmark_detection::LandmarkDetector)
