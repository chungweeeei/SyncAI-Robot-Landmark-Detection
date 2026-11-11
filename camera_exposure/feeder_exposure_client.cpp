#include "syncai_landmark_detection/camera_exposure/feeder_exposure_client.hpp"

#include <exception>
#include <utility>
#include <vector>

#include "syncai_landmark_detection/node_utils.hpp"

namespace syncai_landmark_detection
{
namespace camera_exposure
{

namespace
{

constexpr char kParameter[] = "exposure_override_us";
constexpr char kService[] = "/set_parameters";

}  // namespace

FeederExposureClient::FeederExposureClient(
  const rclcpp::Node::SharedPtr & node, rclcpp::CallbackGroup::SharedPtr callback_group,
  const std::string & feeder_node, double timeout_sec, std::string owner)
: logger_(node->get_logger()),
  owner_(std::move(owner)),
  feeder_node_(feeder_node),
  timeout_(timeout_sec)
{
  // The one service, not rclcpp::AsyncParametersClient: that opens six
  // clients on the feeder to use one.
  client_ = node->create_client<SetParameters>(
    feeder_node + kService, rmw_qos_profile_services_default, callback_group);
  // Runs only while a write is unanswered: a request to a feeder that died
  // after begin() gets no answer and no error.
  timeout_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(200), [this]() { onTimeoutCheck(); }, callback_group);
  timeout_timer_->cancel();
}

FeederExposureClient::~FeederExposureClient()
{
  timeout_timer_->cancel();
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & [key, pending] : pending_) {
    (void)key;
    client_->remove_pending_request(pending.request_id);
  }
}

bool FeederExposureClient::begin()
{
  std::lock_guard<std::mutex> lock(mutex_);
  written_us_ = 0;
  if (!client_->service_is_ready()) {
    usable_ = false;
    status_ = std::string(client_->get_service_name()) + " is not available";
    RCLCPP_WARN(
      logger_,
      "[FeederExposureClient][%s] %s: %s (is %s running? a bag replay has none): the camera's "
      "exposure is left as it is until this detector is woken again; detection goes on",
      __func__, owner_.c_str(), status_.c_str(), feeder_node_.c_str());
    return false;
  }
  usable_ = true;
  status_.clear();
  return true;
}

bool FeederExposureClient::set(int exposure_us)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!usable_) {
    return false;
  }
  if (!send(exposure_us, false)) {
    giveUp(std::string(client_->get_service_name()) + " went away");
    return false;
  }
  written_us_ = exposure_us;
  return true;
}

void FeederExposureClient::release()
{
  std::lock_guard<std::mutex> lock(mutex_);
  usable_ = false;
  status_ = "released";
  if (written_us_ == 0) {
    return;  // the camera was never taken
  }
  if (!send(0, true)) {
    RCLCPP_ERROR(
      logger_,
      "[FeederExposureClient][%s] %s: %s is not available, so the camera stays at the manual "
      "%d us this detector set. %s",
      __func__, owner_.c_str(), client_->get_service_name(), written_us_, restoreHint().c_str());
  }
}

bool FeederExposureClient::usable() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return usable_;
}

std::string FeederExposureClient::status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

bool FeederExposureClient::send(int exposure_us, bool release)
{
  if (!client_->service_is_ready()) {
    return false;
  }
  auto request = std::make_shared<SetParameters::Request>();
  request->parameters.push_back(
    rclcpp::Parameter(kParameter, static_cast<int64_t>(exposure_us)).to_parameter_msg());
  const uint64_t key = next_key_++;
  // The answer cannot run before the entry below exists: it takes mutex_,
  // which the caller holds.
  const auto sent = client_->async_send_request(
    request,
    [this, key](rclcpp::Client<SetParameters>::SharedFuture future) { onResponse(key, future); });
  Pending pending;
  pending.request_id = sent.request_id;
  pending.exposure_us = exposure_us;
  pending.release = release;
  pending.sent_at = std::chrono::steady_clock::now();
  pending_.emplace(key, pending);
  timeout_timer_->reset();
  return true;
}

void FeederExposureClient::onResponse(
  uint64_t key, rclcpp::Client<SetParameters>::SharedFuture future)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = pending_.find(key);
  if (it == pending_.end()) {
    return;  // timed out already, and said so
  }
  const Pending pending = it->second;
  pending_.erase(it);

  std::string refusal;
  try {
    const auto response = future.get();
    if (response->results.empty()) {
      refusal = "an answer with no result";
    } else if (!response->results.front().successful) {
      refusal = "refused: " + response->results.front().reason;
    }
  } catch (const std::exception & e) {
    refusal = std::string("the call failed: ") + e.what();
  }
  if (refusal.empty()) {
    return;
  }
  if (pending.release) {
    RCLCPP_ERROR(
      logger_,
      "[FeederExposureClient][%s] %s: giving the camera back (%s 0) got %s, so it stays at the "
      "manual %d us this detector set. %s",
      __func__, owner_.c_str(), kParameter, refusal.c_str(), written_us_, restoreHint().c_str());
    return;
  }
  // One refused value is not a dead feeder: the next step may be accepted.
  RCLCPP_WARN(
    logger_, "[FeederExposureClient][%s] %s: %s %d on %s got %s; the exposure is unchanged",
    __func__, owner_.c_str(), kParameter, pending.exposure_us, feeder_node_.c_str(),
    refusal.c_str());
}

void FeederExposureClient::onTimeoutCheck()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = std::chrono::steady_clock::now();
  for (auto it = pending_.begin(); it != pending_.end();) {
    const std::chrono::duration<double> waited = now - it->second.sent_at;
    if (waited < timeout_) {
      ++it;
      continue;
    }
    const Pending pending = it->second;
    client_->remove_pending_request(pending.request_id);
    it = pending_.erase(it);
    if (pending.release) {
      RCLCPP_ERROR(
        logger_,
        "[FeederExposureClient][%s] %s: giving the camera back (%s 0) got no answer from %s in "
        "%s, so it may still be at the manual %d us this detector set. %s",
        __func__, owner_.c_str(), kParameter, feeder_node_.c_str(),
        formatSeconds(waited.count()).c_str(), written_us_, restoreHint().c_str());
    } else if (usable_) {
      giveUp(
        std::string("no answer from ") + feeder_node_ + " to " + kParameter + " " +
        std::to_string(pending.exposure_us) + " in " + formatSeconds(waited.count()));
    }
  }
  if (pending_.empty()) {
    timeout_timer_->cancel();
  }
}

void FeederExposureClient::giveUp(const std::string & why)
{
  usable_ = false;
  status_ = why;
  RCLCPP_WARN(
    logger_,
    "[FeederExposureClient][%s] %s: %s: the exposure is no longer adjusted until this detector "
    "is woken again (the camera is at %d us, or at the feeder's own setting if it restarted); "
    "detection goes on",
    __func__, owner_.c_str(), why.c_str(), written_us_);
}

std::string FeederExposureClient::restoreHint() const
{
  // The service's resolved name is the feeder's fully-qualified node name
  // plus "/set_parameters".
  std::string node_name = client_->get_service_name();
  const std::string suffix = kService;
  if (node_name.size() > suffix.size()) {
    node_name.erase(node_name.size() - suffix.size());
  }
  return "It goes back to auto exposure when " + feeder_node_ +
         " restarts, or with: ros2 param set " + node_name + " " + kParameter + " 0";
}

}  // namespace camera_exposure
}  // namespace syncai_landmark_detection
