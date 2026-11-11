#include "syncai_landmark_detection/camera_exposure/exposure_parameters.hpp"

#include <cstdint>
#include <vector>

#include "syncai_landmark_detection/node_utils.hpp"

namespace syncai_landmark_detection
{
namespace camera_exposure
{

ExposureParams declareExposurePolicyParameters(
  const rclcpp::Node::SharedPtr & node, const std::string & prefix)
{
  // Every name sits under the plugin's own, as `<prefix>.exposure.<key>`.
  const auto full_name = [&](const std::string & key) { return prefix + ".exposure." + key; };
  const auto declare = [&](const std::string & key, const rclcpp::ParameterValue & value) {
    declare_parameter_if_not_declared(node, full_name(key), value);
  };
  const auto get = [&](const std::string & key) { return node->get_parameter(full_name(key)); };

  // The defaults are ExposureParams' own, where the reason for each value is kept.
  const ExposureParams defaults;
  declare(
    "ladder_us", rclcpp::ParameterValue(
                   std::vector<int64_t>(defaults.ladder_us.begin(), defaults.ladder_us.end())));
  declare("min_us", rclcpp::ParameterValue(defaults.min_us));
  declare("max_us", rclcpp::ParameterValue(defaults.max_us));
  declare("settle_frames", rclcpp::ParameterValue(defaults.settle_frames));
  declare("miss_frames", rclcpp::ParameterValue(defaults.miss_frames));
  declare("lost_frames", rclcpp::ParameterValue(defaults.lost_frames));
  declare("target_p95", rclcpp::ParameterValue(defaults.target_p95));
  declare("lock_p95_min", rclcpp::ParameterValue(defaults.lock_p95_min));
  declare("lock_p95_max", rclcpp::ParameterValue(defaults.lock_p95_max));
  declare("lock_max_saturated", rclcpp::ParameterValue(defaults.lock_max_saturated));
  declare("unlock_max_saturated", rclcpp::ParameterValue(defaults.unlock_max_saturated));
  declare("unlock_p95_min", rclcpp::ParameterValue(defaults.unlock_p95_min));
  declare("unlock_frames", rclcpp::ParameterValue(defaults.unlock_frames));
  declare("max_step", rclcpp::ParameterValue(defaults.max_step));

  // Copied first: as_integer_array() refers into the Parameter get() returns,
  // which C++17 destroys before a range-for over it starts, and a ladder read
  // that way began with the allocator's bookkeeping (16600, or a negative rung
  // that kept the node from starting).
  const std::vector<int64_t> ladder_us = get("ladder_us").as_integer_array();
  ExposureParams params;
  params.ladder_us.clear();
  for (const int64_t us : ladder_us) {
    params.ladder_us.push_back(static_cast<int>(us));
  }
  // The scalars are returned by value, so they are read directly.
  params.min_us = static_cast<int>(get("min_us").as_int());
  params.max_us = static_cast<int>(get("max_us").as_int());
  params.settle_frames = static_cast<int>(get("settle_frames").as_int());
  params.miss_frames = static_cast<int>(get("miss_frames").as_int());
  params.lost_frames = static_cast<int>(get("lost_frames").as_int());
  params.target_p95 = get("target_p95").as_double();
  params.lock_p95_min = get("lock_p95_min").as_double();
  params.lock_p95_max = get("lock_p95_max").as_double();
  params.lock_max_saturated = get("lock_max_saturated").as_double();
  params.unlock_max_saturated = get("unlock_max_saturated").as_double();
  params.unlock_p95_min = get("unlock_p95_min").as_double();
  params.unlock_frames = static_cast<int>(get("unlock_frames").as_int());
  params.max_step = get("max_step").as_double();
  return params;
}

}  // namespace camera_exposure
}  // namespace syncai_landmark_detection
