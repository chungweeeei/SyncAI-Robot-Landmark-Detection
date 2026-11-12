#ifndef SYNCAI_LANDMARK_DETECTION__CAMERA_EXPOSURE__EXPOSURE_PARAMETERS_HPP_
#define SYNCAI_LANDMARK_DETECTION__CAMERA_EXPOSURE__EXPOSURE_PARAMETERS_HPP_

// The exposure policy's ROS parameters, declared and read in one place so the
// path from a params file to ExposureParams has a test of its own.

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "syncai_landmark_detection/camera_exposure/exposure_policy.hpp"

namespace syncai_landmark_detection
{
namespace camera_exposure
{

/**
 * @brief Declares `<prefix>.exposure.ladder_us` .. `<prefix>.exposure.max_step`
 * with ExposureParams' defaults, unless already declared, and returns their
 * values as the node holds them.
 * @param node The node the parameters live on
 * @param prefix The detector plugin's name, which namespaces its parameters
 * @return The values, unchecked: ExposurePolicy's constructor rejects what cannot work
 */
ExposureParams declareExposurePolicyParameters(
  const rclcpp::Node::SharedPtr & node, const std::string & prefix);

}  // namespace camera_exposure
}  // namespace syncai_landmark_detection

#endif  // SYNCAI_LANDMARK_DETECTION__CAMERA_EXPOSURE__EXPOSURE_PARAMETERS_HPP_
